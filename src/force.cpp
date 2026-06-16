#include <Arduino.h>
#include <EEPROM.h>
#if !defined(HAVE_HWSERIAL1)
#include <SoftwareSerial.h>
#endif
#include "HX711.h"

/*
  Arduino Nano force sensor node for AR4 MK4 Z-axis force control.

  Wiring default:
  - HX711 DOUT -> Mega pin 2
  - HX711 SCK  -> Mega pin 3
  - MAX485 RO  -> Nano pin 10 (SoftwareSerial RX)\r\n  - MAX485 DI  -> Nano pin 11 (SoftwareSerial TX)\r\n  - MAX485 DE and /RE tied together -> Nano pin 8
  - HX711 RATE pin to VCC for maximum sample rate, about 80 SPS.

  RS485 packet sent to Teensy:
  $FZ,<seq>,<millis>,<raw>,<gram>,<newton>,<status>*<xor>\n

  USB Serial commands:
  h              help
  t              tare / zero with no load
  c <gram>       calibrate with a known load, for example: c 1000
  s <factor>     set scale factor manually
  p              toggle RS485 stream on/off
  r              print one reading
  e              erase saved calibration
*/

// ===================== Pin and serial configuration =====================
static const uint8_t HX711_DOUT_PIN = 2;
static const uint8_t HX711_SCK_PIN = 3;
static const uint8_t RS485_DE_RE_PIN = 8;
#if !defined(HAVE_HWSERIAL1)
static const uint8_t RS485_RX_PIN = 10;
static const uint8_t RS485_TX_PIN = 11;
SoftwareSerial rs485Serial(RS485_RX_PIN, RS485_TX_PIN);
#endif

static const unsigned long USB_BAUD = 115200;
#if defined(HAVE_HWSERIAL1)
static const unsigned long RS485_BAUD = 500000;
#else
static const unsigned long RS485_BAUD = 57600;
#endif

// Force direction. Change to -1 if compression/push force reads negative.
static const float FORCE_SIGN = 1.0f;
static const float GRAVITY = 9.80665f;

// Keep this low for max SPS. 1 raw sample per packet follows HX711 ready rate.
static const uint8_t TARE_SAMPLES = 20;
static const uint8_t CALIBRATION_SAMPLES = 20;

// Simple first-order low-pass. 1.0 disables smoothing, lower value smooths more.
static const float FILTER_ALPHA = 0.35f;

// EEPROM layout.
static const uint32_t EEPROM_MAGIC = 0x465A4831UL;  // "FZH1"
static const int EEPROM_ADDR = 0;

struct CalibrationData {
  uint32_t magic;
  float scaleFactor;
  long offset;
};

HX711 scale;

static CalibrationData cal;
static bool calibrationLoaded = false;
static bool streamEnabled = true;
static bool filteredInitialized = false;
static float filteredGram = 0.0f;
static uint32_t sequenceNumber = 0;

static char commandBuffer[40];
static uint8_t commandLength = 0;

static void rs485WriteLine(const char *line) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(10);
  #if defined(HAVE_HWSERIAL1)
  Serial1.print(line);
  Serial1.flush();
#else
  rs485Serial.print(line);
  rs485Serial.flush();
#endif
  delayMicroseconds(10);
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

static uint8_t xorChecksum(const char *payload) {
  uint8_t crc = 0;
  while (*payload != '\0') {
    crc ^= static_cast<uint8_t>(*payload++);
  }
  return crc;
}

static void sendPacket(long raw, float gram, float newton, const char *status) {
  char payload[96];
  char packet[112];
  char gramText[16];
  char newtonText[16];

  dtostrf(gram, 0, 2, gramText);
  dtostrf(newton, 0, 4, newtonText);

  snprintf(payload, sizeof(payload), "FZ,%lu,%lu,%ld,%s,%s,%s",
           static_cast<unsigned long>(sequenceNumber++),
           static_cast<unsigned long>(millis()),
           raw,
           gramText,
           newtonText,
           status);

  snprintf(packet, sizeof(packet), "$%s*%02X\n", payload, xorChecksum(payload));
  rs485WriteLine(packet);
}

static void printHelp() {
  Serial.println();
  Serial.println(F("HX711 Z-axis force node"));
  Serial.println(F("Commands:"));
  Serial.println(F("  h            : help"));
  Serial.println(F("  t            : tare with no load"));
  Serial.println(F("  c <gram>     : calibrate using known load"));
  Serial.println(F("  s <factor>   : set scale factor manually"));
  Serial.println(F("  p            : toggle RS485 stream"));
  Serial.println(F("  r            : print one reading"));
  Serial.println(F("  e            : erase saved calibration"));
  Serial.println();
  Serial.println(F("RS485: $FZ,<seq>,<ms>,<raw>,<gram>,<newton>,<status>*<xor>"));
  Serial.println(F("For max SPS, connect HX711 RATE pin to VCC. Default board rate is 10 SPS."));
  Serial.println();
}

static void saveCalibration() {
  cal.magic = EEPROM_MAGIC;
  cal.scaleFactor = scale.get_scale();
  cal.offset = scale.get_offset();
  EEPROM.put(EEPROM_ADDR, cal);
  calibrationLoaded = true;
}

static bool loadCalibration() {
  EEPROM.get(EEPROM_ADDR, cal);
  if (cal.magic != EEPROM_MAGIC || isnan(cal.scaleFactor) || cal.scaleFactor == 0.0f) {
    return false;
  }

  scale.set_scale(cal.scaleFactor);
  scale.set_offset(cal.offset);
  calibrationLoaded = true;
  return true;
}

static void eraseCalibration() {
  CalibrationData blank = {0, 0.0f, 0};
  EEPROM.put(EEPROM_ADDR, blank);
  calibrationLoaded = false;
  scale.set_scale(1.0f);
  scale.set_offset(0);
}

static bool waitForHx711Ready(unsigned long timeoutMs) {
  const unsigned long startMs = millis();
  while (!scale.is_ready()) {
    if (millis() - startMs >= timeoutMs) {
      return false;
    }
    delay(1);
  }
  return true;
}

static bool tareScale() {
  Serial.println(F("Tare: remove all load from the loadcell..."));
  if (!waitForHx711Ready(2000)) {
    Serial.println(F("ERROR: HX711 not ready."));
    return false;
  }

  scale.tare(TARE_SAMPLES);
  saveCalibration();
  filteredInitialized = false;
  Serial.print(F("Tare done. Offset = "));
  Serial.println(scale.get_offset());
  return true;
}

static bool calibrateScale(float knownGram) {
  if (knownGram <= 0.0f) {
    Serial.println(F("ERROR: known load must be greater than 0 gram."));
    return false;
  }

  Serial.println(F("Calibration step 1: remove all load."));
  Serial.println(F("Send 't' first if zero has not been set."));
  delay(1000);

  if (!waitForHx711Ready(2000)) {
    Serial.println(F("ERROR: HX711 not ready."));
    return false;
  }

  Serial.print(F("Calibration step 2: place known load = "));
  Serial.print(knownGram, 2);
  Serial.println(F(" gram."));
  Serial.println(F("Reading in 3 seconds..."));
  delay(3000);

  if (!waitForHx711Ready(2000)) {
    Serial.println(F("ERROR: HX711 not ready."));
    return false;
  }

  const long value = scale.get_value(CALIBRATION_SAMPLES);
  const float factor = static_cast<float>(value) / knownGram;
  scale.set_scale(factor);
  saveCalibration();
  filteredInitialized = false;

  Serial.print(F("Calibration done. Scale factor = "));
  Serial.println(factor, 6);
  Serial.println(F("If force sign is reversed, change FORCE_SIGN at top of force.cpp."));
  return true;
}

static void printReading() {
  if (!scale.is_ready()) {
    Serial.println(F("HX711 not ready."));
    return;
  }

  const long raw = scale.read();
  const float gram = FORCE_SIGN * ((static_cast<float>(raw) - scale.get_offset()) / scale.get_scale());
  const float newton = (gram / 1000.0f) * GRAVITY;

  Serial.print(F("raw="));
  Serial.print(raw);
  Serial.print(F(" gram="));
  Serial.print(gram, 2);
  Serial.print(F(" newton="));
  Serial.print(newton, 4);
  Serial.print(F(" offset="));
  Serial.print(scale.get_offset());
  Serial.print(F(" scale="));
  Serial.println(scale.get_scale(), 6);
}

static void handleCommand(char *command) {
  while (*command == ' ' || *command == '\t') {
    ++command;
  }

  if (*command == '\0') {
    return;
  }

  switch (command[0]) {
    case 'h':
    case 'H':
      printHelp();
      break;

    case 't':
    case 'T':
      tareScale();
      break;

    case 'c':
    case 'C':
      calibrateScale(atof(command + 1));
      break;

    case 's':
    case 'S': {
      const float factor = atof(command + 1);
      if (factor == 0.0f || isnan(factor)) {
        Serial.println(F("ERROR: invalid scale factor."));
      } else {
        scale.set_scale(factor);
        saveCalibration();
        filteredInitialized = false;
        Serial.print(F("Scale factor saved = "));
        Serial.println(factor, 6);
      }
      break;
    }

    case 'p':
    case 'P':
      streamEnabled = !streamEnabled;
      Serial.print(F("RS485 stream "));
      Serial.println(streamEnabled ? F("ON") : F("OFF"));
      break;

    case 'r':
    case 'R':
      printReading();
      break;

    case 'e':
    case 'E':
      eraseCalibration();
      Serial.println(F("Saved calibration erased."));
      break;

    default:
      Serial.println(F("Unknown command. Send 'h' for help."));
      break;
  }
}

static void pollUsbCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

void setup() {
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  Serial.begin(USB_BAUD);
#if defined(HAVE_HWSERIAL1)
  Serial1.begin(RS485_BAUD);
#else
  rs485Serial.begin(RS485_BAUD);
#endif

  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  scale.set_scale(1.0f);
  scale.set_offset(0);

  delay(300);
  printHelp();

  if (loadCalibration()) {
    Serial.print(F("Loaded calibration. Offset = "));
    Serial.print(scale.get_offset());
    Serial.print(F(", scale = "));
    Serial.println(scale.get_scale(), 6);
  } else {
    Serial.println(F("No saved calibration. Send 't', then 'c <known_gram>'."));
  }
}

void loop() {
  pollUsbCommands();

  if (!scale.is_ready()) {
    return;
  }

  const long raw = scale.read();
  const float gram = FORCE_SIGN * ((static_cast<float>(raw) - scale.get_offset()) / scale.get_scale());

  if (!filteredInitialized) {
    filteredGram = gram;
    filteredInitialized = true;
  } else {
    filteredGram += FILTER_ALPHA * (gram - filteredGram);
  }

  const float newton = (filteredGram / 1000.0f) * GRAVITY;
  const char *status = calibrationLoaded ? "OK" : "UNCAL";

  if (streamEnabled) {
    sendPacket(raw, filteredGram, newton, status);
  }
}

