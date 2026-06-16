#include <Arduino.h>
#include <EEPROM.h>
#if !defined(HAVE_HWSERIAL1)
#include <SoftwareSerial.h>
#endif
#include "HX711.h"

/*
  Arduino Nano force sensor node for AR4 MK4 Z-axis force control.

  Wiring default:
  - HX711 DOUT -> Nano pin 2
  - HX711 SCK  -> Nano pin 3
  - MAX485 RO  -> Nano pin 10 (SoftwareSerial RX)
  - MAX485 DI  -> Nano pin 11 (SoftwareSerial TX)
  - MAX485 DE and /RE tied together -> Nano pin 8
  - HX711 RATE pin to VCC for maximum sample rate, about 80 SPS.

  Nano -> Teensy packet:
  $FZ,<seq>,<millis>,<raw>,<gram>,<newton>,<status>*<xor>\n
  Teensy -> Nano command:
  $FCMD,TARE*<xor>\n
  Supported RS485 command payloads:
  FCMD,TARE
  FCMD,CAL,<known_gram>
  FCMD,SETFACTOR,<factor>
  FCMD,STREAM,0|1
  FCMD,READ
  FCMD,ERASE
*/

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

static const float FORCE_SIGN = 1.0f;
static const float GRAVITY = 9.80665f;
static const uint8_t TARE_SAMPLES = 20;
static const uint8_t CALIBRATION_SAMPLES = 20;
static const float FILTER_ALPHA = 0.35f;
static const uint32_t EEPROM_MAGIC = 0x465A4831UL;
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
static long lastRaw = 0;
static float lastGram = 0.0f;
static float lastNewton = 0.0f;

static char usbCommandBuffer[40];
static uint8_t usbCommandLength = 0;

static char rs485PayloadBuffer[80];
static char rs485ChecksumBuffer[3];
static uint8_t rs485PayloadLength = 0;
static uint8_t rs485ChecksumLength = 0;
static bool rs485FrameActive = false;
static bool rs485ChecksumActive = false;

static Stream &forceSerial() {
#if defined(HAVE_HWSERIAL1)
  return Serial1;
#else
  return rs485Serial;
#endif
}

static uint8_t xorChecksum(const char *payload) {
  uint8_t crc = 0;
  while (*payload != '\0') {
    crc ^= static_cast<uint8_t>(*payload++);
  }
  return crc;
}

static int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool parseHexByte(const char *text, uint8_t &value) {
  const int high = hexValue(text[0]);
  const int low = hexValue(text[1]);
  if (high < 0 || low < 0) {
    return false;
  }
  value = static_cast<uint8_t>((high << 4) | low);
  return true;
}

static void rs485WriteLine(const char *line) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(20);
  forceSerial().print(line);
  forceSerial().flush();
  delayMicroseconds(20);
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

static void sendPayload(const char *payload) {
  char packet[120];
  snprintf(packet, sizeof(packet), "$%s*%02X\n", payload, xorChecksum(payload));
  rs485WriteLine(packet);
}

static void sendAck(const char *command, const char *status) {
  char payload[64];
  snprintf(payload, sizeof(payload), "FACK,%s,%s", command, status);
  sendPayload(payload);
}

static void sendAckValue(const char *command, const char *status, float value, uint8_t digits) {
  char valueText[18];
  char payload[80];
  dtostrf(value, 0, digits, valueText);
  snprintf(payload, sizeof(payload), "FACK,%s,%s,%s", command, status, valueText);
  sendPayload(payload);
}

static void sendForcePacket(const char *status) {
  char payload[96];
  char gramText[16];
  char newtonText[16];

  dtostrf(lastGram, 0, 2, gramText);
  dtostrf(lastNewton, 0, 4, newtonText);

  snprintf(payload, sizeof(payload), "FZ,%lu,%lu,%ld,%s,%s,%s",
           static_cast<unsigned long>(sequenceNumber++),
           static_cast<unsigned long>(millis()),
           lastRaw,
           gramText,
           newtonText,
           status);
  sendPayload(payload);
}

static void printHelp() {
  Serial.println();
  Serial.println(F("HX711 Z-axis force node"));
  Serial.println(F("USB commands:"));
  Serial.println(F("  h            : help"));
  Serial.println(F("  t            : tare with no load"));
  Serial.println(F("  c <gram>     : calibrate using known load"));
  Serial.println(F("  s <factor>   : set scale factor manually"));
  Serial.println(F("  p            : toggle RS485 stream"));
  Serial.println(F("  r            : print one reading"));
  Serial.println(F("  e            : erase saved calibration"));
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
  filteredInitialized = false;
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
  if (!waitForHx711Ready(2000)) {
    return false;
  }

  scale.tare(TARE_SAMPLES);
  saveCalibration();
  filteredInitialized = false;
  return true;
}

static bool calibrateScale(float knownGram, float &factorOut) {
  if (knownGram <= 0.0f) {
    return false;
  }

  if (!waitForHx711Ready(2000)) {
    return false;
  }

  const long value = scale.get_value(CALIBRATION_SAMPLES);
  const float factor = static_cast<float>(value) / knownGram;
  if (factor == 0.0f || isnan(factor)) {
    return false;
  }

  scale.set_scale(factor);
  saveCalibration();
  filteredInitialized = false;
  factorOut = factor;
  return true;
}

static void updateForceReading() {
  lastRaw = scale.read();
  const float gram = FORCE_SIGN * ((static_cast<float>(lastRaw) - scale.get_offset()) / scale.get_scale());

  if (!filteredInitialized) {
    filteredGram = gram;
    filteredInitialized = true;
  } else {
    filteredGram += FILTER_ALPHA * (gram - filteredGram);
  }

  lastGram = filteredGram;
  lastNewton = (lastGram / 1000.0f) * GRAVITY;
}

static void printReading() {
  if (!scale.is_ready()) {
    Serial.println(F("HX711 not ready."));
    return;
  }

  updateForceReading();
  Serial.print(F("raw="));
  Serial.print(lastRaw);
  Serial.print(F(" gram="));
  Serial.print(lastGram, 2);
  Serial.print(F(" newton="));
  Serial.print(lastNewton, 4);
  Serial.print(F(" offset="));
  Serial.print(scale.get_offset());
  Serial.print(F(" scale="));
  Serial.println(scale.get_scale(), 6);
}

static void handleLocalCommand(char *command, bool fromRs485) {
  while (*command == ' ' || *command == '\t') {
    ++command;
  }

  if (*command == '\0') {
    return;
  }

  if (strncmp(command, "FCMD,", 5) == 0) {
    command += 5;
  }

  if (strcasecmp(command, "TARE") == 0 || command[0] == 't' || command[0] == 'T') {
    const bool ok = tareScale();
    if (fromRs485) sendAck("TARE", ok ? "OK" : "ERR");
    else Serial.println(ok ? F("Tare done.") : F("ERROR: HX711 not ready."));
    return;
  }

  if (strncasecmp(command, "CAL,", 4) == 0 || command[0] == 'c' || command[0] == 'C') {
    const char *valueText = (command[0] == 'c' || command[0] == 'C') ? command + 1 : command + 4;
    float factor = 0.0f;
    const bool ok = calibrateScale(atof(valueText), factor);
    if (fromRs485) sendAckValue("CAL", ok ? "OK" : "ERR", factor, 6);
    else {
      Serial.print(ok ? F("Calibration done. Scale factor = ") : F("ERROR: calibration failed. Factor = "));
      Serial.println(factor, 6);
    }
    return;
  }

  if (strncasecmp(command, "SETFACTOR,", 10) == 0 || command[0] == 's' || command[0] == 'S') {
    const char *valueText = (command[0] == 's' || command[0] == 'S') ? command + 1 : command + 10;
    const float factor = atof(valueText);
    const bool ok = factor != 0.0f && !isnan(factor);
    if (ok) {
      scale.set_scale(factor);
      saveCalibration();
      filteredInitialized = false;
    }
    if (fromRs485) sendAckValue("SETFACTOR", ok ? "OK" : "ERR", factor, 6);
    else Serial.println(ok ? F("Scale factor saved.") : F("ERROR: invalid scale factor."));
    return;
  }

  if (strncasecmp(command, "STREAM,", 7) == 0) {
    streamEnabled = atoi(command + 7) != 0;
    if (fromRs485) sendAck("STREAM", streamEnabled ? "ON" : "OFF");
    else Serial.println(streamEnabled ? F("RS485 stream ON") : F("RS485 stream OFF"));
    return;
  }

  if (strcasecmp(command, "READ") == 0 || command[0] == 'r' || command[0] == 'R') {
    if (scale.is_ready()) {
      updateForceReading();
    }
    if (fromRs485) sendForcePacket(calibrationLoaded ? "OK" : "UNCAL");
    else printReading();
    return;
  }

  if (strcasecmp(command, "ERASE") == 0 || command[0] == 'e' || command[0] == 'E') {
    eraseCalibration();
    if (fromRs485) sendAck("ERASE", "OK");
    else Serial.println(F("Saved calibration erased."));
    return;
  }

  if (!fromRs485 && (command[0] == 'h' || command[0] == 'H')) {
    printHelp();
  } else if (fromRs485) {
    sendAck("UNKNOWN", "ERR");
  } else {
    Serial.println(F("Unknown command. Send 'h' for help."));
  }
}

static void handleRs485Frame() {
  rs485PayloadBuffer[rs485PayloadLength] = '\0';
  rs485ChecksumBuffer[rs485ChecksumLength] = '\0';

  uint8_t received = 0;
  if (rs485ChecksumLength != 2 || !parseHexByte(rs485ChecksumBuffer, received)) {
    return;
  }

  if (received != xorChecksum(rs485PayloadBuffer)) {
    sendAck("CHECKSUM", "ERR");
    return;
  }

  handleLocalCommand(rs485PayloadBuffer, true);
}

static void resetRs485Parser() {
  rs485PayloadLength = 0;
  rs485ChecksumLength = 0;
  rs485FrameActive = false;
  rs485ChecksumActive = false;
}

static void pollRs485Commands() {
  while (forceSerial().available() > 0) {
    const char c = static_cast<char>(forceSerial().read());

    if (c == '$') {
      resetRs485Parser();
      rs485FrameActive = true;
      continue;
    }

    if (!rs485FrameActive) {
      continue;
    }

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (rs485ChecksumActive) {
        handleRs485Frame();
      }
      resetRs485Parser();
      continue;
    }

    if (c == '*') {
      rs485ChecksumActive = true;
      continue;
    }

    if (rs485ChecksumActive) {
      if (rs485ChecksumLength < sizeof(rs485ChecksumBuffer) - 1) {
        rs485ChecksumBuffer[rs485ChecksumLength++] = c;
      }
    } else if (rs485PayloadLength < sizeof(rs485PayloadBuffer) - 1) {
      rs485PayloadBuffer[rs485PayloadLength++] = c;
    }
  }
}

static void pollUsbCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      usbCommandBuffer[usbCommandLength] = '\0';
      handleLocalCommand(usbCommandBuffer, false);
      usbCommandLength = 0;
      continue;
    }

    if (usbCommandLength < sizeof(usbCommandBuffer) - 1) {
      usbCommandBuffer[usbCommandLength++] = c;
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
  pollRs485Commands();

  if (!scale.is_ready()) {
    return;
  }

  updateForceReading();

  if (streamEnabled) {
    sendForcePacket(calibrationLoaded ? "OK" : "UNCAL");
  }
}
