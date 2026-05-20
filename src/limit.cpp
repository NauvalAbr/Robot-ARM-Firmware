#include <Arduino.h>

// Define limit switch pins
const int LIMIT_SW_1 = 26;
const int LIMIT_SW_2 = 27;
const int LIMIT_SW_3 = 28;
const int LIMIT_SW_4 = 29;
const int LIMIT_SW_5 = 30;
const int LIMIT_SW_6 = 31;

// Array of limit switch pins
int limitSwitchPins[] = {LIMIT_SW_1, LIMIT_SW_2, LIMIT_SW_3, LIMIT_SW_4, LIMIT_SW_5, LIMIT_SW_6};
int numLimitSwitches = 6;

// For debouncing
unsigned long lastDebounceTime[6] = {0};
const unsigned long debounceDelay = 20; // milliseconds
int lastLimitSwitchState[6] = {HIGH};

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  delay(10);
  Serial.println();
  Serial.println("=== Limit Switch Test ===");
  Serial.println("Testing 6 Limit Switches on pins 26-31");
  Serial.println();

  // Initialize limit switch pins as INPUT_PULLUP
  // Limit switch pulled LOW, goes HIGH when activated (connected to ground)
  for (int i = 0; i < numLimitSwitches; i++) {
    pinMode(limitSwitchPins[i], INPUT_PULLUP);
  }

  Serial.println("Setup Complete. Monitoring Limit Switches...");
  Serial.println();
}

void loop() {
  // Read and display limit switch states
  Serial.print("Limit Switches: ");
  
  for (int i = 0; i < numLimitSwitches; i++) {
    int reading = digitalRead(limitSwitchPins[i]);
    
    // Debounce logic
    if (reading != lastLimitSwitchState[i]) {
      lastDebounceTime[i] = millis();
    }
    
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      // State is stable
      Serial.print("SW");
      Serial.print(i + 1);
      Serial.print(":");
      
      if (reading == HIGH) {
        Serial.print("[PRESSED]");
      } else {
        Serial.print("[OPEN]");
      }
      
      if (i < numLimitSwitches - 1) {
        Serial.print(" | ");
      }
    }
    
    lastLimitSwitchState[i] = reading;
  }
  
  Serial.println();
  delay(500); // Update every 500ms
}
