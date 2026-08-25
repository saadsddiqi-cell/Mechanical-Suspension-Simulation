/**
 * DYNA-RIDE Co-Simulator Arduino Firmware
 * Author: Antigravity
 * 
 * Hardware Requirements & Safety Notes:
 * -------------------------------------
 * 1. WARNING: DO NOT power high-torque servos (e.g. MG996R) directly from the Arduino 5V pin.
 *    Each MG996R can draw up to 1.5A under load. Doing so will reset the Arduino, cause severe
 *    motor delay, jitter, or permanently damage the microcontroller.
 * 2. Connect an external 5V (2A - 3A) DC power supply directly to the servo power pins (+).
 * 3. CRITICAL: Connect the Ground (GND) of the external power supply to the Ground (GND) of the Arduino.
 *    Without a common ground reference, the servo control signals will be corrupt and jitter erraticly.
 * 4. Connect Left Servo control wire to Digital Pin 9.
 *    Connect Right Servo control wire to Digital Pin 10.
 */

#include <Servo.h>

// Servo pin definitions
const int LEFT_SERVO_PIN = 9;
const int RIGHT_SERVO_PIN = 10;

// Servo objects
Servo leftServo;
Servo rightServo;

// Failsafe / Watchdog settings
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 1000; // Center servos if no data for 1 sec
const int NEUTRAL_ANGLE = 90;

// Serial Buffer settings
const int BUFFER_SIZE = 32;
char serialBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  // Initialize Serial Communication at high speed
  Serial.begin(115200);
  
  // Attach servos to pins
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  
  // Initialize servos to center/neutral position
  leftServo.write(NEUTRAL_ANGLE);
  rightServo.write(NEUTRAL_ANGLE);
  
  lastPacketTime = millis();
  
  // Send ready signal to Web GUI console
  Serial.println("[ARDUINO] Firmware initialized. Servos attached to D9/D10. Baud rate: 115200.");
  Serial.println("[ARDUINO] Failsafe watchdog active (1000ms threshold).");
}

void loop() {
  // 1. Parse incoming serial data using non-blocking character reading
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Check for packet terminator
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0'; // Null-terminate string
        parseAndSetServos(serialBuffer);
        bufferIndex = 0; // Reset buffer index
      }
    } 
    // Fill buffer if space allows (reserve room for null terminator)
    else if (bufferIndex < BUFFER_SIZE - 1) {
      serialBuffer[bufferIndex++] = c;
    }
  }

  // 2. Failsafe Safety Watchdog
  // If the USB is disconnected or GUI stops writing, return servos to neutral
  // to prevent continuous motor stalling and overheating.
  if (millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
    leftServo.write(NEUTRAL_ANGLE);
    rightServo.write(NEUTRAL_ANGLE);
    // Send feedback occasionally when in failsafe state
    static unsigned long lastFailsafeReport = 0;
    if (millis() - lastFailsafeReport > 3000) {
      Serial.println("[ARDUINO] WARNING: Failsafe triggered. No packets received. Servos centered.");
      lastFailsafeReport = millis();
    }
  }
}

/**
 * Parses a comma-separated coordinate packet: "LeftAngle,RightAngle"
 * Example: "90,95" -> sets Left to 90, Right to 95
 */
void parseAndSetServos(char* data) {
  // Find comma delimiter
  char* commaPtr = strchr(data, ',');
  if (commaPtr == NULL) {
    // Invalid packet format
    return;
  }
  
  // Split string
  *commaPtr = '\0'; // Temporarily split string at comma
  char* leftPart = data;
  char* rightPart = commaPtr + 1;
  
  // Convert parts to integers
  int leftAngle = atoi(leftPart);
  int rightAngle = atoi(rightPart);
  
  // Validate range (protect mechanical hardware from over-travel binding)
  leftAngle = constrain(leftAngle, 40, 140);
  rightAngle = constrain(rightAngle, 40, 140);
  
  // Write to servos
  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
  
  // Reset failsafe timer since a valid command was executed
  lastPacketTime = millis();
}
