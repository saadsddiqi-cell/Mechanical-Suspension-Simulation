#include <Servo.h>

// =============================================================================
//  SUSPEN-SIM  —  Single Servo Firmware
//
//  Physical layout:
//    Servo → D9
//
//  Serial commands (115200 baud, newline-terminated):
//    X:<float>        — position metres, expected range ±2.0
//    P:<M>,<K>,<B>    — physics params (logged only)
//    T:<int>          — runtime trim e.g. T:-20  shifts S1 CW by 20°
//    C                — print current calibration to Serial Monitor
// =============================================================================

// ── Pins ─────────────────────────────────────────────────────────────────────
const int SERVO1_PIN = 9;    // servo
const int LED_PIN    = 13;

// ── Mechanical calibration ────────────────────────────────────────────────────
const int   BASE_ANGLE   = 87;      // neutral angle [°]
const float SCALE_FACTOR = 45.0f;   // °/m  →  ±2 m gives ±90° swing
const int   ANGLE_MIN    =  5;      // mechanical stop guard
const int   ANGLE_MAX    = 175;     // mechanical stop guard

// ── Direction ──────────────────────────────────────────────────────────────
//   DIR1 = true  → angle increases with x  (normal mount)
//   DIR1 = false → angle decreases with x  (mirrored mount)
const bool DIR1 = true;

// ── Trim (degrees) ─────────────────────────────────────────────────────────
//   Adjust via Serial Monitor with T: command, or set a starting value here.
int trimS1 = 0;

// ── Smoothing ─────────────────────────────────────────────────────────────────
const float SMOOTH_ALPHA  = 0.55f;

// ── Watchdog ──────────────────────────────────────────────────────────────────
const unsigned long WATCHDOG_MS    = 600UL;
const float         WATCHDOG_DECAY = 0.95f;

// ── Serial ────────────────────────────────────────────────────────────────────
const int BUF_SIZE = 48;
char      rxBuf[BUF_SIZE];
int       rxIdx = 0;

// ── State ─────────────────────────────────────────────────────────────────────
Servo         servo1;
float         smoothX       = 0.0f;
unsigned long lastPacketMs  = 0;
bool          everConnected = false;

// =============================================================================
//  xToAngle — single authoritative angle calculation
//  dir=true  : angle increases with x  (normal mount)
//  dir=false : angle decreases with x  (mirrored mount — negate x first)
// =============================================================================
int xToAngle(float x, bool dir, int trim) {
    float directed = dir ? x : -x;
    int   raw      = BASE_ANGLE + (int)(directed * SCALE_FACTOR) + trim;
    return constrain(raw, ANGLE_MIN, ANGLE_MAX);
}

void writeServo(float x) {
    int a1 = xToAngle(x, DIR1, trimS1);
    servo1.write(a1);
}

void printCalibration() {
    Serial.println(F("=== Calibration ==="));
    Serial.print(F("BASE="));         Serial.print(BASE_ANGLE);
    Serial.print(F("  SCALE="));      Serial.print(SCALE_FACTOR, 1);
    Serial.print(F("  ALPHA="));      Serial.println(SMOOTH_ALPHA, 2);
    Serial.print(F("DIR1="));         Serial.println(DIR1 ? "normal" : "inverted");
    Serial.print(F("trimS1="));       Serial.println(trimS1);
    Serial.print(F("Neutral="));      Serial.println(xToAngle(0, DIR1, trimS1));
    Serial.print(F("Angle limits: ")); Serial.print(ANGLE_MIN);
    Serial.print(F(" – "));           Serial.println(ANGLE_MAX);
}

// =============================================================================
//  Setup
// =============================================================================
void setup() {
    Serial.begin(115200);

    servo1.attach(SERVO1_PIN);

    // Move to calibrated neutral immediately on power-up
    servo1.write(xToAngle(0, DIR1, trimS1));

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    delay(500);
    printCalibration();
    Serial.println(F("SUSPEN-SIM SINGLE READY"));
    Serial.println(F("Tip: send C to print calibration, T:<s1> to trim live"));
}

// =============================================================================
//  Loop
// =============================================================================
void loop() {

    // ── Serial read ──────────────────────────────────────────────────────────
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            rxBuf[rxIdx] = '\0';
            if (rxIdx > 0) parsePacket(rxBuf);
            rxIdx = 0;
        } else if (rxIdx < BUF_SIZE - 1) {
            rxBuf[rxIdx++] = ch;
        } else {
            rxIdx = 0;
        }
    }

    // ── Watchdog: drift servo to neutral when Python goes silent ───────────────
    if (everConnected && (millis() - lastPacketMs > WATCHDOG_MS)) {
        smoothX *= WATCHDOG_DECAY;
        writeServo(smoothX);
        digitalWrite(LED_PIN, (millis() % 500) < 250 ? HIGH : LOW);
    }

    delayMicroseconds(200);
}

// =============================================================================
//  Packet parser
// =============================================================================
void parsePacket(const char* pkt) {

    // ── X:<float>  Position command ──────────────────────────────────────────
    if (pkt[0] == 'X' && pkt[1] == ':') {
        const char* numStr = pkt + 2;
        if (numStr[0] == '\0') { Serial.println(F("ERR X: empty")); return; }

        float x = atof(numStr);
        if (x < -4.0f || x > 4.0f) {
            Serial.print(F("ERR X: out of range ")); Serial.println(x, 4);
            return;
        }

        smoothX = SMOOTH_ALPHA * x + (1.0f - SMOOTH_ALPHA) * smoothX;
        writeServo(smoothX);

        lastPacketMs  = millis();
        everConnected = true;
        digitalWrite(LED_PIN, HIGH);

        int a1 = xToAngle(smoothX, DIR1, trimS1);
        Serial.print(F("OK x=")); Serial.print(x, 4);
        Serial.print(F(" sx=")); Serial.print(smoothX, 4);
        Serial.print(F(" a1=")); Serial.println(a1);

    // ── P:<M>,<K>,<B>  Physics params ────────────────────────────────────────
    } else if (pkt[0] == 'P' && pkt[1] == ':') {
        Serial.print(F("PARAMS ")); Serial.println(pkt + 2);

    // ── T:<int>  Live trim ──────────────────────────────────────────────────
    //    Example: T:-25  drops horn down 25°
    //             T:0    reset trim
    } else if (pkt[0] == 'T' && pkt[1] == ':') {
        const char* p = pkt + 2;
        int t1 = (int)strtol(p, nullptr, 10);
        // Wider clamp (±60°) to handle large spline offsets without reseating
        trimS1 = constrain(t1, -60, 60);
        writeServo(smoothX);
        Serial.print(F("TRIM S1=")); Serial.println(trimS1);
        Serial.print(F("Neutral="));  Serial.println(xToAngle(0, DIR1, trimS1));

    // ── C  Print calibration ──────────────────────────────────────────────────
    } else if (pkt[0] == 'C') {
        printCalibration();

    } else {
        Serial.print(F("ERR unknown: ")); Serial.println(pkt);
    }
}
