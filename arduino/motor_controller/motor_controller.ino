/*
 * Robot Navigator — Arduino Motor Controller + Safety Watchdog
 * 
 * Responsibilities:
 *   - PWM motor control (L298N / TB6612FNG)
 *   - Hardware watchdog (stops motors if no heartbeat from Pi)
 *   - Wheel encoder counting (interrupt-driven)
 *   - Battery voltage monitoring
 *   - Emergency stop button
 * 
 * Communication: JSON over Serial (115200 baud)
 *   Pi → Arduino:  {"cmd":"move","left":150,"right":150}
 *   Arduino → Pi:  {"enc_l":1234,"enc_r":1230,"battery":11.2,"estop":false}
 */

#include <ArduinoJson.h>

// ── Pin Definitions ────────────────────────────────────────────

// Motor A (Left)
#define MOTOR_A_EN   5    // PWM enable
#define MOTOR_A_IN1  6    // Direction 1
#define MOTOR_A_IN2  7    // Direction 2

// Motor B (Right)
#define MOTOR_B_EN   10   // PWM enable
#define MOTOR_B_IN1  8    // Direction 1
#define MOTOR_B_IN2  9    // Direction 2

// Encoders (interrupt pins)
#define ENCODER_L_A  2    // Left encoder channel A (interrupt 0)
#define ENCODER_L_B  4    // Left encoder channel B
#define ENCODER_R_A  3    // Right encoder channel A (interrupt 1)
#define ENCODER_R_B  12   // Right encoder channel B

// Battery voltage divider
#define BATTERY_PIN  A0   // Voltage divider: Vbat → 10k → A0 → 4.7k → GND

// Emergency stop button (active LOW with internal pullup)
#define ESTOP_PIN    11

// Status LED
#define LED_PIN      13

// ── Configuration ──────────────────────────────────────────────

#define SERIAL_BAUD     115200
#define WATCHDOG_TIMEOUT 2000    // ms — stop motors if no heartbeat
#define REPORT_INTERVAL  100     // ms — send state to Pi
#define BATTERY_R1       10000.0 // Voltage divider R1 (ohms)
#define BATTERY_R2       4700.0  // Voltage divider R2 (ohms)
#define BATTERY_VREF     5.0     // Arduino ADC reference voltage

// ── Global State ───────────────────────────────────────────────

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

unsigned long lastHeartbeat = 0;
unsigned long lastReport = 0;
bool emergencyStopped = false;
bool watchdogTriggered = false;

int currentLeftPWM = 0;
int currentRightPWM = 0;

// JSON buffer
StaticJsonDocument<256> inputDoc;
StaticJsonDocument<256> outputDoc;

// ── Setup ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(SERIAL_BAUD);
    
    // Motor pins
    pinMode(MOTOR_A_EN, OUTPUT);
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_EN, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    
    // Encoder pins
    pinMode(ENCODER_L_A, INPUT_PULLUP);
    pinMode(ENCODER_L_B, INPUT_PULLUP);
    pinMode(ENCODER_R_A, INPUT_PULLUP);
    pinMode(ENCODER_R_B, INPUT_PULLUP);
    
    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderRightISR, RISING);
    
    // E-stop button
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    
    // LED
    pinMode(LED_PIN, OUTPUT);
    
    // Start with motors stopped
    stopMotors();
    
    lastHeartbeat = millis();
    
    // Startup message
    Serial.println("{\"status\":\"ready\",\"msg\":\"Motor controller initialized\"}");
}

// ── Main Loop ──────────────────────────────────────────────────

void loop() {
    unsigned long now = millis();
    
    // 1. Check emergency stop button
    checkEmergencyStop();
    
    // 2. Check watchdog (no heartbeat from Pi)
    if (now - lastHeartbeat > WATCHDOG_TIMEOUT && !watchdogTriggered) {
        watchdogTriggered = true;
        stopMotors();
        Serial.println("{\"warning\":\"watchdog_timeout\",\"msg\":\"No heartbeat - motors stopped\"}");
    }
    
    // 3. Read serial commands from Pi
    if (Serial.available()) {
        readCommand();
    }
    
    // 4. Send periodic state report
    if (now - lastReport >= REPORT_INTERVAL) {
        sendReport();
        lastReport = now;
    }
    
    // 5. Blink LED based on state
    updateLED(now);
}

// ── Command Processing ─────────────────────────────────────────

void readCommand() {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() == 0) return;
    
    DeserializationError error = deserializeJson(inputDoc, input);
    if (error) {
        Serial.print("{\"error\":\"json_parse\",\"msg\":\"");
        Serial.print(error.c_str());
        Serial.println("\"}");
        return;
    }
    
    const char* cmd = inputDoc["cmd"];
    if (cmd == nullptr) return;
    
    if (strcmp(cmd, "move") == 0) {
        handleMove();
    } else if (strcmp(cmd, "stop") == 0) {
        stopMotors();
        sendAck("stopped");
    } else if (strcmp(cmd, "heartbeat") == 0) {
        lastHeartbeat = millis();
        watchdogTriggered = false;
        sendAck("heartbeat_ok");
    } else if (strcmp(cmd, "reset_estop") == 0) {
        emergencyStopped = false;
        sendAck("estop_reset");
    } else if (strcmp(cmd, "reset_encoders") == 0) {
        encoderLeftCount = 0;
        encoderRightCount = 0;
        sendAck("encoders_reset");
    }
}

void handleMove() {
    if (emergencyStopped) {
        Serial.println("{\"error\":\"estop_active\",\"msg\":\"Cannot move - emergency stop active\"}");
        return;
    }
    
    if (watchdogTriggered) {
        Serial.println("{\"error\":\"watchdog\",\"msg\":\"Send heartbeat first\"}");
        return;
    }
    
    int leftPWM = inputDoc["left"] | 0;
    int rightPWM = inputDoc["right"] | 0;
    
    setMotors(leftPWM, rightPWM);
    sendAck("moving");
}

// ── Motor Control ──────────────────────────────────────────────

void setMotors(int leftPWM, int rightPWM) {
    currentLeftPWM = constrain(leftPWM, -255, 255);
    currentRightPWM = constrain(rightPWM, -255, 255);
    
    // Left motor
    if (currentLeftPWM >= 0) {
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, LOW);
    } else {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, HIGH);
    }
    analogWrite(MOTOR_A_EN, abs(currentLeftPWM));
    
    // Right motor
    if (currentRightPWM >= 0) {
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, LOW);
    } else {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
    }
    analogWrite(MOTOR_B_EN, abs(currentRightPWM));
}

void stopMotors() {
    currentLeftPWM = 0;
    currentRightPWM = 0;
    analogWrite(MOTOR_A_EN, 0);
    analogWrite(MOTOR_B_EN, 0);
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
}

// ── Safety ─────────────────────────────────────────────────────

void checkEmergencyStop() {
    if (digitalRead(ESTOP_PIN) == LOW) {  // Button pressed (active LOW)
        if (!emergencyStopped) {
            emergencyStopped = true;
            stopMotors();
            Serial.println("{\"event\":\"estop\",\"msg\":\"Emergency stop activated!\"}");
        }
    }
}

// ── Sensors ────────────────────────────────────────────────────

float readBatteryVoltage() {
    int raw = analogRead(BATTERY_PIN);
    float voltage = (raw / 1023.0) * BATTERY_VREF;
    // Reverse voltage divider: Vbat = Vmeasured * (R1 + R2) / R2
    float batteryVoltage = voltage * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    return batteryVoltage;
}

// ── Encoder ISRs ───────────────────────────────────────────────

void encoderLeftISR() {
    if (digitalRead(ENCODER_L_B) == HIGH) {
        encoderLeftCount++;
    } else {
        encoderLeftCount--;
    }
}

void encoderRightISR() {
    if (digitalRead(ENCODER_R_B) == HIGH) {
        encoderRightCount++;
    } else {
        encoderRightCount--;
    }
}

// ── Reporting ──────────────────────────────────────────────────

void sendReport() {
    outputDoc.clear();
    outputDoc["enc_l"] = encoderLeftCount;
    outputDoc["enc_r"] = encoderRightCount;
    outputDoc["battery"] = readBatteryVoltage();
    outputDoc["estop"] = emergencyStopped;
    outputDoc["watchdog"] = watchdogTriggered;
    outputDoc["pwm_l"] = currentLeftPWM;
    outputDoc["pwm_r"] = currentRightPWM;
    
    serializeJson(outputDoc, Serial);
    Serial.println();
}

void sendAck(const char* msg) {
    outputDoc.clear();
    outputDoc["ack"] = msg;
    outputDoc["enc_l"] = encoderLeftCount;
    outputDoc["enc_r"] = encoderRightCount;
    outputDoc["battery"] = readBatteryVoltage();
    outputDoc["estop"] = emergencyStopped;
    
    serializeJson(outputDoc, Serial);
    Serial.println();
}

// ── LED Status ─────────────────────────────────────────────────

void updateLED(unsigned long now) {
    if (emergencyStopped) {
        // Fast blink = emergency
        digitalWrite(LED_PIN, (now / 100) % 2);
    } else if (watchdogTriggered) {
        // Slow blink = watchdog
        digitalWrite(LED_PIN, (now / 500) % 2);
    } else if (currentLeftPWM != 0 || currentRightPWM != 0) {
        // Solid = moving
        digitalWrite(LED_PIN, HIGH);
    } else {
        // Off = idle
        digitalWrite(LED_PIN, LOW);
    }
}
