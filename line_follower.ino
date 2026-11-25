// ============================================================
// AIROST IoT Robot: ESP32 + 4 IR Modules + L298N + TCS34725
//                   + OLED (SSD1306) + Gripper Servo + BT
// ============================================================
#include <Wire.h>
#include <BluetoothSerial.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

BluetoothSerial SerialBT;
Servo gripperServo;

// --- OLED Configuration (128x64 I2C, usually address 0x3C) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- TCS34725 Colour Sensor ---
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_24MS,
  TCS34725_GAIN_4X
);

// ============================================================
// 1. Pin Definitions (Matched to Your Layout)
// ============================================================
// 4 IR Sensor Modules (Left -> Right)
const int IR_PINS[4] = {36, 39, 34, 35}; // VP, VN, GPIO34, GPIO35

// Set to true if your IR modules output LOW (0) on the black line
const bool IR_ACTIVE_LOW = true;

// L298N Motor Driver
const int ENA = 26;  // Left Motor PWM (jumper cap removed)
const int IN1 = 27;
const int IN2 = 14;

const int ENB = 13;  // Right Motor PWM (jumper cap removed)
const int IN3 = 16;
const int IN4 = 17;

// Gripper Servo & Mode Button
const int SERVO_PIN = 18;
const int MODE_BTN  = 4; // Active LOW (wired to GND)

// Servo Angles (adjust to match your mechanical gripper)
const int GRIPPER_OPEN_ANGLE  = 30;
const int GRIPPER_CLOSE_ANGLE = 110;

// ============================================================
// 2. PID & Speed Parameters
// ============================================================
float Kp = 28.0;
float Ki = 0.0;
float Kd = 16.0;

const int BASE_SPEED = 145; // 0 - 255
const int MAX_SPEED  = 225;

float error = 0.0, lastError = 0.0, integral = 0.0;
const float INTEGRAL_MAX = 50.0;

// ============================================================
// 3. Timing & System State
// ============================================================
unsigned long lastPidTime   = 0;
unsigned long lastColorTime = 0;
unsigned long lastOledTime  = 0;

const unsigned long PID_INTERVAL_MS   = 10;  // 100 Hz PID loop
const unsigned long COLOR_INTERVAL_MS = 60;  // Poll RGB sensor every 60 ms
const unsigned long OLED_INTERVAL_MS  = 150; // Refresh OLED ~6.7 times/sec

enum RobotMode { AUTO_LINE_FOLLOW, MANUAL_CONTROL };
RobotMode currentMode = AUTO_LINE_FOLLOW;

enum TargetColor { NONE, RED_BALL_1, BLUE_BALL_2, GREEN_BALL_3 };
TargetColor targetDestination = RED_BALL_1;

String lastDetectedColorName = "NONE";

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("AIROST_ESP32_Bot");

  for (int i = 0; i < 4; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(MODE_BTN, INPUT_PULLUP);
  stopMotors();

  // Attach Gripper Servo
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);

  // Start Shared I2C Bus (SDA = 21, SCL = 22)
  Wire.begin(21, 22);

  // Initialize OLED (0x3C)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("WARNING: OLED not found at 0x3C");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("AIROST IoT Robot");
    display.println("Initializing...");
    display.display();
  }

  // Initialize TCS34725 (0x29)
  if (!tcs.begin()) {
    Serial.println("ERROR: TCS34725 not found! Check SDA/SCL & 3.3V.");
  }

  Serial.println("System Ready.");
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  checkModeButton();

  unsigned long now = millis();

  if (currentMode == AUTO_LINE_FOLLOW) {
    // 1. Check destination colour periodically
    if (now - lastColorTime >= COLOR_INTERVAL_MS) {
      lastColorTime = now;
      TargetColor detected = detectColor();
      if (detected != NONE && detected == targetDestination) {
        stopMotors();
        currentMode = MANUAL_CONTROL;
        SerialBT.println("Target reached -> MANUAL MODE");
      }
    }

    // 2. Run PID line-following at 100 Hz
    if (now - lastPidTime >= PID_INTERVAL_MS) {
      lastPidTime = now;
      runLineFollowPID();
    }
  } else {
    handleManualBluetooth();
  }

  // 3. Update OLED status screen
  if (now - lastOledTime >= OLED_INTERVAL_MS) {
    lastOledTime = now;
    updateOLED();
  }
}

// ============================================================
// 4-Sensor Weighted Error + PID Controller
// ============================================================
void runLineFollowPID() {
  // Symmetric weights for 4 sensors: Far-L, Mid-L, Mid-R, Far-R
  const int weights[4] = {-2, -1, 1, 2};
  int activeCount = 0;
  int weightedSum = 0;

  for (int i = 0; i < 4; i++) {
    int raw = digitalRead(IR_PINS[i]);
    bool onLine = IR_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);

    if (onLine) {
      weightedSum += weights[i];
      activeCount++;
    }
  }

  if (activeCount > 0) {
    error = (float)weightedSum / activeCount;
  } else {
    // Holds last steering error across 1 cm dashed gaps or line loss
    error = lastError;
  }

  integral = constrain(integral + error, -INTEGRAL_MAX, INTEGRAL_MAX);
  float derivative = error - lastError;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftSpeed  = constrain(BASE_SPEED + (int)correction, -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - (int)correction, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
}

// ============================================================
// TCS34725 Colour Detection
// ============================================================
TargetColor detectColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (c < 100) {
    lastDetectedColorName = "NONE";
    return NONE;
  }

  float rNorm = (float)r / c;
  float gNorm = (float)g / c;
  float bNorm = (float)b / c;

  if (rNorm > 0.45 && rNorm > gNorm * 1.3 && rNorm > bNorm * 1.3) {
    lastDetectedColorName = "RED (Ball 1)";
    return RED_BALL_1;
  }
  if (bNorm > 0.40 && bNorm > rNorm * 1.2 && bNorm > gNorm * 1.2) {
    lastDetectedColorName = "BLUE (Ball 2)";
    return BLUE_BALL_2;
  }
  if (gNorm > 0.40 && gNorm > rNorm * 1.2 && gNorm > bNorm * 1.2) {
    lastDetectedColorName = "GREEN (Ball 3)";
    return GREEN_BALL_3;
  }

  lastDetectedColorName = "NONE";
  return NONE;
}

// ============================================================
// L298N Motor Control
// ============================================================
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }

  // Right Motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
}

void stopMotors() {
  setMotors(0, 0);
}

// ============================================================
// Bluetooth Manual Mode & Gripper Commands
// Commands: F, B, L, R, S, M, O (Open), C (Close), 1/2/3 (Target)
// ============================================================
void handleManualBluetooth() {
  if (!SerialBT.available()) return;

  char cmd = SerialBT.read();
  switch (cmd) {
    case 'F': setMotors(165, 165);   break;
    case 'B': setMotors(-165, -165); break;
    case 'L': setMotors(-135, 135);  break;
    case 'R': setMotors(135, -135);  break;
    case 'S': stopMotors();          break;

    case 'O': // Open Gripper
      gripperServo.write(GRIPPER_OPEN_ANGLE);
      SerialBT.println("Gripper OPEN");
      break;
    case 'C': // Close Gripper
      gripperServo.write(GRIPPER_CLOSE_ANGLE);
      SerialBT.println("Gripper CLOSED");
      break;

    case 'M': // Return to Autonomous Line-Following
      integral = 0.0;
      lastError = 0.0;
      currentMode = AUTO_LINE_FOLLOW;
      SerialBT.println("Switched to AUTO MODE");
      break;

    case '1': targetDestination = RED_BALL_1;   SerialBT.println("Target: RED");   break;
    case '2': targetDestination = BLUE_BALL_2;  SerialBT.println("Target: BLUE");  break;
    case '3': targetDestination = GREEN_BALL_3; SerialBT.println("Target: GREEN"); break;
  }
}

// ============================================================
// Push-Button Mode Toggle
// ============================================================
void checkModeButton() {
  static unsigned long lastPress = 0;
  if (digitalRead(MODE_BTN) == LOW && (millis() - lastPress > 300)) {
    lastPress = millis();
    stopMotors();
    if (currentMode == AUTO_LINE_FOLLOW) {
      currentMode = MANUAL_CONTROL;
      SerialBT.println("Button -> MANUAL MODE");
    } else {
      integral = 0.0;
      lastError = 0.0;
      currentMode = AUTO_LINE_FOLLOW;
      SerialBT.println("Button -> AUTO MODE");
    }
  }
}

// ============================================================
// OLED Status Display
// ============================================================
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print("Mode: ");
  display.println(currentMode == AUTO_LINE_FOLLOW ? "AUTO PID" : "MANUAL");

  display.print("Target: ");
  if (targetDestination == RED_BALL_1)        display.println("RED (1)");
  else if (targetDestination == BLUE_BALL_2)  display.println("BLUE (2)");
  else if (targetDestination == GREEN_BALL_3) display.println("GREEN (3)");
  else display.println("NONE");

  display.print("RGB: ");
  display.println(lastDetectedColorName);

  display.print("Err: ");
  display.println(error, 2);

  display.display();
}
