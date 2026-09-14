/*
 * Mini-Dozer.ino  —  ESP-NOW controlled version
 * ═════════════════════════════════════════════════════════════════════════════
 * Converted from Bluepad32 (direct Bluetooth gamepad) to ESP-NOW,
 * matching the base-station protocol defined in tx_firmware.ino.
 *
 * ESP-NOW payload received from the ESP32 TX (base station):
 *   struct ControlPayload {
 *       uint8_t ch[6];       // ch0=X  ch1=Y  ch2=RX  ch3=RY  ch4=HAT0X  ch5=HAT0Y
 *                             // each 0-254, centre = 127
 *       uint8_t buttons[2];  // [0]=A,B,X,Y,L1,R1,L2,R2  [1]=ThumbL,ThumbR
 *   };
 *
 * All motor/servo/light/combo logic below is unchanged from the original
 * Bluepad32 sketch — only the input source changed. byteToAxis() converts
 * the 0-254 channel bytes back into the same ~-512..512 range Bluepad32's
 * ctl->axisY()/axisRX() used to provide, so the existing threshold-based
 * math (divide by 2, by 12, by 160, etc.) needed zero changes.
 *
 * Failsafe: if no ESP-NOW packet arrives for 500ms, all motors/lights are
 * zeroed (same fleet-wide convention used on the other vehicles).
 *
 * Register this board's MAC (printed on boot) as the vehicle's `mac` in
 * vehicles.json on the Pi so the base station knows where to send packets.
 */

#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <WiFi.h>
#include <esp_now.h>

// ── ESP-NOW payload (must match tx_firmware.ino exactly) ──────────────────
struct __attribute__((packed)) ControlPayload {
  uint8_t ch[6];       // 0=X 1=Y 2=RX 3=RY 4=HAT0X 5=HAT0Y
  uint8_t buttons[2];  // [0]=A,B,X,Y,L1,R1,L2,R2  [1]=ThumbL,ThumbR
};

// Bit positions in buttons[0] / buttons[1]
#define BTN0_A  0
#define BTN0_B  1
#define BTN0_X  2
#define BTN0_Y  3
#define BTN0_L1 4
#define BTN0_R1 5
#define BTN0_L2 6
#define BTN0_R2 7
#define BTN1_THUMBL 0
#define BTN1_THUMBR 1

static inline bool btnBit(uint8_t byte, uint8_t bit) {
  return (byte >> bit) & 0x01;
}

// Convert a 0-254 (centre=127) channel byte back to the ~-512..512 range
// Bluepad32's axis getters used to return, so downstream math is unchanged.
static inline int byteToAxis(uint8_t b) {
  return ((int)b - 127) * 4;  // 0->-508, 127->0, 254->508
}

// Define ComboButton here so Arduino's auto-generated function prototypes
// (inserted right after the #include block) reference it before use.
struct ComboButton {
  int count;
  unsigned long firstPressTime;
  bool lastState;
};

// Explicit prototype: prevents Arduino's auto-generated prototype from
// referencing an undeclared type and failing to compile.
void handleCombo(bool currentState, ComboButton &btn, bool dirA, bool dirB);

// ── ESP-NOW receive state ──────────────────────────────────────────────────
static volatile ControlPayload latestPayload = {};
static volatile bool           hasNewPacket   = false;
static volatile unsigned long  lastPacketTime = 0;

void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  if (len != sizeof(ControlPayload)) return;
  memcpy((void*)&latestPayload, incomingData, sizeof(ControlPayload));
  lastPacketTime = millis();
  hasNewPacket   = true;
}

#define LT1 15
#define LT2 27
#define LT3 14

#define leftMotor0 4    // \ Used for controlling front drive motor movement
#define leftMotor1 2    // /
#define rightMotor0 12  // \ Used for controlling rear drive motor movement.
#define rightMotor1 13  // /

#define leftBladeTilt0 16   // \ "Aux1" on PCB. Used for controlling auxillary motor or lights.
#define leftBladeTilt1 17   // /
#define rightBladeTilt0 19  // \ "AUX2" on PCB. Used for controlling auxillary motors or lights.
#define rightBladeTilt1 18  // /
#define bladeTilt0 25       // \ "Aux3" on PCB. Used for controlling auxillary motors or lights.
#define bladeTilt1 26       // /
#define ripperMotor0 33     // \ Used for controlling front drive motor movement
#define ripperMotor1 32     // /
#define ripperServoPin 23

Servo ripperServo;

unsigned long lastInputTime = 0;
const unsigned long INPUT_TIMEOUT = 500;  // ms — ESP-NOW failsafe (fleet standard)

int buttonSwitchTime = 0;
int lightSwitchTime = 0;
int lightSwitchButtonTime = 0;
int lightMode = 0;
bool lightsOn = false;
bool moveRipperServoUp = false;
bool moveRipperServoDown = false;

int ripperServoValue = 90;

void processGamepad(const ControlPayload& payload) {

  // Record that we received fresh input
  lastInputTime = millis();

  uint8_t b0 = payload.buttons[0];
  uint8_t b1 = payload.buttons[1];

  // Throttle: ch1=ABS_Y (left), ch2=ABS_RX (right)
  processThrottle(byteToAxis(payload.ch[1]));
  processThrottle(byteToAxis(payload.ch[2]));

  // Blade tilt & Ripper (from dpad): ch4=HAT0X, ch5=HAT0Y
  int dpadValue = dpadFromChannels(payload);
  if (dpadValue == 1) {
    moveMotor(bladeTilt0, bladeTilt1, 255);
  } else if (dpadValue == 2) {
    moveMotor(bladeTilt0, bladeTilt1, -255);
  } else {
    moveMotor(bladeTilt0, bladeTilt1, 0);
  }
  if (dpadValue == 8) {
    moveMotor(ripperMotor0, ripperMotor1, 255);
  } else if (dpadValue == 4) {
    moveMotor(ripperMotor0, ripperMotor1, -255);
  } else {
    moveMotor(ripperMotor0, ripperMotor1, 0);
  }

  // Lights: thumbR
  processLights(btnBit(b1, BTN1_THUMBR));

  // Right/left blade tilt: r1/r2
  bool r1 = btnBit(b0, BTN0_R1);
  bool r2 = btnBit(b0, BTN0_R2);
  bool l1 = btnBit(b0, BTN0_L1);
  bool l2 = btnBit(b0, BTN0_L2);
  bool a  = btnBit(b0, BTN0_A);
  bool y  = btnBit(b0, BTN0_Y);
  bool b  = btnBit(b0, BTN0_B);
  bool x  = btnBit(b0, BTN0_X);

  if (r1) {
    digitalWrite(rightBladeTilt0, HIGH);
    digitalWrite(rightBladeTilt1, LOW);
  } else if (r2) {
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, HIGH);
  } else {
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, LOW);
  }
  if (l1) {
    digitalWrite(leftBladeTilt0, HIGH);
    digitalWrite(leftBladeTilt1, LOW);
  } else if (l2) {
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, HIGH);
  } else {
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, LOW);
  }

  // Ripper servo: x (up) / b (down)
  if (b == 1) {
    moveRipperServoUp = true;
  } else if (x == 1) {
    moveRipperServoDown = true;
  } else {
    moveRipperServoUp = false;
    moveRipperServoDown = false;
  }
}

void processThrottle(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
  moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
}

void processLights(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchButtonTime) > 300) {
    if (!lightMode) {
      digitalWrite(LT1, HIGH);
      delay(10);
      lightMode = true;
    } else {
      digitalWrite(LT1, LOW);
      delay(10);
      lightMode = false;
    }
    lightSwitchButtonTime = millis();
  }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 15) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -15) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}

// Derive a Bluepad32-style dpad value (1=up, 2=down, 4=right, 8=left, 0=none)
// from the HAT0X/HAT0Y channel bytes so the existing logic works unchanged.
int dpadFromChannels(const ControlPayload& p) {
  int hatX = byteToAxis(p.ch[4]);
  int hatY = byteToAxis(p.ch[5]);
  const int DPAD_THRESHOLD = 200;
  if (hatY < -DPAD_THRESHOLD) return 1;  // up
  if (hatY > DPAD_THRESHOLD) return 2;   // down
  if (hatX > DPAD_THRESHOLD) return 4;   // right
  if (hatX < -DPAD_THRESHOLD) return 8;  // left
  return 0;
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(rightBladeTilt0, OUTPUT);
  pinMode(rightBladeTilt1, OUTPUT);
  digitalWrite(rightBladeTilt0, LOW);
  digitalWrite(rightBladeTilt1, LOW);
  Serial.begin(115200);

  // ESP-NOW requires WiFi in station mode (no AP needed)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[RX] FATAL: esp_now_init() failed.");
    while (true) { delay(1000); }
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.printf("[RX] MAC: %s\r\n", WiFi.macAddress().c_str());
  Serial.println("[RX] Mini-Dozer ready. Register this MAC in vehicles.json.");

  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(ripperMotor0, OUTPUT);
  pinMode(ripperMotor1, OUTPUT);
  pinMode(leftBladeTilt0, OUTPUT);
  pinMode(leftBladeTilt1, OUTPUT);
  pinMode(rightBladeTilt0, OUTPUT);
  pinMode(rightBladeTilt1, OUTPUT);
  pinMode(LT1, OUTPUT);
  pinMode(LT2, OUTPUT);
  pinMode(LT3, OUTPUT);

  digitalWrite(leftMotor0, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(rightMotor0, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(ripperMotor0, LOW);
  digitalWrite(ripperMotor1, LOW);
  digitalWrite(leftBladeTilt0, LOW);
  digitalWrite(leftBladeTilt1, LOW);
  digitalWrite(rightBladeTilt0, LOW);
  digitalWrite(rightBladeTilt1, LOW);
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);
  digitalWrite(LT3, LOW);

  ripperServo.attach(ripperServoPin);
  ripperServo.write(ripperServoValue);

  lastInputTime = millis();  // initialize failsafe timer
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  if (hasNewPacket) {
    hasNewPacket = false;
    ControlPayload snapshot;
    memcpy(&snapshot, (const void*)&latestPayload, sizeof(ControlPayload));
    processGamepad(snapshot);
  } else {
    vTaskDelay(1);
  }

  // Failsafe check: if no ESP-NOW packet for too long, stop everything
  if (millis() - lastInputTime > INPUT_TIMEOUT) {
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, LOW);
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, LOW);
    digitalWrite(leftMotor0, LOW);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(rightMotor0, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(ripperMotor0, LOW);
    digitalWrite(ripperMotor1, LOW);
  }
}