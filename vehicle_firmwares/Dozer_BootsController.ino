/*
 * Dozer.ino — ESP-NOW receiver firmware (base station version)
 *
 * Converted from Bluepad32 direct-Bluetooth control. The vehicle now
 * receives the standard ControlPayload from the base station's ESP32 TX
 * over ESP-NOW. The existing motor logic is unchanged: byteToAxis()
 * returns the same ±512 range Bluepad32 used.
 *
 * Payload (8 bytes, same as tx_firmware.ino):
 *   ch[0]=ABS_X  ch[1]=ABS_Y  ch[2]=ABS_RX  ch[3]=ABS_RY
 *   ch[4]=HAT0X  ch[5]=HAT0Y        (0–254, centre = 127)
 *   buttons[0]: bit0=A bit1=B bit2=X bit3=Y bit4=L1 bit5=R1 bit6=L2 bit7=R2
 *   buttons[1]: bit0=ThumbL bit1=ThumbR
 *
 * Control mapping (same as before):
 *   Left stick Y   -> left drive motor
 *   Right stick Y  -> right drive motor
 *   D-pad up/down  -> blade lift (bladeTilt)
 *   D-pad left/right -> ripper motor
 *   L1/L2, R1/R2   -> left / right blade tilt
 *   X / B          -> ripper servo down / up
 *   Right thumb    -> lights toggle
 *
 * Board: ESP32 (Arduino-ESP32 core 2.x), library: ESP32Servo (Kevin Harrington)
 * Remove the Bluepad32 board package / library; use the standard ESP32 core.
 *
 * After flashing, copy the MAC printed on boot into vehicles.json on the Pi.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ── Pins ──────────────────────────────────────────────────────────────────
#define LT1 15
#define LT2 27
#define LT3 14

#define ripperServoPin 23

#define leftMotor0 4        // \ rear drive motor (left)
#define leftMotor1 2        // /
#define rightMotor0 12      // \ rear drive motor (right)
#define rightMotor1 13      // /

#define leftBladeTilt0 16   // \ "Aux1" on PCB (briefly on at power-up)
#define leftBladeTilt1 17   // /
#define rightBladeTilt0 5   // \ "Aux2" on PCB
#define rightBladeTilt1 18  // /
#define bladeTilt0 25       // \ "Aux3" on PCB
#define bladeTilt1 26       // /
#define ripperMotor0 33     // \ ripper motor
#define ripperMotor1 32     // /

// ── Protocol ──────────────────────────────────────────────────────────────
static constexpr uint8_t NUM_CHANNELS     = 6;
static constexpr uint8_t NUM_BUTTON_BYTES = 2;

struct __attribute__((packed)) ControlPayload {
  uint8_t ch[NUM_CHANNELS];
  uint8_t buttons[NUM_BUTTON_BYTES];
};

enum Channel : uint8_t { CH_X = 0, CH_Y, CH_RX, CH_RY, CH_HAT_X, CH_HAT_Y };

// buttons[0] bits
static constexpr uint8_t BTN_A  = 1 << 0;
static constexpr uint8_t BTN_B  = 1 << 1;
static constexpr uint8_t BTN_X  = 1 << 2;
static constexpr uint8_t BTN_Y  = 1 << 3;
static constexpr uint8_t BTN_L1 = 1 << 4;
static constexpr uint8_t BTN_R1 = 1 << 5;
static constexpr uint8_t BTN_L2 = 1 << 6;
static constexpr uint8_t BTN_R2 = 1 << 7;
// buttons[1] bits
static constexpr uint8_t BTN_THUMBL = 1 << 0;
static constexpr uint8_t BTN_THUMBR = 1 << 1;

// Convert a 0–254 channel byte (centre 127) back to Bluepad32's ±512 range.
static inline int byteToAxis(uint8_t b) {
  return ((int)b - 127) * 512 / 127;
}

// ── Config ────────────────────────────────────────────────────────────────
static constexpr unsigned long FAILSAFE_MS     = 500;  // fleet standard
static constexpr int           HAT_THRESHOLD   = 256;  // |axis| above this = pressed
static constexpr unsigned long SERVO_STEP_MS   = 25;   // 2° per step
static constexpr unsigned long LIGHT_DEBOUNCE  = 300;

// ── State ─────────────────────────────────────────────────────────────────
Servo ripperServo;

static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
static ControlPayload rxPayload;          // written by ESP-NOW callback
static volatile bool  rxNew = false;
static volatile unsigned long lastPacketTime = 0;

static bool failsafeActive = false;

static int  ripperServoValue = 90;
static bool moveRipperServoUp = false;
static bool moveRipperServoDown = false;
static unsigned long lastServoStep = 0;

static bool lightMode = false;
static unsigned long lightSwitchButtonTime = 0;

// ── Motor helpers ─────────────────────────────────────────────────────────
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

void processLeftThrottle(int axisYValue) {
  moveMotor(leftMotor0, leftMotor1, axisYValue / 2);
}

void processRightThrottle(int axisRYValue) {
  moveMotor(rightMotor0, rightMotor1, axisRYValue / 2);
}

// hatX/hatY are ±512-scaled: Y negative = up, X negative = left (evdev)
void processBladeTiltAndRipper(int hatX, int hatY) {
  if (hatY < -HAT_THRESHOLD) {            // up
    moveMotor(bladeTilt0, bladeTilt1, 255);
  } else if (hatY > HAT_THRESHOLD) {      // down
    moveMotor(bladeTilt0, bladeTilt1, -255);
  } else {
    moveMotor(bladeTilt0, bladeTilt1, 0);
  }

  if (hatX < -HAT_THRESHOLD) {            // left
    moveMotor(ripperMotor0, ripperMotor1, 255);
  } else if (hatX > HAT_THRESHOLD) {      // right
    moveMotor(ripperMotor0, ripperMotor1, -255);
  } else {
    moveMotor(ripperMotor0, ripperMotor1, 0);
  }
}

void processLights(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchButtonTime) > LIGHT_DEBOUNCE) {
    lightMode = !lightMode;
    digitalWrite(LT1, lightMode ? HIGH : LOW);
    lightSwitchButtonTime = millis();
  }
}

void stopAllMotors() {
  digitalWrite(rightBladeTilt0, LOW);
  digitalWrite(rightBladeTilt1, LOW);
  digitalWrite(leftBladeTilt0, LOW);
  digitalWrite(leftBladeTilt1, LOW);
  digitalWrite(bladeTilt0, LOW);
  digitalWrite(bladeTilt1, LOW);
  digitalWrite(leftMotor0, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(rightMotor0, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(ripperMotor0, LOW);
  digitalWrite(ripperMotor1, LOW);
  moveRipperServoUp = false;
  moveRipperServoDown = false;
}

// ── Control processing (one fresh payload) ────────────────────────────────
void processPayload(const ControlPayload& p) {
  const uint8_t b0 = p.buttons[0];
  const uint8_t b1 = p.buttons[1];

  // Throttle
  processLeftThrottle(byteToAxis(p.ch[CH_Y]));
  processRightThrottle(byteToAxis(p.ch[CH_RY]));

  // Blade tilt + ripper (D-pad arrives as HAT axes)
  processBladeTiltAndRipper(byteToAxis(p.ch[CH_HAT_X]), byteToAxis(p.ch[CH_HAT_Y]));

  // Lights
  processLights(b1 & BTN_THUMBR);

  // Right blade tilt
  if (b0 & BTN_R1) {
    digitalWrite(rightBladeTilt0, HIGH);
    digitalWrite(rightBladeTilt1, LOW);
  } else if (b0 & BTN_R2) {
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, HIGH);
  } else {
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, LOW);
  }

  // Left blade tilt
  if (b0 & BTN_L1) {
    digitalWrite(leftBladeTilt0, HIGH);
    digitalWrite(leftBladeTilt1, LOW);
  } else if (b0 & BTN_L2) {
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, HIGH);
  } else {
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, LOW);
  }

  // Ripper servo direction flags (stepped in loop())
  if (b0 & BTN_X) {
    moveRipperServoDown = true;
    moveRipperServoUp = false;
  } else if (b0 & BTN_B) {
    moveRipperServoUp = true;
    moveRipperServoDown = false;
  } else {
    moveRipperServoDown = false;
    moveRipperServoUp = false;
  }
}

// Time-based servo stepping, independent of packet rate
void updateRipperServo() {
  if (!moveRipperServoUp && !moveRipperServoDown) return;
  if (millis() - lastServoStep < SERVO_STEP_MS) return;
  lastServoStep = millis();

  if (moveRipperServoUp && ripperServoValue < 170) {
    ripperServoValue += 2;
    ripperServo.write(ripperServoValue);
  } else if (moveRipperServoDown && ripperServoValue > 10) {
    ripperServoValue -= 2;
    ripperServo.write(ripperServoValue);
  }
}

// ── ESP-NOW receive callback (Arduino-ESP32 core 2.x signature) ───────────
// On core 3.x change the first argument to: const esp_now_recv_info_t* info
void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(ControlPayload)) return;
  portENTER_CRITICAL(&rxMux);
  memcpy(&rxPayload, data, sizeof(ControlPayload));
  rxNew = true;
  lastPacketTime = millis();
  portEXIT_CRITICAL(&rxMux);
}

// ── Arduino setup ─────────────────────────────────────────────────────────
void setup() {
  // Set aux outputs low first (they pulse at power-up)
  pinMode(rightBladeTilt0, OUTPUT);
  pinMode(rightBladeTilt1, OUTPUT);
  digitalWrite(rightBladeTilt0, LOW);
  digitalWrite(rightBladeTilt1, LOW);

  Serial.begin(115200);

  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(ripperMotor0, OUTPUT);
  pinMode(ripperMotor1, OUTPUT);
  pinMode(leftBladeTilt0, OUTPUT);
  pinMode(leftBladeTilt1, OUTPUT);
  pinMode(bladeTilt0, OUTPUT);
  pinMode(bladeTilt1, OUTPUT);
  pinMode(LT1, OUTPUT);
  pinMode(LT2, OUTPUT);
  pinMode(LT3, OUTPUT);

  stopAllMotors();
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);
  digitalWrite(LT3, LOW);

  ripperServo.attach(ripperServoPin);
  ripperServo.write(ripperServoValue);

  // ESP-NOW needs WiFi in station mode (no AP)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[RX] FATAL: esp_now_init() failed.");
    while (true) { delay(1000); }
  }
  esp_now_register_recv_cb(onReceive);

  Serial.printf("[RX] Dozer ready. MAC: %s\n", WiFi.macAddress().c_str());

  lastPacketTime = millis();
}

// ── Arduino loop ──────────────────────────────────────────────────────────
void loop() {
  // Grab a fresh payload, if any
  ControlPayload p;
  bool fresh = false;
  portENTER_CRITICAL(&rxMux);
  if (rxNew) {
    p = rxPayload;
    rxNew = false;
    fresh = true;
  }
  portEXIT_CRITICAL(&rxMux);

  if (fresh) {
    failsafeActive = false;
    processPayload(p);
  }

  updateRipperServo();

  // Failsafe: stop everything if packets stop arriving
  if (!failsafeActive && (millis() - lastPacketTime) > FAILSAFE_MS) {
    failsafeActive = true;
    stopAllMotors();
    Serial.println("[RX] Failsafe: no packets, motors stopped.");
  }

  vTaskDelay(1);
}
