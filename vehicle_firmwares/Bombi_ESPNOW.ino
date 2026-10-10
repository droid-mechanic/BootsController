/*
 * Bombi_ESPNOW.ino
 * ════════════════
 * Bombi vehicle firmware — ESP-NOW receiver for the Base Station.
 *
 * Converted from Bluepad32 direct-Bluetooth control. The vehicle now
 * receives the standard 8-byte ControlPayload from the fleet transmitter:
 *
 *   ch[0]=ABS_X  ch[1]=ABS_Y  ch[2]=ABS_RX  ch[3]=ABS_RY
 *   ch[4]=HAT0X  ch[5]=HAT0Y        (0–254, centre = 127)
 *   buttons[0]  bit0=A bit1=B bit2=X bit3=Y bit4=L1 bit5=R1 bit6=L2 bit7=R2
 *   buttons[1]  bit0=ThumbL bit1=ThumbR
 *
 * D-pad travels on the HAT channels (not in the button bitmask).
 * Failsafe: all motors stop if no packet arrives for 500 ms.
 *
 * After flashing, open the Serial Monitor: the MAC address is printed on
 * boot. Register it in vehicles.json on the Pi.
 *
 * Board / libs: ESP32 (Arduino-ESP32 core 2.x or 3.x), ESP32Servo, Cdrv8833.
 * (Bluepad32 is no longer required — use a regular ESP32 board package,
 *  not the Bluepad32 one, or leave it; it just won't be used.)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include "Cdrv8833.h"

// ── Pins ──────────────────────────────────────────────────────────────────
#define rightMotor0 33
#define rightMotor0Dir HIGH
#define rightMotor1 32
#define rightMotor1Dir HIGH

#define leftMotor0 26
#define leftMotor0Dir HIGH
#define leftMotor1 25
#define leftMotor1Dir HIGH

#define armMotor0 21
#define armMotor0Dir HIGH
#define armMotor1 19
#define armMotor1Dir HIGH

#define bucketServoPin 23
#define clawServoPin 22

#define auxLights0 5
#define auxLights1 18

Cdrv8833 rightMotor;
Cdrv8833 leftMotor;
Cdrv8833 armMotor;

Servo bucketServo;
Servo clawServo;

// ── Protocol ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) ControlPayload {
  uint8_t ch[6];
  uint8_t buttons[2];
};

static constexpr uint32_t FAILSAFE_MS = 500;

// Written from the ESP-NOW callback (WiFi task), read from loop().
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
static ControlPayload rxPayload = {{127, 127, 127, 127, 127, 127}, {0, 0}};
static volatile uint32_t lastRxMs = 0;
static volatile bool rxFresh = false;

// ── State ─────────────────────────────────────────────────────────────────
// Pin starts LOW, so the lights are really off at boot; flag matches that.
static bool auxLightsOn = false;
static bool dpadUpPrev = false;
static bool failsafeActive = true;

// ── Helpers ───────────────────────────────────────────────────────────────

// 0–254 (centre 127) → roughly -512..+512, matching Bluepad32 axis range
// so the original threshold logic is unchanged.
static int byteToAxis(uint8_t b) {
  int v = ((int)b - 127) * 512 / 127;
  return constrain(v, -512, 511);
}

static void stopAll() {
  rightMotor.stop();
  leftMotor.stop();
  armMotor.stop();
}

// ── ESP-NOW receive callback ──────────────────────────────────────────────
static void handlePacket(const uint8_t* data, int len) {
  if (len != (int)sizeof(ControlPayload)) return;
  portENTER_CRITICAL(&rxMux);
  memcpy(&rxPayload, data, sizeof(ControlPayload));
  lastRxMs = millis();
  rxFresh = true;
  portEXIT_CRITICAL(&rxMux);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  handlePacket(data, len);
}
#else
void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
  handlePacket(data, len);
}
#endif

// ── Control logic (same behaviour as the Bluepad32 version) ───────────────
static void processPayload(const ControlPayload& p) {
  int LYValue = byteToAxis(p.ch[1]);   // ABS_Y  (left stick Y, up = negative)
  int RYValue = byteToAxis(p.ch[3]);   // ABS_RY (right stick Y)

  int8_t leftMotorSpeed  = map(LYValue, -512, 511, -100, 100);
  int8_t rightMotorSpeed = map(RYValue, -512, 511, -100, 100);

  leftMotor.move(leftMotorSpeed);
  rightMotor.move(rightMotorSpeed);

  if (LYValue > -2 && LYValue < 2 && RYValue > -2 && RYValue < 2) {
    rightMotor.stop();
    leftMotor.stop();
  }

  // D-pad up (HAT0Y: 0 = up, 127 = centre, 254 = down) toggles aux lights.
  // Edge-detected so holding the button toggles once, not every packet.
  bool dpadUp = p.ch[5] < 64;
  if (dpadUp && !dpadUpPrev) {
    auxLightsOn = !auxLightsOn;
    digitalWrite(auxLights1, auxLightsOn ? HIGH : LOW);
  }
  dpadUpPrev = dpadUp;
}

// ── Arduino setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  rightMotor.init(rightMotor0, rightMotor1, 5);
  rightMotor.setDecayMode(drv8833DecaySlow);
  rightMotor.swapDirection(false);
  leftMotor.init(leftMotor0, leftMotor1, 6);
  leftMotor.setDecayMode(drv8833DecaySlow);
  leftMotor.swapDirection(true);
  armMotor.init(armMotor0, armMotor1, 7);
  armMotor.setDecayMode(drv8833DecaySlow);

  pinMode(auxLights0, OUTPUT);
  pinMode(auxLights1, OUTPUT);
  digitalWrite(auxLights0, HIGH);
  digitalWrite(auxLights1, LOW);

  stopAll();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: esp_now_init() failed.");
    while (true) { delay(1000); }
  }
  esp_now_register_recv_cb(onReceive);

  Serial.printf("Bombi MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("Ready. Waiting for ESP-NOW packets.");
}

// ── Arduino loop ──────────────────────────────────────────────────────────
void loop() {
  ControlPayload p;
  bool fresh;
  uint32_t last;

  portENTER_CRITICAL(&rxMux);
  p = rxPayload;
  fresh = rxFresh;
  rxFresh = false;
  last = lastRxMs;
  portEXIT_CRITICAL(&rxMux);

  if (fresh) {
    failsafeActive = false;
    processPayload(p);
  } else if (!failsafeActive && (millis() - last) > FAILSAFE_MS) {
    failsafeActive = true;
    stopAll();
    dpadUpPrev = false;
    Serial.println("Failsafe: no packets, motors stopped.");
  }

  delay(1);
}
