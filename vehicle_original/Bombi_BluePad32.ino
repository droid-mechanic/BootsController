#include <Arduino.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>
#include "Cdrv8833.h"

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
Cdrv8833 leftMotorB;
Cdrv8833 rightMotorB;

Servo bucketServo;
Servo clawServo;


volatile bool auxLightsOn = true;

ControllerPtr controller;

void onConnectedController(ControllerPtr ctl) {
  if (controller == nullptr) {
    Serial.printf("CALLBACK: Controller is connected");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    controller = ctl;
    ctl->setColorLED(255, 0, 0);
    ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */);
  } else {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (controller == ctl) {
    Serial.printf("CALLBACK: Controller disconnected");
    controller = nullptr;
  } else {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  int LYValue = ctl->axisY();
  int RYValue = ctl->axisRY();

  int8_t leftMotorSpeed = map(LYValue, -512, 511, -100, 100);
  int8_t rightMotorSpeed = map(RYValue, -512, 511, -100, 100);

  leftMotor.move(leftMotorSpeed);
  rightMotor.move(rightMotorSpeed);

  if (LYValue > -2 && LYValue < 2 && RYValue > -2 && RYValue < 2) {
    rightMotor.stop();
    leftMotor.stop();
  }

  Serial.println(ctl->dpad());



  if (ctl->dpad() == 1) {
    if (!auxLightsOn) {
      //digitalWrite(auxLights0, HIGH);
      digitalWrite(auxLights1, HIGH);
      auxLightsOn = true;
    } else {
      //digitalWrite(auxLights0, LOW);
      digitalWrite(auxLights1, LOW);
      auxLightsOn = false;
    }
  }
}

void processControllers() {
  if (controller && controller->isConnected() && controller->hasData()) {
    if (controller->isGamepad()) {
      processGamepad(controller);
    } else {
      Serial.println("Unsupported controller");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  Serial.println("Ready.");

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
  //digitalWrite(auxLights1, HIGH);
}

void loop() {
  unsigned long currentTime = millis();
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
}