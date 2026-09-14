//If nothing shows up under "Ports" when the esp32 is plugged in first verify that your usb cable has data lines. Some cables only cary power.
//Next look into installing CP210x Drivers. Youtube tutorial here: https://www.youtube.com/watch?v=BtSE0TeybfI
#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>
//25,26,32,33,21,19,22,23,2,4,17,16

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/*What the different Serial commands for the trailer esp32 daughter board do
1-Trailer Legs Up
2-Trailer Legs Down
3-Ramp Up
4-Ramp Down
5-auxMotor1 Forward
6-auxMotor1 Reverse
7-auxMotor1 STOP
8-auxMotor2 Forward
9-auxMotor2 Reverse
10-auxMotor2 STOP
11- LT1 LOW
12- LT1 HIGH
13- LT2 LOW
14- LT2 HIGH
15- LT3 LOW
16- LT3 HIGH
*/
#define LT1 15
#define LT2 27
#define LT3 14

#define RX0 3
#define TX0 1

#define frontSteeringServoPin 23
#define hitchServoPin 22
#define clawServoPin 21

Servo frontSteeringServo;
Servo clawServo;

#define frontMotor0 33  // \ Used for controlling front drive motor movement
#define frontMotor1 32  // /
#define rearMotor0 2    // \ Used for controlling rear drive motor movement
#define rearMotor1 4    // /
#define rearMotor2 12   // \ Used for controlling second rear drive motor movement.
#define rearMotor3 13   // /

#define auxAttach0 17  // \ "AUX1" on PCB. Used for controlling auxillary motors or lights.
#define auxAttach1 16  // /
#define auxAttach2 5   // \ "Aux2" on PCB. Used for controlling auxillary motor or lights.  Keep in mind this will always breifly turn on when the model is powered on.
#define auxAttach3 18  // /
#define auxAttach4 25  // \ "Aux3" on PCB. Used for controlling auxillary motors or lights.
#define auxAttach5 26  // /


int lightSwitchButtonTime = 0;
int loaderModeButtonTime = 0;
int lightSwitchTime = 0;
int adjustedSteeringValue = 90;
int hitchServoValue = 160;
int steeringTrim = 0;
int lightMode = 0;
int clawServoValue = 90;
int servoDelay = 0;
bool lightsOn = false;
bool auxLightsOn = false;
bool blinkLT = false;
bool hazardLT = false;
bool hazardsOn = false;
bool moveClawServoUp = false;
bool moveClawServoDown = false;
bool loaderMode = false;
bool hitchUp = true;

bool trailerAuxMtr1Forward = false;
bool trailerAuxMtr1Reverse = false;
bool trailerAuxMtr2Forward = false;
bool trailerAuxMtr2Reverse = false;


void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  //Throttle
  processThrottleDipper(ctl->axisY());
  //Steering
  processSteeringGrapple(ctl->axisRX());
  //Steering trim and hitch
  processTrimAndHitch(ctl->dpad());
  //Lights
  processLights(ctl->thumbR());
  processLoaderMode(ctl->thumbL());

  //TrailerServos
  if (!loaderMode) {
    processTrailerLegsUp(ctl->y());
    processTrailerLegsDown(ctl->a());
  }
  processTrailerRampUp(ctl->b());
  processTrailerRampDown(ctl->x());
  //TrailerMTRs
  processTrailerAuxMtr1Forward(ctl->r2());
  processTrailerAuxMtr1Reverse(ctl->r1());
  processTrailerAuxMtr2Forward(ctl->l2());
  processTrailerAuxMtr2Reverse(ctl->l1());

  //Loader
  processPivot(ctl->axisX());
  processBoom(ctl->axisRY());

  if (blinkLT && (millis() - lightSwitchTime) > 300) {
    if (!lightsOn) {
      if (adjustedSteeringValue <= 70) {
        digitalWrite(LT1, HIGH);
        Serial.println(12);
      } else if (adjustedSteeringValue >= 110) {
        digitalWrite(LT2, HIGH);
        Serial.println(14);
      }
      lightsOn = true;
    } else {
      if (adjustedSteeringValue <= 70) {
        digitalWrite(LT2, HIGH);
        digitalWrite(LT1, LOW);
        Serial.println(11);
        delay(10);
        Serial.println(14);
      } else if (adjustedSteeringValue >= 110) {
        digitalWrite(LT1, HIGH);
        digitalWrite(LT2, LOW);
        Serial.println(13);
        delay(10);
        Serial.println(12);
      }
      lightsOn = false;
    }
    lightSwitchTime = millis();
  }
  if (blinkLT && adjustedSteeringValue > 70 && adjustedSteeringValue < 110) {
    digitalWrite(LT1, HIGH);
    digitalWrite(LT2, HIGH);
    Serial.println(12);
    delay(10);
    Serial.println(14);
  }
  if (hazardLT && (millis() - lightSwitchTime) > 300) {
    if (!hazardsOn) {
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      Serial.println(12);
      delay(10);
      Serial.println(14);
      hazardsOn = true;
    } else {
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      Serial.println(11);
      delay(10);
      Serial.println(13);
      hazardsOn = false;
    }
    lightSwitchTime = millis();
  }
  if (ctl->a() == 1) {
    moveClawServoDown = true;
  } else if (ctl->y() == 1) {
    moveClawServoUp = true;
  } else {
    moveClawServoDown = false;
    moveClawServoUp = false;
  }
  if (loaderMode) {
    if (moveClawServoUp) {
      if (servoDelay == 2) {
        if (clawServoValue >= 10 && clawServoValue < 170) {
          //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
          clawServoValue = clawServoValue + 2;
          clawServo.write(clawServoValue);
        }
        servoDelay = 0;
      }
      servoDelay++;
    }
    if (moveClawServoDown) {
      if (servoDelay == 2) {
        if (clawServoValue <= 170 && clawServoValue > 10) {
          //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
          clawServoValue = clawServoValue - 2;
          clawServo.write(clawServoValue);
        }
        servoDelay = 0;
      }
      servoDelay++;
    }
  }
}
void processTrailerLegsUp(int value) {
  if (value == 1) {
    Serial.println(1);
    delay(10);
  }
}
void processTrailerLegsDown(int value) {
  if (value == 1) {
    Serial.println(2);
    delay(10);
  }
}
void processTrailerRampUp(int value) {
  if (value == 1) {
    Serial.println(3);
    delay(10);
  }
}
void processTrailerRampDown(int value) {
  if (value == 1) {
    Serial.println(4);
    delay(10);
  }
}
void processPivot(int axisXValue) {
  int adjustedThrottleValue = axisXValue / 2;
  if (loaderMode && (adjustedThrottleValue > 90 || adjustedThrottleValue < -90)) {
    moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue - 90);
  }
  if (loaderMode) {
    if (adjustedThrottleValue > 90 && adjustedThrottleValue < 200) {
      moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue - 90);
    } else if (adjustedThrottleValue > 200) {
      moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue);
    } else if (adjustedThrottleValue < -90 && adjustedThrottleValue > -200) {
      moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue + 90);
    } else if (adjustedThrottleValue < -200) {
      moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue);
    } else if (adjustedThrottleValue > -90 && adjustedThrottleValue < 90) {
      moveMotor(rearMotor0, rearMotor1, 0);
    }
  }
}
void processBoom(int axisRYValue) {
  int adjustedThrottleValue = axisRYValue / 2;
  if (loaderMode) {
    if (adjustedThrottleValue > 80 || adjustedThrottleValue < -80) {
      moveMotor(auxAttach2, auxAttach3, adjustedThrottleValue);
    } else if (adjustedThrottleValue > -80 && adjustedThrottleValue < 80) {
      moveMotor(auxAttach2, auxAttach3, 0);
    }
  }
}
void processThrottleDipper(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  if (!loaderMode) {
    int smokeThrottle = adjustedThrottleValue / 3;
    moveMotor(rearMotor2, rearMotor3, adjustedThrottleValue);
    moveMotor(frontMotor0, frontMotor1, adjustedThrottleValue);
    moveMotor(auxAttach2, auxAttach3, smokeThrottle);
  } else if (adjustedThrottleValue > 80 || adjustedThrottleValue < -80) {
    moveMotor(auxAttach0, auxAttach1, adjustedThrottleValue);
  } else if (adjustedThrottleValue > -80 && adjustedThrottleValue < 80) {
    moveMotor(auxAttach0, auxAttach1, 0);
  }
}

void processTrimAndHitch(int dpadValue) {
  if (dpadValue == 4 && steeringTrim < 20) {
    steeringTrim = steeringTrim + 1;
    delay(50);
  } else if (dpadValue == 8 && steeringTrim > -20) {
    steeringTrim = steeringTrim - 1;
    delay(50);
  }
  if (dpadValue == 1 || dpadValue == 2) {
    clawServo.detach();
    delay(50);
    clawServo.attach(hitchServoPin);
    if (dpadValue == 1) {
      clawServo.write(hitchServoValue);
      delay(40);
    } else if (dpadValue == 2) {
      clawServo.write(85);
      delay(40);
    }
    clawServo.detach();
    delay(20);
    clawServo.attach(clawServoPin);
  }
}
void processSteeringGrapple(int axisRXValue) {
  int adjustedThrottleValue = axisRXValue / 2;
  if (!loaderMode) {
    // Serial.println(axisRXValue);
    adjustedSteeringValue = (90 - (axisRXValue / 9)) - steeringTrim;
    frontSteeringServo.write(180 - adjustedSteeringValue);

    Serial.print("Steering Value:");
    Serial.println(adjustedSteeringValue);
  } else if (adjustedThrottleValue > 90 || adjustedThrottleValue < -90) {
    moveMotor(auxAttach5, auxAttach4, adjustedThrottleValue);
  } else if (adjustedThrottleValue > -110 && adjustedThrottleValue < 110) {
    moveMotor(auxAttach5, auxAttach4, 0);
  }
}

void processLights(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchButtonTime) > 300) {
    lightMode++;
    if (lightMode == 1) {
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      Serial.println(12);
      delay(10);
      Serial.println(14);
    } else if (lightMode == 2) {
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      delay(100);
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      blinkLT = true;
    } else if (lightMode == 3) {
      blinkLT = false;
      hazardLT = true;
    } else if (lightMode == 4) {
      hazardLT = false;
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      Serial.println(11);
      delay(10);
      Serial.println(13);
      lightMode = 0;
      if (!auxLightsOn) {
        digitalWrite(LT3, HIGH);
        Serial.println(16);
        auxLightsOn = true;
      } else {
        digitalWrite(LT3, LOW);
        Serial.println(15);
        auxLightsOn = false;
      }
    }
    lightSwitchButtonTime = millis();
  }
}
void processTrailerAuxMtr1Forward(bool value) {
  if (value) {
    Serial.println(5);
    delay(10);
    trailerAuxMtr1Forward = true;
  } else if (trailerAuxMtr1Forward) {
    Serial.println(7);
    delay(10);
    trailerAuxMtr1Forward = false;
  }
}
void processTrailerAuxMtr1Reverse(bool value) {
  if (value) {
    Serial.println(6);
    delay(10);
    trailerAuxMtr1Reverse = true;
  } else if (trailerAuxMtr1Reverse) {
    Serial.println(7);
    delay(10);
    trailerAuxMtr1Reverse = false;
  }
}
void processTrailerAuxMtr2Forward(bool value) {
  if (value) {
    Serial.println(8);
    delay(10);
    trailerAuxMtr2Forward = true;
  } else if (trailerAuxMtr2Forward) {
    Serial.println(10);
    delay(10);
    trailerAuxMtr2Forward = false;
  }
}
void processTrailerAuxMtr2Reverse(bool value) {
  if (value) {
    Serial.println(9);
    delay(10);
    trailerAuxMtr2Reverse = true;
  } else if (trailerAuxMtr2Reverse) {
    Serial.println(10);
    delay(10);
    trailerAuxMtr2Reverse = false;
  }
}
void processLoaderMode(bool buttonValue) {
  if (buttonValue && (millis() - loaderModeButtonTime) > 300) {
    if (!loaderMode) {
      loaderMode = true;
      moveMotor(rearMotor2, rearMotor3, 0);
      moveMotor(frontMotor0, frontMotor1, 0);
      moveMotor(auxAttach2, auxAttach3, 0);
    } else {
      loaderMode = false;
    }
    loaderModeButtonTime = millis();
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
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.forgetBluetoothKeys();

  BP32.enableVirtualDevice(false);
  pinMode(rearMotor0, OUTPUT);
  pinMode(rearMotor1, OUTPUT);
  pinMode(rearMotor2, OUTPUT);
  pinMode(rearMotor3, OUTPUT);
  pinMode(frontMotor0, OUTPUT);
  pinMode(frontMotor1, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
  pinMode(auxAttach2, OUTPUT);
  pinMode(auxAttach3, OUTPUT);
  pinMode(auxAttach4, OUTPUT);
  pinMode(auxAttach5, OUTPUT);
  pinMode(LT1, OUTPUT);
  pinMode(LT2, OUTPUT);
  pinMode(LT3, OUTPUT);

  digitalWrite(rearMotor0, LOW);
  digitalWrite(rearMotor1, LOW);
  digitalWrite(rearMotor2, LOW);
  digitalWrite(rearMotor3, LOW);
  digitalWrite(frontMotor0, LOW);
  digitalWrite(frontMotor1, LOW);
  digitalWrite(auxAttach0, LOW);
  digitalWrite(auxAttach1, LOW);
  digitalWrite(auxAttach2, LOW);
  digitalWrite(auxAttach3, LOW);
  digitalWrite(auxAttach4, LOW);
  digitalWrite(auxAttach5, LOW);
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);
  digitalWrite(LT3, LOW);

  frontSteeringServo.attach(frontSteeringServoPin);
  frontSteeringServo.write(adjustedSteeringValue);
  clawServo.attach(clawServoPin);
  clawServo.write(clawServoValue);
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
}
