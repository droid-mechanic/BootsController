#include <Arduino.h>
#include <ESP32Servo.h> // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define rightMotor0 25
#define rightMotor1 26

#define leftMotor0 33
#define leftMotor1 32

#define armMotor0 21
#define armMotor1 19

#define bucketServoPin  23
#define clawServoPin 22

#define auxLights0 18
#define auxLights1 5

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

Servo bucketServo;
Servo clawServo;

int bucketServoValue = 150;
int clawServoValue = 150;
int servoDelay = 0;

bool auxLightsOn = true;

#define rightMotor0 25
#define rightMotor1 26

#define leftMotor0 33
#define leftMotor1 32

#define armMotor0 21
#define armMotor1 19

#define rightTrigger1 32
#define rightTrigger2 128
#define leftTrigger1 16
#define leftTrigger2 64

void processGamepad(ControllerPtr ctl) {
  processMovement(ctl->buttons());
  //Boom Arm Motor move
  processAxisMotor(ctl->axisY(), armMotor0, armMotor1);
  //Bucket ServoMove
  processAxisServo(bucketServo, ctl->axisRY(), bucketServoValue);
  //Claw Servo Move
  //processAxisServo(clawServo, ctl->axisRX(), clawServoValue);
}

void processAxisMotor(int axisValue, int motor0, int motor1) {
  int minimal_control_input_stick = 300;

  int movement = STOP;
  if (abs(axisValue) >= minimal_control_input_stick) {
    if (axisValue > 0) {
      movement++;
    }
    if (axisValue < 0) {
      movement--;
    }
  }
  moveMotor(motor0, motor1, movement);
}

void processAxisServo(Servo &servo, int axisValue, int &servoValue) {
  int minimal_control_input_stick = 300;

  int movement = STOP;
  if (abs(axisValue) >= minimal_control_input_stick) {
    if (axisValue > 0) {
      movement++;
    }
    if (axisValue < 0) {
      movement--;
    }
  }

  Serial.println(movement);
  switch (movement) {
    case 1:
        if(bucketServoValue >= 10 && bucketServoValue < 170)
        {
          Serial.println(bucketServoValue);
          bucketServoValue = bucketServoValue + 5;
          servo.write(bucketServoValue);
        }
    break;
    case -1:
        if(bucketServoValue <= 170 && bucketServoValue > 10)
        {
          Serial.println(bucketServoValue);
          bucketServoValue = bucketServoValue - 5;
          servo.write(bucketServoValue);
        }
    break;
  }
}

void processMovement(int movementBitmask) {
  //Serial.println(movementBitmask);
  //Right Motor
  int rightMovement = STOP;
  if (movementBitmask & rightTrigger2) {
    rightMovement++;
  }
  if (movementBitmask & rightTrigger1) {
    rightMovement--;
  }
  moveMotor(rightMotor0, rightMotor1, rightMovement);

  //Left Motor
  int leftMovement = STOP;
  if (movementBitmask & leftTrigger2) {
    leftMovement++;
  }
  if (movementBitmask & leftTrigger1) {
    leftMovement--;
  }
  moveMotor(leftMotor0, leftMotor1, leftMovement);
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity == FORWARD)
  {
    digitalWrite(motorPin0, HIGH);
    digitalWrite(motorPin1, LOW);
  }
  else if (velocity == BACKWARD)
  {
    digitalWrite(motorPin0, LOW);
    digitalWrite(motorPin1, HIGH);
  }
  else
  {
    digitalWrite(motorPin0, LOW);
    digitalWrite(motorPin1, LOW);
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

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  // You could add additional error handling here,
  // such as logging the error or attempting to recover.
  // For example, you might attempt to reset the MCP23X17
  // and retry initialization before giving up completely.
  // Then, you could gracefully exit the program or continue
  // running with limited functionality.
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(armMotor0, OUTPUT);
  pinMode(armMotor1, OUTPUT);
  
  
  bucketServo.attach(bucketServoPin);
  clawServo.attach(clawServoPin);

  bucketServo.write(bucketServoValue);
  clawServo.write(clawServoValue);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
    {
        processControllers();
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    else {vTaskDelay(1);}
}