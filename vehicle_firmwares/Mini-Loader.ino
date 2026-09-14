/*
 * Mini-Loader.ino  —  ESP-NOW controlled version
 * ═══════════════════════════════════════════════
 * Converted from Bluepad32 (direct Bluetooth gamepad) to ESP-NOW,
 * matching the base-station protocol defined in tx_firmware.ino.
 *
 * ESP-NOW payload received from the ESP32 TX (base station):
 *   struct ControlPayload {
 *       uint8_t ch[6];       // ch0=X ch1=Y ch2=RX ch3=RY ch4=HAT0X ch5=HAT0Y
 *                             // each 0-254, centre = 127
 *       uint8_t buttons[2];  // buttons[0] bit0=A bit1=B bit2=X bit3=Y
 *                             //            bit4=L1 bit5=R1 bit6=L2 bit7=R2
 *                             // buttons[1] bit0=ThumbL bit1=ThumbR
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

// Defined up here (rather than near its point of use, like the original
// sketch had it) so it's visible to Arduino's auto-generated function
// prototypes, which get inserted right after the #include block — if
// ComboButton were declared later, the prototype for handleCombo() would
// reference an undeclared type and fail to compile.
struct ComboButton {
  int count;
  unsigned long firstPressTime;
  bool lastState;
};

ComboButton bBtn = { 0, 0, false };
ComboButton xBtn = { 0, 0, false };

// Explicit prototype: prevents Arduino's auto-generated prototype (which
// gets inserted above this point, before ComboButton is visible) from
// being created and failing to compile.
void handleCombo(bool currentState, ComboButton &btn, bool dirA, bool dirB);

// ── ESP-NOW receive state ──────────────────────────────────────────────────
static volatile ControlPayload latestPayload = {};
static volatile bool           hasNewPacket   = false;
static volatile unsigned long  lastPacketTime = 0;

void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  if (len != sizeof(ControlPayload)) return;  // ignore malformed/foreign packets
  memcpy((void*)&latestPayload, incomingData, sizeof(ControlPayload));
  lastPacketTime = millis();
  hasNewPacket   = true;
}

#define LT1 15
#define LT2 27
#define LT3 14

#define steeringServoPin 23
#define attachmentServoPin 22

Servo steeringServo;
Servo attachmentServo;

#define leftMotor0 4    // \ Used for controlling front drive motor movement
#define leftMotor1 2    // /
#define rightMotor0 12  // \ Used for controlling rear drive motor movement.
#define rightMotor1 13  // /

#define leftBladeTilt0 17
#define leftBladeTilt1 16
#define rightBladeTilt0 18
#define rightBladeTilt1 19
#define attachmentMotor0 26
#define attachmentMotor1 25
#define ripperMotor0 32
#define ripperMotor1 33

unsigned long lastInputTime = 0;
const unsigned long INPUT_TIMEOUT = 500;  // ms — ESP-NOW failsafe (fleet standard)

int buttonSwitchTime = 0;
int lightSwitchTime = 0;
int lightSwitchButtonTime = 0;
int lightMode = 0;
bool lightsOn = false;
bool auxLightsOn = false;
bool blinkLT = false;
bool hazardLT = false;
bool hazardsOn = false;
bool attachmentOn = false;
int adjustedSteeringValue = 90;
int steeringTrim = 0;
unsigned long lastSteeringServoTime = 0;
bool incrementalSteeringMode = false;


unsigned long servoTimer = 0;
bool servoActive = false;
// Triple-tap tracking
int tapCount = 0;
unsigned long firstTapTime = 0;
const unsigned long tapWindow = 800;  // time window to count taps (ms)
int lastDpadValue = 0;

const unsigned long comboWindow = 600;  // ms

// Derive a Bluepad32-style dpad value (1=up, 2=down, 4=right, 8=left, 0=none)
// from the HAT0X/HAT0Y channel bytes so the existing triple-tap logic below
// (which only checks ==1 and ==2) works unchanged.
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

void processGamepad(const ControlPayload& payload) {

  // Record that we received fresh input
  lastInputTime = millis();

  uint8_t b0 = payload.buttons[0];
  uint8_t b1 = payload.buttons[1];

  //Throttle
  processThrottle(byteToAxis(payload.ch[1]));  // ABS_Y

  //Steering
  processSteering(byteToAxis(payload.ch[2]));  // ABS_RX

  processSteeringMode(btnBit(b1, BTN1_THUMBL));
  //Lights
  processLights(btnBit(b1, BTN1_THUMBR));

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
  if (!attachmentOn) {
    if (a) {
      digitalWrite(attachmentMotor0, HIGH);
      digitalWrite(attachmentMotor1, LOW);
    } else if (y) {
      digitalWrite(attachmentMotor0, LOW);
      digitalWrite(attachmentMotor1, HIGH);
    } else {
      digitalWrite(attachmentMotor0, LOW);
      digitalWrite(attachmentMotor1, LOW);
    }
  }
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

  handleCombo(b, bBtn, true, false);  // B: motor0 HIGH
  handleCombo(x, xBtn, false, true);  // X: motor1 HIGH

  int dpadValue = dpadFromChannels(payload);  // get current D-pad value
  int targetPosition = -1;

  // Detect edge (new press)
  if (dpadValue != lastDpadValue && dpadValue != 0) {
    // Check if tap window expired
    if (millis() - firstTapTime > tapWindow) {
      tapCount = 0;  // reset taps
      firstTapTime = millis();
    }

    tapCount++;                                  // increment tap count
    if (tapCount == 1) firstTapTime = millis();  // start timer on first tap

    // Triple-tap detected
    if (tapCount >= 3) {
      if (dpadValue == 2) targetPosition = 115;
      else if (dpadValue == 1) targetPosition = 10;

      if (targetPosition != -1) {
        attachmentServo.attach(attachmentServoPin);
        attachmentServo.write(targetPosition);
        servoTimer = millis();
        servoActive = true;
      }

      // Reset tap count after activation
      tapCount = 0;
      firstTapTime = 0;
    }
  }

  // Turn off servo after 2 seconds
  if (servoActive && millis() - servoTimer >= 2000) {
    attachmentServo.detach();
    servoActive = false;
  }

  // Save last D-pad value
  lastDpadValue = dpadValue;
}

void handleCombo(bool currentState, ComboButton &btn, bool dirA, bool dirB) {
  if (currentState && !btn.lastState) {
    unsigned long now = millis();

    // If attachment is ON → single press turns it OFF
    if (attachmentOn) {
      digitalWrite(attachmentMotor0, LOW);
      digitalWrite(attachmentMotor1, LOW);
      attachmentOn = false;

      btn.count = 0;
      btn.lastState = currentState;
      return;
    }

    // Attachment is OFF → count combo presses
    if (btn.count == 0) {
      btn.firstPressTime = now;
    }

    btn.count++;

    if ((now - btn.firstPressTime) > comboWindow) {
      btn.count = 1;
      btn.firstPressTime = now;
    }

    // 3 presses → turn ON
    if (btn.count == 3) {
      digitalWrite(attachmentMotor0, dirA ? HIGH : LOW);
      digitalWrite(attachmentMotor1, dirB ? HIGH : LOW);
      attachmentOn = true;
      btn.count = 0;
    }
  }

  btn.lastState = currentState;
}
void processThrottle(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
  moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
}
void processSteering(int axisRXValue) {
  if (incrementalSteeringMode) {
    if (millis() - lastSteeringServoTime >= 20) {
      adjustedSteeringValue = adjustedSteeringValue + axisRXValue / 160;
      if (adjustedSteeringValue > 145) {
        adjustedSteeringValue = 144;
      }
      if (adjustedSteeringValue < 45) {
        adjustedSteeringValue = 46;
      }
      steeringServo.write(adjustedSteeringValue);
      lastSteeringServoTime = millis();
    }
  } else {
    // Serial.println(axisRXValue);
    adjustedSteeringValue = 180 - ((90 - (axisRXValue / 12)) - steeringTrim);
    steeringServo.write(adjustedSteeringValue);

    Serial.print("Steering Value:");
    Serial.println(adjustedSteeringValue);
  }
}
void processSteeringMode(bool buttonValue) {
  if (buttonValue && (millis() - buttonSwitchTime) > 300) {
    if (!incrementalSteeringMode) {
      incrementalSteeringMode = true;
    } else {
      incrementalSteeringMode = false;
    }
    buttonSwitchTime = millis();
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
  Serial.println("[RX] Mini-Loader ready. Register this MAC in vehicles.json.");

  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(ripperMotor0, OUTPUT);
  pinMode(ripperMotor1, OUTPUT);
  pinMode(leftBladeTilt0, OUTPUT);
  pinMode(leftBladeTilt1, OUTPUT);
  pinMode(attachmentMotor0, OUTPUT);
  pinMode(attachmentMotor1, OUTPUT);
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
  digitalWrite(attachmentMotor0, LOW);
  digitalWrite(attachmentMotor1, LOW);
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);
  digitalWrite(LT3, LOW);

  steeringServo.attach(steeringServoPin);
  steeringServo.write(adjustedSteeringValue);

  attachmentServo.attach(attachmentServoPin);
  attachmentServo.write(10);


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
    if (!attachmentOn) {
      digitalWrite(attachmentMotor0, LOW);
      digitalWrite(attachmentMotor1, LOW);
    }
  }
}
