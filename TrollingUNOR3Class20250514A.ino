#include "trollingmotor.h"

#define PWM_TOP_VALUE 1065

// Pin definitions
const uint8_t PIN_DIR_CTRL = 11;
const uint8_t PIN_FWD_BWD = 10;
const uint8_t PIN_PWM = 9;            // OC1A (Timer1)
const uint8_t PIN_EMERGENCY = 7;
const uint8_t PIN_ESC_ONOFF = 12;

TrollingMotor motor(PIN_DIR_CTRL, PIN_FWD_BWD, PIN_PWM, &OCR1A, &ICR1, PIN_EMERGENCY, PIN_ESC_ONOFF);

Direction targetDir = FORWARD;
uint16_t targetSpeed = 0;
bool rampAfterDirChange = false;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_EMERGENCY, OUTPUT);
  digitalWrite(PIN_EMERGENCY, HIGH);

  motor.begin(PWM_TOP_VALUE, true);
  motor.enableESC();

  Serial.println("Serial Motor Control Test Started");
  Serial.println("Type S<number> for speed, D0 or D1 for direction (0=FWD, 1=REV), E to toggle emergency.");
  Serial.println("Type ON to enable ESC, OFF to disable ESC.");}

void loop() {
  
  if (rampAfterDirChange && !motor.inEmergency()) {
    motor.setSpeed(targetSpeed, targetDir);
    rampAfterDirChange = false;
  }
  motor.update();
  handleSerial();
}

void handleSerial() {
  static String input;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input.startsWith("S")) {
      int speed = input.substring(1).toInt();
      speed = constrain(speed, 0, PWM_TOP_VALUE);
      targetSpeed = speed;
      motor.setSpeed(speed, targetDir);
      Serial.print("Set speed: "); Serial.println(speed);
    } else if (input.startsWith("D")) {
      int dir = input.substring(1).toInt();
      targetDir = (dir == 1) ? REVERSE : FORWARD;
      motor.setSpeed(0, targetDir); // Stop first; resume ramp in loop
      rampAfterDirChange = true;
      Serial.print("Set direction: "); Serial.println(targetDir == REVERSE ? "REVERSE" : "FORWARD");
    } else if (input == "E") {
      static bool emergency = false;
      emergency = !emergency;
      digitalWrite(PIN_EMERGENCY, emergency ? LOW : HIGH);
      Serial.print("Emergency: "); Serial.println(emergency ? "ACTIVE" : "CLEARED");
    } else if (input == "ON") {
      motor.enableESC();
      Serial.println("ESC enabled");
    } else if (input == "OFF") {
      motor.disableESC();
      Serial.println("ESC disabled");
    }
      input = "";
    } else {
      input += c;
    }
  }
}

