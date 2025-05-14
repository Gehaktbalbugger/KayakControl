#ifndef TROLLINGMOTOR_H
#define TROLLINGMOTOR_H

#include <Arduino.h>
#include <EEPROM.h>

// Enumeration for motor direction
enum Direction { FORWARD, REVERSE };

// Main TrollingMotor class
class TrollingMotor {
private:
  // Pin assignments
  uint8_t pinFwdBwd;
  uint8_t pinDirCtrl;
  uint8_t pinPWM;
  uint8_t pinEmergencyStop;
  uint8_t pinESCOnOff;

  // Pointers to timer registers
  volatile uint16_t* pwmRegister;
  volatile uint16_t* topRegister;

  // Current and target motor direction and speed
  Direction currentDirection = FORWARD;
  Direction targetDirection = FORWARD;
  uint16_t currentSpeed = 0;
  uint16_t targetSpeed = 0;

  // Internal state machine for motor operation
  enum Status {
    IDLE,
    STOPPING,
    WAIT_BEFORE_SWITCH,
    SWITCH_DIR_1,
    SWITCH_DIR_2,
    SWITCH_DIR_3,
    RAMP_UP,
    RESTART_AFTER_EMERGENCY
  } status = IDLE;

  // Timers and timestamps for delays
  unsigned long lastStepTime = 0;
  unsigned long switchTimestamp = 0;
  unsigned long emergencyStartTime = 0;

  // Configurable parameters
  const uint16_t PWM_STEP = 50;
  const unsigned long STEP_INTERVAL = 50;
  unsigned long dirWaitTime1 = 500;
  unsigned long dirWaitTime2 = 500;
  unsigned long dirWaitTime3 = 500;
  const unsigned long EMERGENCY_DURATION = 30000;

  bool isInEmergency = false;
  bool waitingAfterEmergency = false;
  uint16_t maxPWM = 0;
  bool debug = false;

  const int EEPROM_ADDR = 0;

  struct Config {
    uint16_t signature = 0xABCD;
    uint8_t version = 1;  // EEPROM structure identifier
    unsigned long dirWait1;
    unsigned long dirWait2;
    unsigned long dirWait3;
    Direction lastDirection;
    uint16_t lastSpeed;
  } config;

  void log(const String& message) {
    if (debug) {
      Serial.print("["); Serial.print(millis()); Serial.print("] [DEBUG] "); Serial.println(message);
    }
  }

public:
  TrollingMotor(uint8_t dirCtrl, uint8_t fwdBwd, uint8_t pwm, volatile uint16_t* pwmReg, volatile uint16_t* topReg, uint8_t emergencyPin, uint8_t escPin)
    : pinDirCtrl(dirCtrl), pinFwdBwd(fwdBwd), pinPWM(pwm), pwmRegister(pwmReg), topRegister(topReg), pinEmergencyStop(emergencyPin), pinESCOnOff(escPin) {}

  void begin(uint16_t topValue, bool debugMode = false) {
    debug = debugMode;
    log("TrollingMotor initialization started");
    pinMode(pinDirCtrl, OUTPUT);
    pinMode(pinFwdBwd, OUTPUT);
    pinMode(pinEmergencyStop, INPUT_PULLUP);
    pinMode(pinPWM, OUTPUT);
    pinMode(pinESCOnOff, OUTPUT);
    digitalWrite(pinDirCtrl, HIGH);
    digitalWrite(pinFwdBwd, LOW);
    digitalWrite(pinESCOnOff, LOW);
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    *topRegister = topValue;
    *pwmRegister = 0;
    maxPWM = topValue;
    loadConfig();
    dirWaitTime1 = config.dirWait1;
    dirWaitTime2 = config.dirWait2;
    dirWaitTime3 = config.dirWait3;
    currentDirection = config.lastDirection;
    targetDirection = config.lastDirection;
    currentSpeed = 0;
    targetSpeed = config.lastSpeed;
    log("Loaded config from EEPROM");
  }

  void setDebug(bool enable) {
    debug = enable;
  }

  void setWaitTimes(unsigned long w1, unsigned long w2, unsigned long w3) {
    dirWaitTime1 = w1;
    dirWaitTime2 = w2;
    dirWaitTime3 = w3;
    config.dirWait1 = w1;
    config.dirWait2 = w2;
    config.dirWait3 = w3;
    saveConfig();
  }

  void setSpeed(uint16_t speed, Direction dir) {
    speed = constrain(speed, 0, maxPWM);
    targetSpeed = speed;
    targetDirection = dir;
    config.lastSpeed = speed;
    config.lastDirection = dir;
    saveConfig();
    log("Speed requested: " + String(speed) + ", Direction: " + (dir == FORWARD ? "FORWARD" : "REVERSE"));
    if (isInEmergency || waitingAfterEmergency) return;
    status = (dir != currentDirection) ? STOPPING : RAMP_UP;
  }

  void saveConfig() {
    EEPROM.put(EEPROM_ADDR, config);
    log("Settings saved to EEPROM");
  }

  void loadConfig() {
    EEPROM.get(EEPROM_ADDR, config);
    if (config.signature != 0xABCD || config.version != 2) {
      log("EEPROM signature mismatch — resetting to defaults");
      resetEEPROM();
    }
  }

  void resetEEPROM() {
    config.signature = 0xABCD;
    config.version = 2;
    config.dirWait1 = 250;
    config.dirWait2 = 500;
    config.dirWait3 = 150;
    config.lastDirection = FORWARD;
    config.lastSpeed = 0;
    saveConfig();
    log("EEPROM reset to defaults");
  }

  void update() {
    unsigned long now = millis();
    if (digitalRead(pinEmergencyStop) == LOW) {
      if (!isInEmergency) {
        *pwmRegister = 0;
        currentSpeed = 0;
        targetSpeed = 0;
        status = IDLE;
        digitalWrite(pinDirCtrl, HIGH);
        digitalWrite(pinESCOnOff, HIGH);
        isInEmergency = true;
        emergencyStartTime = now;
        config.lastSpeed = 0;
        saveConfig();
        log("Emergency stop triggered and state saved");
      }
      return;
    } else {
      if (isInEmergency) {
        if (now - emergencyStartTime >= EMERGENCY_DURATION) {
          isInEmergency = false;
          waitingAfterEmergency = true;
          emergencyStartTime = now;
          log("Emergency resolved, waiting before restart");
        } else return;
      }
      if (waitingAfterEmergency) {
        if (now - emergencyStartTime >= EMERGENCY_DURATION) {
          waitingAfterEmergency = false;
          digitalWrite(pinESCOnOff, LOW);
          digitalWrite(pinDirCtrl, LOW);
          status = RESTART_AFTER_EMERGENCY;
          lastStepTime = now;
          currentSpeed = 0;
          *pwmRegister = 0;
          log("Restarting after emergency delay");
        } else return;
      }
    }

    switch (status) {
      case IDLE: break;
      case RESTART_AFTER_EMERGENCY:
        if (now - lastStepTime >= STEP_INTERVAL) {
          lastStepTime = now;
          if (currentSpeed < targetSpeed) {
            currentSpeed += PWM_STEP;
            if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
            *pwmRegister = currentSpeed;
            log("Rebuilding after emergency: " + String(currentSpeed));
          } else {
            status = IDLE;
            log("Rebuild complete after emergency");
          }
        }
        break;
      case STOPPING:
        if (now - lastStepTime >= STEP_INTERVAL) {
          lastStepTime = now;
          if (currentSpeed > 0) {
            currentSpeed = (currentSpeed > PWM_STEP) ? (currentSpeed - PWM_STEP) : 0;
            *pwmRegister = currentSpeed;
            log("Stopping: " + String(currentSpeed));
          } else {
            status = WAIT_BEFORE_SWITCH;
            switchTimestamp = now;
            log("Waiting before direction switch");
          }
        }
        break;
      case WAIT_BEFORE_SWITCH:
        if (now - switchTimestamp >= dirWaitTime1) {
          status = SWITCH_DIR_1;
          digitalWrite(pinDirCtrl, HIGH);
          switchTimestamp = now;
          log("pinDirCtrl set HIGH");
        }
        break;
      case SWITCH_DIR_1:
        if (now - switchTimestamp >= dirWaitTime2) {
          digitalWrite(pinFwdBwd, (targetDirection == FORWARD) ? LOW : HIGH);
          switchTimestamp = now;
          status = SWITCH_DIR_2;
          log("pinFwdBwd toggled");
        }
        break;
      case SWITCH_DIR_2:
        if (now - switchTimestamp >= dirWaitTime3) {
          status = SWITCH_DIR_3;
          digitalWrite(pinDirCtrl, LOW);
          switchTimestamp = now;
          log("pinDirCtrl set LOW");
        }
        break;
      case SWITCH_DIR_3:
        if (now - switchTimestamp >= dirWaitTime3) {
          currentDirection = targetDirection;
          lastStepTime = now;
          status = RAMP_UP;
          log("Direction switch complete, starting ramp up");
        }
        break;
      case RAMP_UP:
        if (now - lastStepTime >= STEP_INTERVAL) {
          lastStepTime = now;
          if (currentSpeed != targetSpeed) {
            if (currentSpeed < targetSpeed) {
              currentSpeed += PWM_STEP;
              if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
            } else {
              currentSpeed -= PWM_STEP;
              if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
            }
            *pwmRegister = currentSpeed;
            log("Ramping: " + String(currentSpeed));
          } else {
            status = IDLE;
            log("Target speed reached");
          }
        }
        break;
    }
  }

  void enableESC() { 
    unsigned long now = millis();
    digitalWrite(pinDirCtrl, HIGH);
    digitalWrite(pinESCOnOff, LOW);
    *pwmRegister = 0;
    while (millis() - now < dirWaitTime2) {}
    digitalWrite(pinDirCtrl, LOW);
    }

  void disableESC() { 
    targetSpeed = 0;
    status = STOPPING;
    log("Disabling ESC: ramping down speed");
    while (currentSpeed > 0) {
      update();
    }
  //  unsigned long now = millis();
  //  currentSpeed = *pwmRegister;
  //  while (currentSpeed >= 0) {
  //   if (now - lastStepTime >= STEP_INTERVAL) {
  //        lastStepTime = now;
  //       if (currentSpeed > 0) {
 //           currentSpeed = (currentSpeed > PWM_STEP) ? (currentSpeed - PWM_STEP) : 0;
 //           *pwmRegister = currentSpeed;
 //           log("ESC Stopping: " + String(currentSpeed));
 //         }
 //       }
//     }
    digitalWrite(pinDirCtrl, HIGH);
    digitalWrite(pinESCOnOff, HIGH);
    log("ESC disabled after full stop");
    status = IDLE;
    }
  
  bool inEmergency() { return isInEmergency; }
  
  uint8_t getVersion() {
    return config.version;
  }

  void cancel() {
    targetSpeed = 0;
    *pwmRegister = 0;
    currentSpeed = 0;
    status = IDLE;
    log("Cancelled by cancel()");
  }
};

#endif

