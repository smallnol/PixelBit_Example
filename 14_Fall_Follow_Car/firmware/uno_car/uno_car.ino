/*
 * UNO R3 + BMP73T104｜Pi-slave 馬達（無 VL53）
 *
 * ※ 2026-11-25：VL53L0X 避障暫拆，本韌體只執行 F/L/R/Z/S/X。
 *   舊版（含 4×VL53）見同目錄註解與 DEPLOY.md「VL53 已停用」說明。
 *
 * 接線：
 *   ESP32 GPIO17 (TX2) → UNO D10（SoftwareSerial RX，收 F/L/R/Z/S/X）
 *   UNO D11 (SoftSerial TX) → ESP32 GPIO16（保留；目前不送 O:）
 *   GND 共地
 *
 * 庫：BMP73T104、SoftwareSerial
 * USB 監看：115200
 */
#include <BMP73T104.h>
#include <SoftwareSerial.h>
#include <ctype.h>

SoftwareSerial piSerial(10, 11); // RX=D10 <- ESP32 TX；TX=D11 -> ESP32 RX

const unsigned long CMD_TIMEOUT_MS = 1200;
const unsigned long CMD_REFRESH_MS = 250;
const unsigned long SOFT_RAMP_MS = 40;

const int BASE_FORWARD_SPEED = 85;
const int BASE_TURN_SPEED = 60;
const int BASE_SPIN_SPEED = 50;
const int BASE_TURN_INNER = 20;
const unsigned int MOTOR_CMD_GAP_US = 4000;

BMP73T104 myM1(2, DCMotor); // 左後
BMP73T104 myM2(2, DCMotor); // 右後
BMP73T104 myM3(2, DCMotor); // 右前
BMP73T104 myM4(2, DCMotor); // 左前

const float TRIM_M1 = 1.08;
const float TRIM_M2 = 1.08;
const float TRIM_M3 = 1.08;
const float TRIM_M4 = 1.08;
const int DIR_M1 = 1;
const int DIR_M2 = 1;
const int DIR_M3 = 1;
const int DIR_M4 = 1;

enum Cmd : char {
  CMD_NONE = 0,
  CMD_FORWARD = 'F',
  CMD_LEFT = 'L',
  CMD_RIGHT = 'R',
  CMD_SPIN = 'Z',
  CMD_STOP = 'S',
  CMD_BRAKE = 'X',
};

char requestedCmd = CMD_STOP;
char activeCmd = CMD_STOP;
unsigned long lastCmdTime = 0;
unsigned long lastApplyTime = 0;
unsigned long lastRampTime = 0;

int curLeft = 0;
int curRight = 0;
int tgtLeft = 0;
int tgtRight = 0;
bool emergencyHold = false;

void setSingleMotorSafe(BMP73T104 &motor, int rawSpeed, float trim, int dir) {
  int targetSpeed = constrain((int)(rawSpeed * trim * dir), -100, 100);
  motor.dcMotorRun(targetSpeed);
  delayMicroseconds(MOTOR_CMD_GAP_US);
}

void setLeftMotors(int speed) {
  setSingleMotorSafe(myM1, speed, TRIM_M1, DIR_M1);
  setSingleMotorSafe(myM4, speed, TRIM_M4, DIR_M4);
}

void setRightMotors(int speed) {
  setSingleMotorSafe(myM2, speed, TRIM_M2, DIR_M2);
  setSingleMotorSafe(myM3, speed, TRIM_M3, DIR_M3);
}

void clearMotorFaults() {
  myM1.clearOverCurrent();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM2.clearOverCurrent();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM3.clearOverCurrent();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM4.clearOverCurrent();
  delayMicroseconds(MOTOR_CMD_GAP_US);
}

void emergencyBrake() {
  myM1.dcMotorBrake();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM2.dcMotorBrake();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM3.dcMotorBrake();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM4.dcMotorBrake();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  curLeft = curRight = tgtLeft = tgtRight = 0;
  lastApplyTime = millis();
  emergencyHold = true;
}

void cmdToTargets(char cmd, int &L, int &R) {
  switch (cmd) {
    case CMD_FORWARD:
      L = BASE_FORWARD_SPEED;
      R = BASE_FORWARD_SPEED;
      break;
    case CMD_LEFT:
      L = BASE_TURN_INNER;
      R = BASE_TURN_SPEED;
      break;
    case CMD_RIGHT:
      L = BASE_TURN_SPEED;
      R = BASE_TURN_INNER;
      break;
    case CMD_SPIN:
      L = -BASE_SPIN_SPEED;
      R = BASE_SPIN_SPEED;
      break;
    case CMD_BRAKE:
    case CMD_STOP:
    default:
      L = 0;
      R = 0;
      break;
  }
}

int stepToward(int cur, int tgt, int step) {
  if (cur < tgt) return min(cur + step, tgt);
  if (cur > tgt) return max(cur - step, tgt);
  return cur;
}

void softApplyTargets(bool force) {
  unsigned long now = millis();
  if (!force && (now - lastRampTime < SOFT_RAMP_MS)) return;
  lastRampTime = now;

  int step = 18;
  if (activeCmd == CMD_LEFT || activeCmd == CMD_RIGHT || activeCmd == CMD_SPIN) {
    step = 12;
  }
  if ((curLeft > 0 && tgtLeft < 0) || (curLeft < 0 && tgtLeft > 0)) {
    tgtLeft = 0;
  }
  if ((curRight > 0 && tgtRight < 0) || (curRight < 0 && tgtRight > 0)) {
    tgtRight = 0;
  }

  int nL = stepToward(curLeft, tgtLeft, step);
  int nR = stepToward(curRight, tgtRight, step);
  if (nL == curLeft && nR == curRight && !force) return;

  setLeftMotors(nL);
  setRightMotors(nR);
  curLeft = nL;
  curRight = nR;
  lastApplyTime = now;
}

void setActiveCommand(char cmd, bool force) {
  if (cmd == CMD_BRAKE) {
    emergencyBrake();
    activeCmd = CMD_BRAKE;
    requestedCmd = CMD_BRAKE;
    return;
  }

  emergencyHold = false;
  if (force && (activeCmd == CMD_STOP || activeCmd == CMD_BRAKE) &&
      (cmd == CMD_FORWARD || cmd == CMD_LEFT || cmd == CMD_RIGHT || cmd == CMD_SPIN)) {
    clearMotorFaults();
    Serial.println(F("cleared OCP before move"));
  }

  activeCmd = cmd;
  cmdToTargets(cmd, tgtLeft, tgtRight);
  if (cmd == CMD_STOP) {
    softApplyTargets(true);
  } else if (force) {
    softApplyTargets(true);
  }
}

bool isValidCmd(char c) {
  return c == CMD_FORWARD || c == CMD_LEFT || c == CMD_RIGHT ||
         c == CMD_SPIN || c == CMD_STOP || c == CMD_BRAKE;
}

void pollPiCommands() {
  static char buf[16];
  static uint8_t idx = 0;

  while (piSerial.available() > 0) {
    char c = piSerial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0';
      idx = 0;

      char cmd = 0;
      for (uint8_t i = 0; buf[i] != '\0'; i++) {
        char u = toupper(buf[i]);
        if (isValidCmd(u)) {
          cmd = u;
          break;
        }
      }
      if (cmd == 0) continue;

      lastCmdTime = millis();
      requestedCmd = cmd;

      bool force = (cmd != activeCmd) || (cmd == CMD_BRAKE);
      setActiveCommand(cmd, force);

      static char lastLogReq = 0;
      if (cmd != lastLogReq || cmd == CMD_BRAKE) {
        Serial.print(F("CMD="));
        Serial.write(cmd);
        Serial.println();
        lastLogReq = cmd;
      }
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      idx = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);

  Serial.println(F("=== UNO Pi-slave 馬達（VL53 已停用）==="));
  piSerial.begin(9600);
  delay(50);

  myM1.begin(M1); delay(80);
  myM2.begin(M2); delay(80);
  myM3.begin(M3); delay(80);
  myM4.begin(M4); delay(80);
  clearMotorFaults();
  emergencyBrake();
  emergencyHold = false;
  activeCmd = CMD_STOP;
  requestedCmd = CMD_STOP;
  lastCmdTime = millis();

  Serial.println(F("命令 F/L/R/Z/S/X｜無 O: 遙測｜無避障覆寫"));
}

void loop() {
  static unsigned long lastFaultClear = 0;

  pollPiCommands();

  if (activeCmd != CMD_STOP && activeCmd != CMD_BRAKE &&
      (millis() - lastCmdTime > CMD_TIMEOUT_MS)) {
    Serial.println(F("Pi 命令逾時 -> 緊急煞停"));
    emergencyBrake();
    activeCmd = CMD_STOP;
    requestedCmd = CMD_STOP;
  }

  if (!emergencyHold) {
    softApplyTargets(false);
  }

  if (activeCmd == CMD_FORWARD || activeCmd == CMD_LEFT ||
      activeCmd == CMD_RIGHT || activeCmd == CMD_SPIN) {
    if (millis() - lastApplyTime >= CMD_REFRESH_MS) {
      softApplyTargets(true);
    }
    if (millis() - lastFaultClear > 1000) {
      clearMotorFaults();
      lastFaultClear = millis();
    }
  }
}
