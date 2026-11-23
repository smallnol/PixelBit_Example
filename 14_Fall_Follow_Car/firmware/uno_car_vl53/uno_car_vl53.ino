/*
 * UNO R3 + BMP73T104 + 4×VL53L0X｜Pi-slave 馬達 + 避障／防墜
 *
 * 接線：
 *   ESP32 GPIO17 (TX2) → UNO D10（SoftwareSerial RX，收 F/L/R/Z/S/X）
 *   UNO D11 (SoftSerial TX) → ESP32 GPIO16（回傳 O:mm,mm,mm,mm）
 *   GND 共地
 *
 * VL53L0X XSHUT（共用 I2C SDA/SCL）：
 *   [0]左前       D2 → addr 0x30
 *   [1]正前       D3 → addr 0x31
 *   [2]右前       D4 → addr 0x32
 *   [3]前端下視   D5 → addr 0x33  （朝地面；距離變大＝懸空／樓梯邊緣 → 強制停）
 *
 * 庫：Wire、BMP73T104、SoftwareSerial、Pololu VL53L0X
 * USB 監看：115200
 */
#include <Wire.h>
#include <BMP73T104.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <ctype.h>

SoftwareSerial piSerial(10, 11); // RX=D10 <- ESP32 TX；TX=D11 -> ESP32 RX

// --- 命令逾時／刷新 ---
const unsigned long CMD_TIMEOUT_MS = 1200;
const unsigned long CMD_REFRESH_MS = 250;
const unsigned long TOF_TELEM_MS = 200;
const unsigned long SOFT_RAMP_MS = 40;

// --- 避障／防墜 ---
const uint16_t AVOID_MM = 250;          // 前方障礙：小於此值要躲／停
const uint16_t CLIFF_MM = 150;          // 下視：大於此值視為無地板（樓梯／落差）
const uint16_t TOF_FAIL_MM = 8190;      // 超量程／失敗

// --- 馬達（弧線轉，降低 OCP）---
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

char requestedCmd = CMD_STOP;   // Pi 下達
char activeCmd = CMD_STOP;      // 實際執行（可被避障覆寫）
unsigned long lastCmdTime = 0;
unsigned long lastApplyTime = 0;
unsigned long lastTelemTime = 0;
unsigned long lastRampTime = 0;

int curLeft = 0;
int curRight = 0;
int tgtLeft = 0;
int tgtRight = 0;
bool emergencyHold = false;

// --- VL53L0X（4 顆）---
const uint8_t NUM_TOF = 4;
const uint8_t XSHUT_PINS[NUM_TOF] = {2, 3, 4, 5};          // D2~D5
const uint8_t TOF_ADDR[NUM_TOF] = {0x30, 0x31, 0x32, 0x33};
VL53L0X tof[NUM_TOF];
uint16_t tofMm[NUM_TOF] = {TOF_FAIL_MM, TOF_FAIL_MM, TOF_FAIL_MM, TOF_FAIL_MM};
bool tofOk[NUM_TOF] = {false, false, false, false};

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
  // Soft-start / soft-stop：避免瞬間反轉過流觸發 BMP73T104 OCP
  unsigned long now = millis();
  if (!force && (now - lastRampTime < SOFT_RAMP_MS)) return;
  lastRampTime = now;

  int step = 18;
  // 轉向／自轉用更小步階
  if (activeCmd == CMD_LEFT || activeCmd == CMD_RIGHT || activeCmd == CMD_SPIN) {
    step = 12;
  }
  // 方向相反時先快速收到 0 再加速，降低對拉電流
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
    // 軟停：目標歸零，由 softApply 緩降
    softApplyTargets(true);
  } else if (force) {
    softApplyTargets(true);
  }
}

bool isValidCmd(char c) {
  return c == CMD_FORWARD || c == CMD_LEFT || c == CMD_RIGHT ||
         c == CMD_SPIN || c == CMD_STOP || c == CMD_BRAKE;
}

uint16_t readTofMm(uint8_t i) {
  if (!tofOk[i]) return TOF_FAIL_MM;
  uint16_t mm = tof[i].readRangeContinuousMillimeters();
  if (tof[i].timeoutOccurred() || mm == 0) return TOF_FAIL_MM;
  if (mm > TOF_FAIL_MM) mm = TOF_FAIL_MM;
  return mm;
}

void pollTof() {
  for (uint8_t i = 0; i < NUM_TOF; i++) {
    tofMm[i] = readTofMm(i);
  }
}

void sendTofTelemetry() {
  // O:左前,正前,右前,下視
  piSerial.print(F("O:"));
  for (uint8_t i = 0; i < NUM_TOF; i++) {
    if (i) piSerial.print(',');
    piSerial.print(tofMm[i]);
  }
  piSerial.print('\n');

  Serial.print(F("O:"));
  for (uint8_t i = 0; i < NUM_TOF; i++) {
    if (i) Serial.print(',');
    Serial.print(tofMm[i]);
  }
  Serial.println();
}

bool cliffDetected() {
  // 前端下視朝地面：正常可見地板（距離較小）；
  // 臨樓梯／落差時距離突然變大或超量程 → 防墜煞
  if (!tofOk[3]) return false;
  uint16_t down = tofMm[3];
  return (down >= CLIFF_MM) || (down >= 8000);
}

char applyObstacleOverride(char cmd) {
  // X 最高優先，不受避障／防墜覆寫
  if (cmd == CMD_BRAKE) return CMD_BRAKE;

  // 防墜：任何移動指令（F/L/R/Z）遇到懸空 → 強制停
  if (cmd != CMD_STOP && cliffDetected()) {
    Serial.println(F("CLIFF: drop-off -> S"));
    return CMD_STOP;
  }

  // 前方避障只在前進時介入
  if (cmd != CMD_FORWARD) return cmd;

  uint16_t fl = tofMm[0];
  uint16_t ff = tofMm[1];
  uint16_t fr = tofMm[2];

  bool frontBlocked = (fl < AVOID_MM) || (ff < AVOID_MM) || (fr < AVOID_MM);
  if (!frontBlocked) return CMD_FORWARD;

  if (fl < AVOID_MM && ff < AVOID_MM && fr < AVOID_MM) {
    Serial.println(F("AVOID: front full block -> S"));
    return CMD_STOP;
  }

  // 用左前／右前開闊度決定微偏
  if (fl > fr + 30) {
    Serial.println(F("AVOID: steer L"));
    return CMD_LEFT;
  }
  if (fr > fl + 30) {
    Serial.println(F("AVOID: steer R"));
    return CMD_RIGHT;
  }
  Serial.println(F("AVOID: ambiguous -> S"));
  return CMD_STOP;
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

      char execCmd = applyObstacleOverride(cmd);
      bool force = (execCmd != activeCmd) || (cmd == CMD_BRAKE);
      setActiveCommand(execCmd, force);

      Serial.print(F("CMD req="));
      Serial.write(cmd);
      Serial.print(F(" exec="));
      Serial.write(execCmd);
      Serial.println();
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      idx = 0;
    }
  }
}

bool initTofSensors() {
  // 全部 XSHUT 拉低休眠
  for (uint8_t i = 0; i < NUM_TOF; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(20);

  bool any = false;
  for (uint8_t i = 0; i < NUM_TOF; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(15);
    tof[i].setTimeout(200);
    if (!tof[i].init()) {
      Serial.print(F("VL53 init FAIL idx="));
      Serial.println(i);
      tofOk[i] = false;
      // 失敗也維持 HIGH，避免其他感測器位址衝突時被誤喚醒到 0x29
      continue;
    }
    tof[i].setAddress(TOF_ADDR[i]);
    tof[i].startContinuous();
    tofOk[i] = true;
    any = true;
    Serial.print(F("VL53 OK idx="));
    Serial.print(i);
    Serial.print(F(" addr=0x"));
    Serial.println(TOF_ADDR[i], HEX);
  }
  return any;
}

void setup() {
  Serial.begin(115200);
  piSerial.begin(9600);
  delay(400);

  Wire.begin();
  Wire.setClock(100000);
  delay(200);

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

  Serial.println(F("=== UNO Pi-slave + 4xVL53 避障/防墜 ==="));
  if (!initTofSensors()) {
    Serial.println(F("警告：VL53 全部失敗，避障停用（馬達命令仍可用）"));
  }
  Serial.println(F("命令 F/L/R/Z/S/X｜O:4｜AVOID 250mm｜CLIFF 150mm"));
}

void loop() {
  static unsigned long lastFaultClear = 0;

  pollPiCommands();
  pollTof();

  // 持續用最新 ToF 覆寫（前進避障／任何移動防墜）
  if (!emergencyHold &&
      (requestedCmd == CMD_FORWARD || requestedCmd == CMD_LEFT ||
       requestedCmd == CMD_RIGHT || requestedCmd == CMD_SPIN)) {
    char execCmd = applyObstacleOverride(requestedCmd);
    if (execCmd != activeCmd) {
      setActiveCommand(execCmd, true);
    }
  }

  if (millis() - lastTelemTime >= TOF_TELEM_MS) {
    lastTelemTime = millis();
    sendTofTelemetry();
  }

  // 逾時無命令 → 急停
  if (activeCmd != CMD_STOP && activeCmd != CMD_BRAKE &&
      (millis() - lastCmdTime > CMD_TIMEOUT_MS)) {
    Serial.println(F("Pi 命令逾時 -> 緊急煞停"));
    emergencyBrake();
    activeCmd = CMD_STOP;
    requestedCmd = CMD_STOP;
  }

  // Soft-start 斜坡 + 定期重送
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
