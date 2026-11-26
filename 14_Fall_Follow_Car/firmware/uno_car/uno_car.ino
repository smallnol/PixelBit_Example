/*
 * UNO R3 + BMP73T104 + 1×VL53L0X（正前）｜Pi-slave 馬達 + 前方避障
 *
 * ※ 2026-11-25：改為僅 1 顆正前 VL53（不再用左前／右前／正後）。
 *
 * 接線：
 *   ESP32 GPIO17 (TX2) → UNO D10（SoftwareSerial RX，收 F/L/R/Z/S/X）
 *   UNO D11 (SoftSerial TX) → ESP32 GPIO16（回傳 O:mm）
 *   GND 共地
 *
 * VL53L0X 正前（單顆）：
 *   VCC→UNO 5V、GND→GND、SDA→A4、SCL→A5
 *   XSHUT：不接（懸空即可；單顆用預設 0x29，不必改址）
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

const unsigned long CMD_TIMEOUT_MS = 1200;
const unsigned long CMD_REFRESH_MS = 250;
const unsigned long TOF_TELEM_MS = 200;
const unsigned long SOFT_RAMP_MS = 30;

const uint16_t AVOID_MM = 250;
const uint16_t TOF_FAIL_MM = 8190;
// 距離校正：實際 ≈ 讀值 + TOF_OFFSET_MM
// 例：尺量 150mm、序列 171~174 → 偏高約 22mm → 設 -22
// 改完請重燒 UNO；再尺量一次微調
const int16_t TOF_OFFSET_MM = -22;

// 前進維持；轉向／自轉提高扭力（舊差速 60/20 在地毯常轉不動）
const int BASE_FORWARD_SPEED = 85;
const int BASE_TURN_SPEED = 85;   // L/R 近原地轉向單側轉速
const int BASE_SPIN_SPEED = 80;   // Z 搜尋自轉
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
unsigned long lastTelemTime = 0;
unsigned long lastRampTime = 0;

int curLeft = 0;
int curRight = 0;
int tgtLeft = 0;
int tgtRight = 0;
bool emergencyHold = false;

VL53L0X tof;
uint16_t tofMm = TOF_FAIL_MM;
bool tofOk = false;

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
    // L/R：左右輪反向 → 近原地轉向（差速緩轉在負載下幾乎轉不動）
    case CMD_LEFT:
      L = -BASE_TURN_SPEED;
      R = BASE_TURN_SPEED;
      break;
    case CMD_RIGHT:
      L = BASE_TURN_SPEED;
      R = -BASE_TURN_SPEED;
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

  // 轉向／自轉加快爬升，避免短按或找人時還在爬坡就停
  int step = 18;
  if (activeCmd == CMD_LEFT || activeCmd == CMD_RIGHT || activeCmd == CMD_SPIN) {
    step = 28;
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

uint16_t readTofMm() {
  if (!tofOk) return TOF_FAIL_MM;
  uint16_t mm = tof.readRangeSingleMillimeters();
  if (tof.timeoutOccurred() || mm == 0) return TOF_FAIL_MM;
  if (mm >= TOF_FAIL_MM) return TOF_FAIL_MM;
  int32_t corrected = (int32_t)mm + (int32_t)TOF_OFFSET_MM;
  if (corrected < 1) corrected = 1;
  if (corrected >= (int32_t)TOF_FAIL_MM) return TOF_FAIL_MM;
  return (uint16_t)corrected;
}

void pollTof() {
  tofMm = readTofMm();
}

void sendTofTelemetry() {
  // O:正前mm（單顆）
  piSerial.print(F("O:"));
  piSerial.println(tofMm);
  Serial.print(F("O:"));
  Serial.println(tofMm);
}

char applyObstacleOverride(char cmd) {
  if (cmd == CMD_BRAKE) return CMD_BRAKE;
  // 僅前進時用正前距離避障；L/R/Z 不覆寫（單顆無側／後）
  if (cmd != CMD_FORWARD) return cmd;
  if (!tofOk) return cmd;
  if (tofMm < AVOID_MM) {
    Serial.println(F("AVOID: front blocked -> S"));
    return CMD_STOP;
  }
  return CMD_FORWARD;
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

      static char lastLogReq = 0;
      static char lastLogExec = 0;
      if (cmd != lastLogReq || execCmd != lastLogExec || cmd == CMD_BRAKE) {
        Serial.print(F("CMD req="));
        Serial.write(cmd);
        Serial.print(F(" exec="));
        Serial.write(execCmd);
        Serial.println();
        lastLogReq = cmd;
        lastLogExec = execCmd;
      }
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      idx = 0;
    }
  }
}

bool initFrontTof() {
  // 單顆：不控 XSHUT，直接用預設位址 0x29
  delay(50);
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println(F("VL53 front init FAIL"));
    tofOk = false;
    return false;
  }

  uint16_t probe = 65535;
  for (uint8_t t = 0; t < 3; t++) {
    probe = tof.readRangeSingleMillimeters();
    if (!tof.timeoutOccurred() && probe > 0 && probe < 8000) break;
    delay(30);
  }
  Serial.print(F("VL53 front OK addr=0x29 probe="));
  Serial.print(probe);
  Serial.println(F(" mm"));
  tofOk = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(400);

  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  Serial.println(F("=== UNO Pi-slave + 1xVL53 正前 ==="));
  if (!initFrontTof()) {
    Serial.println(F("警告：正前 VL53 失敗，避障停用（馬達仍可用）"));
  }

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

  Serial.print(F("命令 F/L/R/Z/S/X｜O:正前mm｜AVOID "));
  Serial.print(AVOID_MM);
  Serial.print(F("｜offset "));
  Serial.print(TOF_OFFSET_MM);
  Serial.println(F("mm｜僅 F 避障"));
}

void loop() {
  static unsigned long lastFaultClear = 0;

  pollPiCommands();
  pollTof();

  if (!emergencyHold && requestedCmd == CMD_FORWARD) {
    char execCmd = applyObstacleOverride(requestedCmd);
    if (execCmd != activeCmd) {
      setActiveCommand(execCmd, true);
    }
  }

  if (millis() - lastTelemTime >= TOF_TELEM_MS) {
    lastTelemTime = millis();
    sendTofTelemetry();
  }

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
