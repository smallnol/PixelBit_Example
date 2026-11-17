/*
 * UNO R3 + BMP73T104｜Pi-slave 馬達控制（AOA / 跟隨）
 *
 * 硬體：
 *   BMP73T104 為 UNO 相容擴充板，直接插在 UNO 上；
 *   4S 18650 ≈ 14.8V 接擴充板馬達 V+/V-（板內隔離，不灌進 UNO 邏輯電）；
 *   UNO 邏輯電：行充／USB 5V。
 *
 * 接線（AOA 架構）：
 *   樹莓派 → USB-TTL TX → UNO D10（SoftwareSerial RX）
 *   USB-TTL GND → UNO GND
 *   （AOA Anchor USB-Native 接 Pi，與此線分開）
 *
 * 協定（9600，一行一命令，大小寫皆可）：
 *   F  前進
 *   L  弧線左轉（Tag 在左側／角度為負）
 *   R  弧線右轉
 *   Z  自轉搜尋（AOA 無鎖定）
 *   S  軟停
 *   X  緊急煞停（跌倒確認建議送這個）
 *
 * 約 CMD_TIMEOUT_MS 無新命令 → 緊急煞停
 * USB Serial Monitor：115200
 *
 * 函式庫：BMP73T104（Arduino Library Manager）
 */

#include <Wire.h>
#include <BMP73T104.h>
#include <SoftwareSerial.h>
#include <ctype.h>

SoftwareSerial piSerial(10, 11);  // RX=D10 <- Pi/USB-TTL TX；TX=D11 不接

const unsigned long CMD_TIMEOUT_MS = 1200;
const unsigned long CMD_REFRESH_MS = 250;

// BMP73T104：單路約 1.1A、總約 4A；原地對拉易過流鎖死
const int BASE_FORWARD_SPEED = 85;
const int BASE_TURN_SPEED = 60;
const int BASE_SPIN_SPEED = 50;
const int BASE_TURN_INNER = 20;  // 弧線內輪，比正反轉對拉省電流

const unsigned int MOTOR_CMD_GAP_US = 4000;

BMP73T104 myM1(2, DCMotor);  // 左後
BMP73T104 myM2(2, DCMotor);  // 右後
BMP73T104 myM3(2, DCMotor);  // 右前
BMP73T104 myM4(2, DCMotor);  // 左前

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

char activeCmd = CMD_STOP;
unsigned long lastCmdTime = 0;
unsigned long lastApplyTime = 0;

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

void applyDrive(int leftSpeed, int rightSpeed) {
  setLeftMotors(leftSpeed);
  setRightMotors(rightSpeed);
  lastApplyTime = millis();
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
  lastApplyTime = millis();
}

void softStop() {
  applyDrive(0, 0);
}

void applyCommand(char cmd, bool force) {
  if (!force && cmd == activeCmd && (millis() - lastApplyTime < CMD_REFRESH_MS)) {
    return;
  }

  if (force && (activeCmd == CMD_STOP || activeCmd == CMD_BRAKE) &&
      (cmd == CMD_FORWARD || cmd == CMD_LEFT || cmd == CMD_RIGHT || cmd == CMD_SPIN)) {
    clearMotorFaults();
    Serial.println(F("cleared OCP before move"));
  }

  switch (cmd) {
    case CMD_FORWARD:
      applyDrive(BASE_FORWARD_SPEED, BASE_FORWARD_SPEED);
      break;
    case CMD_LEFT:
      applyDrive(BASE_TURN_INNER, BASE_TURN_SPEED);
      break;
    case CMD_RIGHT:
      applyDrive(BASE_TURN_SPEED, BASE_TURN_INNER);
      break;
    case CMD_SPIN:
      applyDrive(-BASE_SPIN_SPEED, BASE_SPIN_SPEED);
      break;
    case CMD_BRAKE:
      emergencyBrake();
      break;
    case CMD_STOP:
    default:
      softStop();
      cmd = CMD_STOP;
      break;
  }

  activeCmd = cmd;
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
      bool force = (cmd != activeCmd);
      applyCommand(cmd, force);

      Serial.print(F("CMD "));
      Serial.write(cmd);
      Serial.println();
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      idx = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  piSerial.begin(9600);
  delay(500);

  Wire.begin();
  Wire.setClock(100000);
  delay(300);

  myM1.begin(M1); delay(100);
  myM2.begin(M2); delay(100);
  myM3.begin(M3); delay(100);
  myM4.begin(M4); delay(100);
  clearMotorFaults();

  emergencyBrake();
  activeCmd = CMD_STOP;
  lastCmdTime = millis();

  Serial.println(F("=== UNO Pi-slave (AOA follow) ready ==="));
  Serial.println(F("CMDs: F/L/R/Z/S/X  (Pi USB-TTL -> D10 @ 9600)"));
  Serial.println(F("Power: 14.8V -> BMP V+/V- (isolated); UNO via USB/powerbank 5V"));
}

void loop() {
  static unsigned long lastFaultClear = 0;

  pollPiCommands();

  if (activeCmd != CMD_STOP && activeCmd != CMD_BRAKE &&
      (millis() - lastCmdTime > CMD_TIMEOUT_MS)) {
    Serial.println(F("Pi cmd timeout -> emergency brake"));
    emergencyBrake();
    activeCmd = CMD_STOP;
  }

  if (activeCmd == CMD_FORWARD || activeCmd == CMD_LEFT ||
      activeCmd == CMD_RIGHT || activeCmd == CMD_SPIN) {
    applyCommand(activeCmd, false);
    if (millis() - lastFaultClear > 1000) {
      clearMotorFaults();
      lastFaultClear = millis();
    }
  }
}
