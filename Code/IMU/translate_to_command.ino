#include <Wire.h>
#include <math.h>

#define SDA_PIN 22
#define SCL_PIN 23

#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// 加速度计灵敏度：±2g 对应 16384 LSB/g
const float ACC_LSB_PER_G = 16384.0f;

// 进入动作的角度阈值（对称使用 ±ANGLE_ENTER）
const float ANGLE_ENTER = 10.0f;   // 超过 10° 认为有动作
// 角度低通滤波系数
const float ALPHA_ANGLE = 0.15f;
// 小死区：小于这个角度直接当 0，减少抖动
const float SMALL_DEADBAND = 2.0f;

float baseRoll = 0.0f;
float basePitch = 0.0f;

float rollFilt = 0.0f;
float pitchFilt = 0.0f;

// 仅用于内部逻辑的记忆（当前方向）
String lastCmd = "STOP";

// ========== I2C 写一个字节 ==========
bool mpuWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  uint8_t err = Wire.endTransmission(true);
  return (err == 0);
}

// ========== I2C 连续读多个字节 ==========
bool mpuReadBytes(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len);
  uint8_t i = 0;
  while (Wire.available() && i < len) {
    buf[i] = Wire.read();
    i++;
  }
  return (i == len);
}

// ========== 初始化 MPU6050 ==========
bool mpuInit() {
  if (!mpuWriteByte(REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  if (!mpuWriteByte(REG_SMPLRT_DIV, 0x07)) return false;
  if (!mpuWriteByte(REG_CONFIG, 0x06)) return false;

  if (!mpuWriteByte(REG_GYRO_CONFIG, 0x00)) return false;   // Gyro ±250 dps
  if (!mpuWriteByte(REG_ACCEL_CONFIG, 0x00)) return false;  // Accel ±2g

  return true;
}

// ========== 由加速度解算 roll / pitch（绝对角度，单位：度）==========
bool readRollPitch(float &rollDeg, float &pitchDeg) {
  uint8_t buf[6];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, buf, 6)) {
    return false;
  }

  int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

  float ax = ((float)ax_raw) / ACC_LSB_PER_G;
  float ay = ((float)ay_raw) / ACC_LSB_PER_G;
  float az = ((float)az_raw) / ACC_LSB_PER_G;

  // roll：绕 X 轴左右倾斜
  // pitch：绕 Y 轴前后倾斜
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  const float RAD2DEG = 57.2957795f;
  rollDeg  = roll * RAD2DEG;
  pitchDeg = pitch * RAD2DEG;

  return true;
}

// ========== 初始水平校准 ==========
void calibrateLevel() {
  Serial.println("Calibrating level, please keep board still and level...");
  Serial.println();

  const int N = 200;
  float sumRoll = 0.0f;
  float sumPitch = 0.0f;

  for (int i = 0; i < N; i++) {
    float r, p;
    if (readRollPitch(r, p)) {
      sumRoll += r;
      sumPitch += p;
    }
    delay(10);
  }

  baseRoll = sumRoll / (float)N;
  basePitch = sumPitch / (float)N;

  rollFilt = 0.0f;
  pitchFilt = 0.0f;

  Serial.println("Calibration done. Base roll / pitch:");
  Serial.println(baseRoll);
  Serial.println(basePitch);
  Serial.println();
}

// ========== 根据“相对水平角度”输出方向（已做 F↔R, B↔L 交换）==========
String decideCommandFromAngles(float rollRel, float pitchRel) {
  // 小死区：太小的值直接当 0，减少抖动
  if (fabs(rollRel) < SMALL_DEADBAND) {
    rollRel = 0.0f;
  }
  if (fabs(pitchRel) < SMALL_DEADBAND) {
    pitchRel = 0.0f;
  }

  float ar = fabs(rollRel);
  float ap = fabs(pitchRel);

  // 两个方向都很小 -> STOP
  if (ar < ANGLE_ENTER && ap < ANGLE_ENTER) {
    return "STOP";
  }

  // 选绝对值更大的轴作为主方向
  if (ap >= ar) {
    // ✅ pitch 主导：前后倾
    // 之前是：pitch>0 -> RIGHT, pitch<0 -> LEFT
    // 现在反过来：pitch>0 -> LEFT, pitch<0 -> RIGHT
    if (pitchRel >= ANGLE_ENTER) {
      return "LEFT";
    } else if (pitchRel <= -ANGLE_ENTER) {
      return "RIGHT";
    } else {
      return "STOP";
    }
  } else {
    // ✅ roll 主导：左右倾
    // 保持不变：roll>0 -> FORWARD, roll<0 -> BACKWARD
    if (rollRel >= ANGLE_ENTER) {
      return "FORWARD";
    } else if (rollRel <= -ANGLE_ENTER) {
      return "BACKWARD";
    } else {
      return "STOP";
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("BOOT OK");
  Serial.println();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(200);

  if (!mpuInit()) {
    Serial.println("MPU init failed");
    Serial.println();
    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU init OK");
  Serial.println();
  calibrateLevel();

  Serial.println("Tilt relative to level:");
  Serial.println("pitch 主导 -> (RIGHT / LEFT), roll 主导 -> (FORWARD / BACKWARD)");
  Serial.println("CMD is printed every loop.");
  Serial.println();
}

void loop() {
  float rollDeg, pitchDeg;
  if (!readRollPitch(rollDeg, pitchDeg)) {
    Serial.println("Read roll/pitch failed");
    Serial.println();
    delay(200);
    return;
  }

  // 相对水平角度
  float rollRel  = rollDeg  - baseRoll;
  float pitchRel = pitchDeg - basePitch;

  // 角度低通滤波
  rollFilt  = rollFilt  + ALPHA_ANGLE * (rollRel  - rollFilt);
  pitchFilt = pitchFilt + ALPHA_ANGLE * (pitchRel - pitchFilt);

  String cmd = decideCommandFromAngles(rollFilt, pitchFilt);
  lastCmd = cmd;

  // 每次 loop 都输出当前角度和 CMD
  Serial.println("ROLL / PITCH relative (deg):");
  Serial.println(rollFilt);
  Serial.println(pitchFilt);
  Serial.println();

  Serial.println("CMD:");
  Serial.println(cmd);
  Serial.println();

  delay(40);  // ~25Hz
}
