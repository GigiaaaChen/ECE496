#include <Wire.h>

/* ========= I2C 引脚 ========= */
#define SDA_PIN 22
#define SCL_PIN 23

/* ========= MPU6050 I2C 地址 ========= */
#define MPU_ADDR 0x68

/* ========= MPU6050 寄存器地址 ========= */
#define REG_WHO_AM_I     0x75
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

/* ========= 量程选择（本代码固定用：Accel ±2g, Gyro ±250 dps） =========
   Accel sensitivity: 16384 LSB/g  (±2g)
   Gyro  sensitivity: 131   LSB/(°/s) (±250 dps)
*/
const float ACC_LSB_PER_G = 16384.0f;
const float GYR_LSB_PER_DPS = 131.0f;

/* ========= 小工具：写 8-bit 寄存器 ========= */
bool mpuWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  uint8_t err = Wire.endTransmission(true);
  return (err == 0);
}

/* ========= 小工具：读 8-bit 寄存器 ========= */
bool mpuReadByte(uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  if (Wire.available()) {
    out = Wire.read();
    return true;
  }
  return false;
}

/* ========= 小工具：读一段连续寄存器 ========= */
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

/* ========= I2C 扫描 ========= */
void i2cScanOnce() {
  Serial.println("Scanning I2C bus...");
  Serial.println();

  int devices = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission(true);
    if (err == 0) {
      Serial.println("Found device at:");
      Serial.println(addr, HEX);
      Serial.println();
      devices++;
    }
    delay(5);
  }

  Serial.println("Scan done. Devices count:");
  Serial.println(devices);
  Serial.println();
}

/* ========= MPU6050 初始化 ========= */
bool mpuInit() {
  Serial.println("Initializing MPU6050...");
  Serial.println();

  // 1) 唤醒：PWR_MGMT_1 = 0x00（清 sleep）
  if (!mpuWriteByte(REG_PWR_MGMT_1, 0x00)) {
    Serial.println("❌ Write PWR_MGMT_1 failed");
    Serial.println();
    return false;
  }
  delay(100);

  // 2) 采样率分频：SMPLRT_DIV
  // Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
  // Gyro Output Rate 通常为 8kHz 或 1kHz（受 DLPF 影响）
  // 这里先设置成 0x07，常见、稳定
  if (!mpuWriteByte(REG_SMPLRT_DIV, 0x07)) {
    Serial.println("❌ Write SMPLRT_DIV failed");
    Serial.println();
    return false;
  }

  // 3) DLPF：CONFIG（低通滤波）
  // 0x06 是常用选项之一（更平滑一些）
  if (!mpuWriteByte(REG_CONFIG, 0x06)) {
    Serial.println("❌ Write CONFIG failed");
    Serial.println();
    return false;
  }

  // 4) 陀螺仪量程：±250 dps -> GYRO_CONFIG = 0x00
  if (!mpuWriteByte(REG_GYRO_CONFIG, 0x00)) {
    Serial.println("❌ Write GYRO_CONFIG failed");
    Serial.println();
    return false;
  }

  // 5) 加速度量程：±2g -> ACCEL_CONFIG = 0x00
  if (!mpuWriteByte(REG_ACCEL_CONFIG, 0x00)) {
    Serial.println("❌ Write ACCEL_CONFIG failed");
    Serial.println();
    return false;
  }

  Serial.println("✅ MPU6050 init done");
  Serial.println();
  return true;
}

/* ========= 读取 6 轴原始数据 ========= */
bool readAccelGyroRaw(int16_t &ax, int16_t &ay, int16_t &az,
                      int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];

  // 从 ACCEL_XOUT_H 连续读 14 bytes：
  // AX(2) AY(2) AZ(2) TEMP(2) GX(2) GY(2) GZ(2)
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, buf, 14)) {
    return false;
  }

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);

  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("BOOT OK");
  Serial.println();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);   // 先用 100kHz，最稳
  delay(200);

  /* 1) I2C Scan */
  i2cScanOnce();

  /* 2) WHO_AM_I */
  Serial.println("Reading WHO_AM_I...");
  Serial.println();

  uint8_t whoami = 0;
  if (mpuReadByte(REG_WHO_AM_I, whoami)) {
    Serial.println("WHO_AM_I value:");
    Serial.println(whoami, HEX);
    Serial.println();

    if (whoami == 0x68) {
      Serial.println("✅ WHO_AM_I correct (0x68)");
      Serial.println();
    } else {
      Serial.println("⚠️ WHO_AM_I unexpected");
      Serial.println();
    }
  } else {
    Serial.println("❌ Failed to read WHO_AM_I");
    Serial.println();
  }

  /* 3) Init MPU6050 */
  if (!mpuInit()) {
    Serial.println("Stopping due to init failure");
    Serial.println();
    while (1) {
      delay(1000);
    }
  }

  Serial.println("Starting streaming AX AY AZ GX GY GZ...");
  Serial.println("Units: accel in g, gyro in deg/s");
  Serial.println();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  bool ok = readAccelGyroRaw(ax, ay, az, gx, gy, gz);

  if (!ok) {
    Serial.println("❌ Read accel/gyro failed");
    Serial.println();
    delay(500);
    return;
  }

  // 转换为物理单位
  float ax_g = (float)ax / ACC_LSB_PER_G;
  float ay_g = (float)ay / ACC_LSB_PER_G;
  float az_g = (float)az / ACC_LSB_PER_G;

  float gx_dps = (float)gx / GYR_LSB_PER_DPS;
  float gy_dps = (float)gy / GYR_LSB_PER_DPS;
  float gz_dps = (float)gz / GYR_LSB_PER_DPS;

  // 输出（全部 println）
  Serial.println("RAW AX AY AZ:");
  Serial.println(ax);
  Serial.println(ay);
  Serial.println(az);
  Serial.println();

  Serial.println("RAW GX GY GZ:");
  Serial.println(gx);
  Serial.println(gy);
  Serial.println(gz);
  Serial.println();

  Serial.println("ACCEL (g) AX AY AZ:");
  Serial.println(ax_g, 6);
  Serial.println(ay_g, 6);
  Serial.println(az_g, 6);
  Serial.println();

  Serial.println("GYRO (deg/s) GX GY GZ:");
  Serial.println(gx_dps, 6);
  Serial.println(gy_dps, 6);
  Serial.println(gz_dps, 6);
  Serial.println();

  Serial.println("-----");
  Serial.println();

  delay(200); // 5Hz 输出，便于观察与截图
}
