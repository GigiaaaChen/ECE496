#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>

// =========================
// I2C / MPU6050
// =========================
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

// =========================
// 角度参数
// =========================

// 很小的抖动直接当作 0
const float SMALL_DEADBAND = 2.0f;

// 轻微倾斜仍然保持 STOP
const float STOP_RANGE = 15.0f;

// 角度低通滤波系数
const float ALPHA_ANGLE = 0.15f;

// =========================
// WiFi / TCP
// =========================
const char* ssid     = "RDKX5-Hotspot";
const char* password = "ECE49666";

// NetworkManager hotspot 常见默认：10.42.0.1
const char* host = "10.42.0.1";

// 如果你后面换成 hostapd 方案，再改成这个：
// const char* host = "10.5.5.1";

const int port = 5000;

WiFiClient client;

// =========================
// 全局状态
// =========================
float baseRoll = 0.0f;
float basePitch = 0.0f;

float rollFilt = 0.0f;
float pitchFilt = 0.0f;

String lastCmd = "STOP";

// =========================
// MPU6050: 写 1 字节
// =========================
bool mpuWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  uint8_t err = Wire.endTransmission(true);
  return (err == 0);
}

// =========================
// MPU6050: 连续读多个字节
// =========================
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

// =========================
// 初始化 MPU6050
// =========================
bool mpuInit() {
  if (!mpuWriteByte(REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  if (!mpuWriteByte(REG_SMPLRT_DIV, 0x07)) return false;
  if (!mpuWriteByte(REG_CONFIG, 0x06)) return false;
  if (!mpuWriteByte(REG_GYRO_CONFIG, 0x00)) return false;
  if (!mpuWriteByte(REG_ACCEL_CONFIG, 0x00)) return false;

  return true;
}

// =========================
// 读 roll / pitch
// =========================
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

  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  const float RAD2DEG = 57.2957795f;
  rollDeg  = roll * RAD2DEG;
  pitchDeg = pitch * RAD2DEG;

  return true;
}

// =========================
// 初始水平校准
// =========================
void calibrateLevel() {
  Serial.println("Calibrating level, please keep board still and level...");
  Serial.println();

  const int N = 200;
  float sumRoll = 0.0f;
  float sumPitch = 0.0f;
  int validCount = 0;

  for (int i = 0; i < N; i++) {
    float r, p;
    if (readRollPitch(r, p)) {
      sumRoll += r;
      sumPitch += p;
      validCount++;
    }
    delay(10);
  }

  if (validCount > 0) {
    baseRoll = sumRoll / (float)validCount;
    basePitch = sumPitch / (float)validCount;
  } else {
    baseRoll = 0.0f;
    basePitch = 0.0f;
  }

  rollFilt = 0.0f;
  pitchFilt = 0.0f;

  Serial.println("Calibration done.");
  Serial.println("Base roll:");
  Serial.println(baseRoll);
  Serial.println("Base pitch:");
  Serial.println(basePitch);
  Serial.println();
}

// =========================
// 根据相对角度判断命令
// =========================
String decideCommandFromAngles(float rollRel, float pitchRel) {
  if (fabs(rollRel) < SMALL_DEADBAND) {
    rollRel = 0.0f;
  }

  if (fabs(pitchRel) < SMALL_DEADBAND) {
    pitchRel = 0.0f;
  }

  float ar = fabs(rollRel);
  float ap = fabs(pitchRel);

  // 轻微倾斜仍然认为是 STOP
  if (ar < STOP_RANGE && ap < STOP_RANGE) {
    return "STOP";
  }

  if (ap >= ar) {
    // pitch 主导
    if (pitchRel >= STOP_RANGE) {
      return "LEFT";
    } else if (pitchRel <= -STOP_RANGE) {
      return "RIGHT";
    } else {
      return "STOP";
    }
  } else {
    // roll 主导
    if (rollRel >= STOP_RANGE) {
      return "FORWARD";
    } else if (rollRel <= -STOP_RANGE) {
      return "BACKWARD";
    } else {
      return "STOP";
    }
  }
}

// =========================
// 连接 WiFi
// =========================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.println("Connecting to WiFi...");
  Serial.println("SSID:");
  Serial.println(ssid);
  Serial.println();

  WiFi.begin(ssid, password);

  unsigned long t0 = millis();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Waiting for WiFi...");
    Serial.println();
    delay(500);

    if (millis() - t0 > 20000) {
      Serial.println("WiFi connect timeout, retrying...");
      Serial.println();
      WiFi.disconnect(true);
      delay(1000);
      WiFi.begin(ssid, password);
      t0 = millis();
    }
  }

  Serial.println("WiFi connected.");
  Serial.println("ESP32 IP:");
  Serial.println(WiFi.localIP());
  Serial.println("RSSI:");
  Serial.println(WiFi.RSSI());
  Serial.println();
}

// =========================
// 保证 TCP 已连接
// 连上后发一次 MANUAL
// =========================
bool ensureTcpConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi dropped. Reconnecting...");
    Serial.println();
    connectWiFi();
  }

  if (client.connected()) {
    return true;
  }

  Serial.println("Connecting to RDK server...");
  Serial.println("Host:");
  Serial.println(host);
  Serial.println("Port:");
  Serial.println(port);
  Serial.println();

  if (client.connect(host, port)) {
    client.setNoDelay(true);

    Serial.println("TCP connected to RDK.");
    Serial.println();

    // 连上 TCP 后发送一次 MANUAL 给 server
    String initMsg = "MANUAL\n";
    client.print(initMsg);

    // 串口打印
    Serial.println("Sending initial command to RDK:");
    Serial.println("MANUAL");
    Serial.println();

    while (client.available() > 0) {
      String line = client.readStringUntil('\n');
      Serial.println("RDK replied:");
      Serial.println(line);
      Serial.println();
    }

    return true;
  } else {
    Serial.println("TCP connect failed.");
    Serial.println();
    return false;
  }
}

// =========================
// 发送命令到 RDK
// =========================
void sendCommandToRDK(const String &cmd) {
  if (!ensureTcpConnected()) {
    Serial.println("Send skipped: TCP not connected.");
    Serial.println();
    return;
  }

  String msg = cmd + "\n";

  Serial.println("Sending command to RDK:");
  Serial.println(cmd);
  Serial.println();

  client.print(msg);

  Serial.println("Send done.");
  Serial.println();

  while (client.available() > 0) {
    String line = client.readStringUntil('\n');
    Serial.println("RDK replied:");
    Serial.println(line);
    Serial.println();
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
    Serial.println("MPU init failed.");
    Serial.println();
    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU init OK.");
  Serial.println();

  calibrateLevel();
  connectWiFi();

  Serial.println("System ready.");
  Serial.println("pitch dominant -> LEFT / RIGHT");
  Serial.println("roll dominant  -> FORWARD / BACKWARD");
  Serial.println("Small tilt still stays in STOP range.");
  Serial.println("Every loop will print and send current CMD.");
  Serial.println();
}

void loop() {
  float rollDeg, pitchDeg;

  if (!readRollPitch(rollDeg, pitchDeg)) {
    Serial.println("Read roll/pitch failed.");
    Serial.println();
    delay(200);
    return;
  }

  float rollRel  = rollDeg  - baseRoll;
  float pitchRel = pitchDeg - basePitch;

  rollFilt  = rollFilt  + ALPHA_ANGLE * (rollRel  - rollFilt);
  pitchFilt = pitchFilt + ALPHA_ANGLE * (pitchRel - pitchFilt);

  String cmd = decideCommandFromAngles(rollFilt, pitchFilt);
  lastCmd = cmd;

  Serial.println("ROLL relative (deg):");
  Serial.println(rollFilt);
  Serial.println("PITCH relative (deg):");
  Serial.println(pitchFilt);
  Serial.println();

  Serial.println("Current CMD:");
  Serial.println(cmd);
  Serial.println();

  sendCommandToRDK(cmd);

  delay(40);
}
