#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BleCombo.h>     // ESP32_BLE_Combo_Keyboard_Mouse
#include <esp_sleep.h>
#include <esp_gap_ble_api.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <stdlib.h>

// =====================================================
// BLE 组合设备：先键盘，再把鼠标关联上
// =====================================================
BleComboKeyboard bleKB("HeadsUp-Remote KBM v5", "Caroline-496-remote", 100);
BleComboMouse    bleMouse(&bleKB);

// =====================================================
// 引脚 / I2C 地址
// =====================================================
// 这里让 touchpad 和 MPU6050 共用同一条 I2C 总线
#define SDA_PIN      21
#define SCL_PIN      22

// touchpad
#define I2C_ADDR     0x74
#define RST_PIN      23
#define RDY_PIN      19

// buttons / LED
#define LED_PIN      2
#define BUTTON_BOOT  0    // BOOT（按下=LOW）
#define BUTTON1_PIN  4   // mode toggle（按下=LOW）
#define BUTTON2_PIN  5   // lock/unlock toggle（按下=LOW）

// =====================================================
// IQS5xx 常用寄存器
// =====================================================
#define REG_NUM_FINGERS      0x0011
#define REG_ABS_X_F1         0x0017
#define REG_ABS_Y_F1         0x0019
#define REG_GESTURE_EVENTS0  0x000D   // bit0: SINGLE_TAP
#define REG_END_WINDOW       0xEEEE

// =====================================================
// 触控板鼠标移动参数
// =====================================================
const int MOVE_DIV = 40;      // 原来30，变大后整体更不敏感
const int MOVE_CAP = 9;       // 原来7，限制单次最大移动
const int MOTION_THRESH = 18; // 原来8，小抖动直接忽略
bool INVERT_Y = false;  

// =====================================================
// BOOT 参数
// =====================================================
const unsigned long CLEAR_BOND_LONG_MS = 5000;  // 长按5秒：清除旧 BLE 配对并重启
bool btnPrev = false;
unsigned long btnDownAt = 0;
bool clearBondTriggered = false;

// =====================================================
// button1 / button2 状态
// =====================================================
bool button1Prev = false;
bool button2Prev = false;

// button2: lock / unlock 翻转状态
// false 表示当前未锁，因此下一次按下会发送 "lock"
// true  表示当前已锁，因此下一次按下会发送 "unlock"
bool lockState = false;

// 避免 button2 在 manual mode 下触发后，同一轮 loop 又立刻发送 IMU 命令
bool skipManualCmdThisLoop = false;

// =====================================================
// 上一帧 touchpad 坐标
// =====================================================
uint16_t lastX = 0, lastY = 0;
bool haveLast = false;

// =====================================================
// WiFi / TCP
// =====================================================
const char* ssid     = "RDKX5-Hotspot";
const char* password = "ECE49666";
const char* host     = "10.42.0.1";
const int port       = 5000;

WiFiClient client;

// 是否至少成功建立过一次 manual 用的网络链路
bool manualNetworkReady = false;

// =====================================================
// 模式
// =====================================================
enum ControlMode {
  AUTO_MODE = 0,
  MANUAL_MODE = 1
};

ControlMode currentMode = AUTO_MODE;

// =====================================================
// MPU6050
// =====================================================
#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

const float ACC_LSB_PER_G = 16384.0f;

// 角度参数
const float SMALL_DEADBAND = 2.0f;
const float STOP_RANGE     = 15.0f;
const float ALPHA_ANGLE    = 0.15f;

// IMU 全局状态
float baseRoll = 0.0f;
float basePitch = 0.0f;

float rollFilt = 0.0f;
float pitchFilt = 0.0f;

String lastCmd = "STOP";

// =====================================================
// I2C helpers: touchpad
// =====================================================
static bool iqs_read(uint16_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission((uint8_t)I2C_ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)len);
  if (n != len) return false;

  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool iqs_read_u16(uint16_t reg, uint16_t &out) {
  uint8_t b[2];
  if (!iqs_read(reg, b, 2)) return false;
  out = ((uint16_t)b[0] << 8) | b[1];
  return true;
}

static bool iqs_write1(uint16_t reg, uint8_t val) {
  Wire.beginTransmission((uint8_t)I2C_ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

// =====================================================
// 触控板硬复位
// =====================================================
static void hardResetTP() {
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(20);
  digitalWrite(RST_PIN, HIGH);
  delay(50);
}

// =====================================================
// 清除所有 BLE bonded devices
// =====================================================
static void clearAllBondedDevices() {
  int dev_num = esp_ble_get_bond_device_num();
  Serial.printf("Bonded device count: %d\n", dev_num);

  if (dev_num <= 0) {
    Serial.println("No bonded devices.");
    return;
  }

  esp_ble_bond_dev_t *dev_list =
      (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);

  if (dev_list == NULL) {
    Serial.println("malloc failed.");
    return;
  }

  if (esp_ble_get_bond_device_list(&dev_num, dev_list) == ESP_OK) {
    for (int i = 0; i < dev_num; i++) {
      esp_err_t err = esp_ble_remove_bond_device(dev_list[i].bd_addr);

      Serial.printf("Removing bonded device %d: ", i);
      for (int j = 0; j < 6; j++) {
        Serial.printf("%02X", dev_list[i].bd_addr[j]);
        if (j < 5) Serial.print(":");
      }
      Serial.printf(" -> %s\n", (err == ESP_OK) ? "OK" : "FAIL");
    }
  } else {
    Serial.println("Failed to get bonded device list.");
  }

  free(dev_list);
}

// =====================================================
// MPU6050: 写 1 字节
// =====================================================
bool mpuWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  uint8_t err = Wire.endTransmission(true);
  return (err == 0);
}

// =====================================================
// MPU6050: 连续读多个字节
// =====================================================
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

// =====================================================
// 初始化 MPU6050
// =====================================================
bool mpuInit() {
  if (!mpuWriteByte(REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  if (!mpuWriteByte(REG_SMPLRT_DIV, 0x07)) return false;
  if (!mpuWriteByte(REG_CONFIG, 0x06)) return false;
  if (!mpuWriteByte(REG_GYRO_CONFIG, 0x00)) return false;
  if (!mpuWriteByte(REG_ACCEL_CONFIG, 0x00)) return false;

  return true;
}

// =====================================================
// 读 roll / pitch
// =====================================================
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

// =====================================================
// 初始水平校准
// =====================================================
void calibrateLevel() {
  Serial.println("Calibrating level, keep board still...");
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
  Serial.print("Base roll: ");
  Serial.println(baseRoll);
  Serial.print("Base pitch: ");
  Serial.println(basePitch);
}

// =====================================================
// 根据相对角度判断命令
// =====================================================
String decideCommandFromAngles(float rollRel, float pitchRel) {
  if (fabs(rollRel) < SMALL_DEADBAND) {
    rollRel = 0.0f;
  }

  if (fabs(pitchRel) < SMALL_DEADBAND) {
    pitchRel = 0.0f;
  }

  float ar = fabs(rollRel);
  float ap = fabs(pitchRel);

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

// =====================================================
// 连接 WiFi
// =====================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);

  // 不要在 BLE + WiFi 共存时关闭 modem sleep
  // WiFi.setSleep(false);

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.println("Connecting to WiFi...");
  Serial.print("SSID: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  unsigned long t0 = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Waiting for WiFi...");

    if (millis() - t0 > 20000) {
      Serial.println("WiFi connect timeout, retrying...");
      WiFi.disconnect(true);
      delay(1000);
      WiFi.begin(ssid, password);
      t0 = millis();
    }
  }

  Serial.println("WiFi connected.");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
}

// =====================================================
// 保证 TCP 已连接
// =====================================================
bool ensureTcpConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi dropped. Reconnecting...");
    connectWiFi();
  }

  if (client.connected()) {
    return true;
  }

  Serial.println("Connecting to RDK server...");
  Serial.print("Host: ");
  Serial.println(host);
  Serial.print("Port: ");
  Serial.println(port);

  if (client.connect(host, port)) {
    client.setNoDelay(true);
    Serial.println("TCP connected to RDK.");
    return true;
  } else {
    Serial.println("TCP connect failed.");
    return false;
  }
}

// =====================================================
// 发送一行到 RDK
// 成功返回 true，失败返回 false
// =====================================================
bool sendLineToRDK(const String &line) {
  if (!ensureTcpConnected()) {
    Serial.println("Send skipped: TCP not connected.");
    return false;
  }

  client.print(line);
  client.print("\n");

  Serial.print("Sent to RDK: ");
  Serial.println(line);

  while (client.available() > 0) {
    String resp = client.readStringUntil('\n');
    Serial.print("RDK replied: ");
    Serial.println(resp);
  }

  return true;
}

// =====================================================
// 进入 manual mode
// 首次进入时才真正建立 WiFi/TCP
// 每次进入前都要先发 MANUAL
// =====================================================
void switchToManualMode() {
  if (!manualNetworkReady) {
    connectWiFi();
    if (!ensureTcpConnected()) {
      Serial.println("Cannot enter MANUAL mode because TCP is not ready.");
      return;
    }
    manualNetworkReady = true;
  } else {
    if (!ensureTcpConnected()) {
      Serial.println("TCP reconnect failed, MANUAL mode switch aborted.");
      return;
    }
  }

  if (!sendLineToRDK("MANUAL")) {
    Serial.println("Failed to send MANUAL.");
    return;
  }

  currentMode = MANUAL_MODE;
  lastCmd = "STOP";
  Serial.println("Mode changed to MANUAL_MODE");
}

// =====================================================
// 回到 auto mode
// 切换前先发 AUTO，再切模式
// =====================================================
void switchToAutoMode() {
  if (manualNetworkReady) {
    if (ensureTcpConnected()) {
      if (!sendLineToRDK("AUTO")) {
        Serial.println("Warning: failed to send AUTO before switching back.");
      }
    } else {
      Serial.println("Warning: failed to send AUTO before switching back.");
    }
  }

  currentMode = AUTO_MODE;
  haveLast = false;
  Serial.println("Mode changed to AUTO_MODE");
}

// =====================================================
// 处理“长按5秒清配对”的按钮逻辑
// =====================================================
void handlePowerButton() {
  bool pressed = (digitalRead(BUTTON_BOOT) == LOW);

  if (pressed && !btnPrev) {
    btnDownAt = millis();
    clearBondTriggered = false;
  }

  if (pressed && btnPrev && !clearBondTriggered) {
    unsigned long held = millis() - btnDownAt;
    if (held >= CLEAR_BOND_LONG_MS) {
      clearBondTriggered = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("BOOT 5s hold -> clearing BLE bonds...");
      clearAllBondedDevices();
      Serial.println("BLE bonds cleared. Restarting...");
      delay(300);
      ESP.restart();
    }
  }

  btnPrev = pressed;
}

// =====================================================
// button1：切换模式
// =====================================================
void handleButton1() {
  bool pressed = (digitalRead(BUTTON1_PIN) == LOW);

  if (pressed && !button1Prev) {
    Serial.println("button1 pressed");

    if (currentMode == AUTO_MODE) {
      switchToManualMode();
    } else {
      switchToAutoMode();
    }
  }

  button1Prev = pressed;
}

// =====================================================
// button2：
// 当 TCP 曾经成功连接过至少一次后，
// 每次按下在 "lock" / "unlock" 之间翻转并发送
// =====================================================
void handleButton2() {
  bool pressed = (digitalRead(BUTTON2_PIN) == LOW);

  if (pressed && !button2Prev) {
    Serial.println("button2 pressed");

    if (!manualNetworkReady) {
      Serial.println("button2 ignored: TCP has never been connected yet.");
    } else {
      if (!ensureTcpConnected()) {
        Serial.println("button2 action failed: TCP reconnect failed.");
      } else {
        String lineToSend;

        if (!lockState) {
          lineToSend = "lock";
        } else {
          lineToSend = "unlock";
        }

        if (sendLineToRDK(lineToSend)) {
          lockState = !lockState;
          Serial.print("button2 action -> ");
          Serial.println(lineToSend);

          if (currentMode == MANUAL_MODE) {
            skipManualCmdThisLoop = true;
          }
        } else {
          Serial.println("button2 action failed: send failed.");
        }
      }
    }
  }

  button2Prev = pressed;
}

// =====================================================
// AUTO mode：处理 touchpad -> BLE mouse
// =====================================================
// AUTO mode：处理 touchpad -> BLE mouse
void handleAutoModeTouchpad() {
  bool connected = bleKB.isConnected();

  // 重新落指后的稳定计数
  static uint8_t touchStableCount = 0;
  const uint8_t TOUCH_SETTLE_FRAMES = 4;

  // 滚轮专用参数：故意设得比较钝一点
  const int SCROLL_THRESH = 20;
  const int SCROLL_DIV    = 65;
  const int SCROLL_CAP    = 6;

  if (digitalRead(RDY_PIN) != LOW) {   // 极性不对就改成 == LOW
    bool anyData = false;

    // SINGLE_TAP -> 左键点击
    uint8_t ge0 = 0;
    if (iqs_read(REG_GESTURE_EVENTS0, &ge0, 1)) {
      anyData = true;
      if (ge0 & (1 << 0)) {
        if (connected) bleMouse.click(MOUSE_LEFT);
        Serial.println("[gesture] SINGLE_TAP -> left click");
      }
    }

    // 读手指数
    uint8_t nf = 0;
    if (iqs_read(REG_NUM_FINGERS, &nf, 1)) {
      if (nf > 0) {
        uint16_t x = 0, y = 0;
        bool okx = iqs_read_u16(REG_ABS_X_F1, x);
        bool oky = iqs_read_u16(REG_ABS_Y_F1, y);

        if (okx && oky) {
          anyData = true;

          // 第一次接触 / 重新落指：只记录起点，不产生动作
          if (!haveLast) {
            lastX = x;
            lastY = y;
            haveLast = true;
            touchStableCount = 0;
          } else {
            // 前几帧先稳定，不移动也不滚动
            if (touchStableCount < TOUCH_SETTLE_FRAMES) {
              lastX = x;
              lastY = y;
              touchStableCount++;
            } else {
              int16_t dx = (int16_t)x - (int16_t)lastX;
              int16_t dy = (int16_t)y - (int16_t)lastY;

              // =========================
              // 双指：上下滚轮
              // =========================
              if (nf >= 2) {
                int wheel = 0;
                int hWheel = 0;

                // 哪个轴更明显，就只发那个轴的 scroll
                if (abs(dy) > abs(dx) * 2) {
                  if (abs(dy) >= SCROLL_THRESH) {
                    wheel = dy / SCROLL_DIV;
                    if (wheel == 0) wheel = (dy > 0) ? 1 : -1;
                    if (wheel >  SCROLL_CAP) wheel =  SCROLL_CAP;
                    if (wheel < -SCROLL_CAP) wheel = -SCROLL_CAP;
                  }
                } else if (abs(dx) > abs(dy) * 2) {
                  if (abs(dx) >= SCROLL_THRESH) {
                    hWheel = -dx / SCROLL_DIV;
                    if (hWheel == 0) hWheel = (dx > 0) ? -1 : 1;
                    if (hWheel >  SCROLL_CAP) hWheel =  SCROLL_CAP;
                    if (hWheel < -SCROLL_CAP) hWheel = -SCROLL_CAP;
                  }
                }

                if (connected && (wheel != 0 || hWheel != 0)) {
                  bleMouse.move(0, 0, (int8_t)wheel, (int8_t)hWheel);
                }

                lastX = x;
                lastY = y;
              }

              // =========================
              // 单指：鼠标移动
              // =========================
              else {
                // 轴向偏置：明显横向就压掉纵向抖动，明显纵向就压掉横向抖动
                if (abs(dx) > abs(dy) * 2) {
                  dy = 0;
                } else if (abs(dy) > abs(dx) * 2) {
                  dx = 0;
                }

                // 小抖动忽略
                if (abs(dx) < MOTION_THRESH) dx = 0;
                if (abs(dy) < MOTION_THRESH) dy = 0;

                if (dx != 0 || dy != 0) {
                  // 你之前已经确认过需要左右/上下都反转
                  int mx = -dx / MOVE_DIV;
                  int my = -dy / MOVE_DIV;

                  // 防止除完变成 0
                  if (dx != 0 && mx == 0) mx = (dx > 0) ? -1 : 1;
                  if (dy != 0 && my == 0) my = (dy > 0) ? -1 : 1;

                  if (mx >  MOVE_CAP) mx =  MOVE_CAP;
                  if (mx < -MOVE_CAP) mx = -MOVE_CAP;
                  if (my >  MOVE_CAP) my =  MOVE_CAP;
                  if (my < -MOVE_CAP) my = -MOVE_CAP;

                  if (connected) {
                    bleMouse.move((int8_t)mx, (int8_t)my);
                  }
                }

                // 每帧更新基准点，减少误差累计
                lastX = x;
                lastY = y;
              }
            }
          }
        }
      } else {
        // 手指离开：断开本次轨迹
        haveLast = false;
        touchStableCount = 0;
      }
    }

    digitalWrite(LED_PIN, anyData ? HIGH : LOW);
    iqs_write1(REG_END_WINDOW, 0x00);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

// =====================================================
// MANUAL mode：处理 IMU -> TCP command
// =====================================================
void handleManualModeIMU() {
  float rollDeg, pitchDeg;

  if (!readRollPitch(rollDeg, pitchDeg)) {
    Serial.println("Read roll/pitch failed.");
    delay(50);
    return;
  }

  float rollRel  = rollDeg  - baseRoll;
  float pitchRel = pitchDeg - basePitch;

  rollFilt  = rollFilt  + ALPHA_ANGLE * (rollRel  - rollFilt);
  pitchFilt = pitchFilt + ALPHA_ANGLE * (pitchRel - pitchFilt);

  String cmd = decideCommandFromAngles(rollFilt, pitchFilt);
  lastCmd = cmd;

  Serial.print("ROLL relative (deg): ");
  Serial.println(rollFilt);
  Serial.print("PITCH relative (deg): ");
  Serial.println(pitchFilt);
  Serial.print("Current CMD: ");
  Serial.println(cmd);

  sendLineToRDK(cmd);
}

// =====================================================
// setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(RDY_PIN, INPUT);
  pinMode(BUTTON_BOOT, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  hardResetTP();

  bleKB.begin();

  Serial.println();
  Serial.println("BLE Combo (Keyboard+Mouse) started");
  Serial.println("BOOT hold 5s -> clear BLE bonds and restart");
  Serial.println("BUTTON1 -> toggle AUTO / MANUAL mode");
  Serial.println("BUTTON2 -> lock / unlock toggle after first TCP connection");
  Serial.println("Default mode: AUTO_MODE");

  if (!mpuInit()) {
    Serial.println("MPU init failed.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU init OK.");
  calibrateLevel();

  Serial.println("System ready.");
}

// =====================================================
// loop
// =====================================================
void loop() {
  // 1) BOOT 长按清配对
  handlePowerButton();

  // 2) button1 / button2
  handleButton1();
  handleButton2();

  // 3) 按当前模式处理
  if (currentMode == AUTO_MODE) {
    handleAutoModeTouchpad();
  } else {
    if (skipManualCmdThisLoop) {
      skipManualCmdThisLoop = false;
    } else {
      handleManualModeIMU();
    }
  }

  delay(10);
}
