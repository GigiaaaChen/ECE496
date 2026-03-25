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
// I2C 引脚 / 总线
// =====================================================
// touchpad 独立 I2C
#define TP_SDA_PIN   21
#define TP_SCL_PIN   22

// IMU 独立 I2C
#define IMU_SDA_PIN  14
#define IMU_SCL_PIN  26

TwoWire TPWire  = TwoWire(0);
TwoWire IMUWire = TwoWire(1);

// =====================================================
// touchpad
// =====================================================
#define I2C_ADDR     0x74
#define RST_PIN      23
#define RDY_PIN      19

// buttons / LED
#define LED_PIN      2
#define BUTTON_BOOT  0    // BOOT（按下=LOW）
#define BUTTON1_PIN  4    // mode toggle（按下=LOW）
#define BUTTON2_PIN  5    // lock/unlock toggle（按下=LOW）

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
const int MOVE_DIV = 40;
const int MOVE_CAP = 9;
const int MOTION_THRESH = 18;
bool INVERT_Y = false;

// =====================================================
// BOOT 参数
// =====================================================
const unsigned long CLEAR_BOND_LONG_MS = 5000;
bool btnPrev = false;
unsigned long btnDownAt = 0;
bool clearBondTriggered = false;

// =====================================================
// button1 / button2 状态
// =====================================================
bool button1Prev = false;
bool button2Prev = false;

// button2: lock / unlock 翻转状态
bool lockState = false;

// 避免 button2 在 manual mode 下触发后，同一轮 loop 又立刻发送 IMU 命令
bool skipManualCmdThisLoop = false;

// =====================================================
// 上一帧 touchpad 坐标
// =====================================================
uint16_t lastX = 0, lastY = 0;
bool haveLast = false;

// 连续接触开始时间：用于区分“精细微调”还是“快速大范围移动”
unsigned long touchStartTime = 0;

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

// TCP reliability controls
unsigned long lastSendMs = 0;
unsigned long lastHeartbeatMs = 0;
String lastSentCmd = "";
bool tcpSessionReady = false;

const unsigned long TCP_SEND_INTERVAL_MS   = 80;    // 最小发送间隔
const unsigned long TCP_HEARTBEAT_MS       = 1500;  // 空闲心跳
const unsigned long TCP_REPLY_TIMEOUT_MS   = 300;   // 等待 ACK/PONG
const unsigned long TCP_CONNECT_TIMEOUT_MS = 2000;  // connect 超时
const unsigned long TCP_WARMUP_READ_MS     = 150;   // 吃掉开头 banner
const bool TCP_PRINT_RXTX                  = true;

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
  TPWire.beginTransmission((uint8_t)I2C_ADDR);
  TPWire.write((uint8_t)(reg >> 8));
  TPWire.write((uint8_t)(reg & 0xFF));
  if (TPWire.endTransmission(false) != 0) return false;

  uint8_t n = TPWire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)len);
  if (n != len) return false;

  for (uint8_t i = 0; i < len; i++) buf[i] = TPWire.read();
  return true;
}

static bool iqs_read_u16(uint16_t reg, uint16_t &out) {
  uint8_t b[2];
  if (!iqs_read(reg, b, 2)) return false;
  out = ((uint16_t)b[0] << 8) | b[1];
  return true;
}

static bool iqs_write1(uint16_t reg, uint8_t val) {
  TPWire.beginTransmission((uint8_t)I2C_ADDR);
  TPWire.write((uint8_t)(reg >> 8));
  TPWire.write((uint8_t)(reg & 0xFF));
  TPWire.write(val);
  return TPWire.endTransmission(true) == 0;
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
  IMUWire.beginTransmission(MPU_ADDR);
  IMUWire.write(reg);
  IMUWire.write(data);
  uint8_t err = IMUWire.endTransmission(true);
  return (err == 0);
}

// =====================================================
// MPU6050: 连续读多个字节
// =====================================================
bool mpuReadBytes(uint8_t startReg, uint8_t *buf, uint8_t len) {
  IMUWire.beginTransmission(MPU_ADDR);
  IMUWire.write(startReg);

  if (IMUWire.endTransmission(false) != 0) {
    return false;
  }

  IMUWire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len);

  uint8_t i = 0;
  while (IMUWire.available() && i < len) {
    buf[i] = IMUWire.read();
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
    if (pitchRel >= STOP_RANGE) {
      return "LEFT";
    } else if (pitchRel <= -STOP_RANGE) {
      return "RIGHT";
    } else {
      return "STOP";
    }
  } else {
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
// TCP helpers
// =====================================================
void resetTcpClient() {
  if (client) {
    client.stop();
  }
  delay(50);
  tcpSessionReady = false;
}

bool readReplyLine(String &reply, unsigned long timeoutMs) {
  unsigned long start = millis();
  reply = "";

  while (millis() - start < timeoutMs) {
    while (client.available() > 0) {
      char c = (char)client.read();

      if (c == '\r') continue;

      if (c == '\n') {
        reply.trim();
        return true;
      }

      reply += c;
    }
    delay(2);
  }

  return false;
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

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected.");
    return false;
  }

  if (client.connected()) {
    return true;
  }

  resetTcpClient();

  Serial.println("Connecting to RDK server...");
  Serial.print("Host: ");
  Serial.println(host);
  Serial.print("Port: ");
  Serial.println(port);

  client.setTimeout(TCP_REPLY_TIMEOUT_MS);

  bool ok = client.connect(host, port, TCP_CONNECT_TIMEOUT_MS);

  if (!ok) {
    Serial.println("TCP connect failed.");
    resetTcpClient();
    return false;
  }

  client.setNoDelay(true);

  unsigned long warmupStart = millis();
  while (millis() - warmupStart < TCP_WARMUP_READ_MS) {
    while (client.available() > 0) {
      String s = client.readStringUntil('\n');
      s.trim();
      if (s.length() > 0) {
        Serial.print("RDK startup: ");
        Serial.println(s);
      }
    }
    delay(5);
  }

  Serial.println("TCP connected to RDK.");
  tcpSessionReady = true;
  return true;
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

  while (client.available() > 0) {
    String junk = client.readStringUntil('\n');
    junk.trim();
    if (junk.length() > 0) {
      Serial.print("Discarding stale reply: ");
      Serial.println(junk);
    }
  }

  client.print(line);
  client.print("\n");

  if (TCP_PRINT_RXTX) {
    Serial.print("Sent to RDK: ");
    Serial.println(line);
  }

  String resp;
  if (!readReplyLine(resp, TCP_REPLY_TIMEOUT_MS)) {
    Serial.println("No ACK from RDK -> resetting TCP.");
    resetTcpClient();
    return false;
  }

  if (TCP_PRINT_RXTX) {
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
  lastSentCmd = "";
  lastSendMs = 0;
  lastHeartbeatMs = 0;
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
  lastSentCmd = "";
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
void handleAutoModeTouchpad() {
  bool connected = bleKB.isConnected();

  static float vScrollAccum = 0.0f;
  static float hScrollAccum = 0.0f;
  static float dxScrollFilt = 0.0f;
  static float dyScrollFilt = 0.0f;

  const int   SCROLL_DEADBAND = 4;
  const float SCROLL_ALPHA    = 0.18f;
  const float SCROLL_GAIN_V   = 1.0f / 90.0f;
  const float SCROLL_GAIN_H   = 1.0f / 100.0f;
  const int   SCROLL_STEP_CAP = 1;
  const float AXIS_DOM_RATIO  = 1.25f;

  static uint8_t touchStableCount = 0;
  const uint8_t TOUCH_SETTLE_FRAMES = 4;

  if (digitalRead(RDY_PIN) != LOW) {
    bool anyData = false;

    uint8_t ge0 = 0;
    if (iqs_read(REG_GESTURE_EVENTS0, &ge0, 1)) {
      anyData = true;
      if (ge0 & (1 << 0)) {
        if (connected) bleMouse.click(MOUSE_LEFT);
        Serial.println("[gesture] SINGLE_TAP -> left click");
      }
    }

    uint8_t nf = 0;
    if (iqs_read(REG_NUM_FINGERS, &nf, 1)) {
      if (nf > 0) {
        uint16_t x = 0, y = 0;
        bool okx = iqs_read_u16(REG_ABS_X_F1, x);
        bool oky = iqs_read_u16(REG_ABS_Y_F1, y);

        if (okx && oky) {
          anyData = true;

          if (!haveLast) {
            lastX = x;
            lastY = y;
            haveLast = true;
            touchStableCount = 0;
            touchStartTime = millis();
          } else {
            if (touchStableCount < TOUCH_SETTLE_FRAMES) {
              lastX = x;
              lastY = y;
              touchStableCount++;
            } else {
              int16_t dx = (int16_t)x - (int16_t)lastX;
              int16_t dy = (int16_t)y - (int16_t)lastY;

              if (nf >= 2) {
                int wheel = 0;
                int hWheel = 0;

                dxScrollFilt = dxScrollFilt + SCROLL_ALPHA * ((float)dx - dxScrollFilt);
                dyScrollFilt = dyScrollFilt + SCROLL_ALPHA * ((float)dy - dyScrollFilt);

                float adx = fabs(dxScrollFilt);
                float ady = fabs(dyScrollFilt);

                if (ady > adx * AXIS_DOM_RATIO) {
                  if (ady >= SCROLL_DEADBAND) {
                    vScrollAccum += dyScrollFilt * SCROLL_GAIN_V;
                  }
                  hScrollAccum *= 0.6f;
                }
                else if (adx > ady * AXIS_DOM_RATIO) {
                  if (adx >= SCROLL_DEADBAND) {
                    hScrollAccum += (-dxScrollFilt) * SCROLL_GAIN_H;
                  }
                  vScrollAccum *= 0.6f;
                }
                else {
                  vScrollAccum *= 0.85f;
                  hScrollAccum *= 0.85f;
                }

                while (vScrollAccum >= 1.0f && wheel < SCROLL_STEP_CAP) {
                  wheel++;
                  vScrollAccum -= 1.0f;
                }
                while (vScrollAccum <= -1.0f && wheel > -SCROLL_STEP_CAP) {
                  wheel--;
                  vScrollAccum += 1.0f;
                }

                while (hScrollAccum >= 1.0f && hWheel < SCROLL_STEP_CAP) {
                  hWheel++;
                  hScrollAccum -= 1.0f;
                }
                while (hScrollAccum <= -1.0f && hWheel > -SCROLL_STEP_CAP) {
                  hWheel--;
                  hScrollAccum += 1.0f;
                }

                if (connected && (wheel != 0 || hWheel != 0)) {
                  bleMouse.move(0, 0, (int8_t)wheel, (int8_t)hWheel);
                }

                lastX = x;
                lastY = y;
              } else {
                if (abs(dx) > abs(dy) * 2) {
                  dy = 0;
                } else if (abs(dy) > abs(dx) * 2) {
                  dx = 0;
                }

                if (abs(dx) < MOTION_THRESH) dx = 0;
                if (abs(dy) < MOTION_THRESH) dy = 0;

                if (dx != 0 || dy != 0) {
                  unsigned long touchAge = millis() - touchStartTime;

                  int moveDivNow = MOVE_DIV;
                  int moveCapNow = MOVE_CAP;

                  if (touchAge < 180) {
                    moveDivNow = MOVE_DIV / 2;
                    if (moveDivNow < 1) moveDivNow = 1;
                    moveCapNow = MOVE_CAP + 3;
                  } else if (touchAge < 600) {
                    moveDivNow = MOVE_DIV;
                    moveCapNow = MOVE_CAP;
                  } else {
                    moveDivNow = MOVE_DIV * 2;
                    moveCapNow = (MOVE_CAP > 3) ? 3 : MOVE_CAP;
                  }

                  int mx = -dx / moveDivNow;
                  int my = -dy / moveDivNow;

                  if (dx != 0 && mx == 0) mx = (dx > 0) ? -1 : 1;
                  if (dy != 0 && my == 0) my = (dy > 0) ? -1 : 1;

                  if (mx >  moveCapNow) mx =  moveCapNow;
                  if (mx < -moveCapNow) mx = -moveCapNow;
                  if (my >  moveCapNow) my =  moveCapNow;
                  if (my < -moveCapNow) my = -moveCapNow;

                  if (connected) {
                    bleMouse.move((int8_t)mx, (int8_t)my);
                  }
                }

                lastX = x;
                lastY = y;
              }
            }
          }
        }
      } else {
        haveLast = false;
        touchStableCount = 0;
        touchStartTime = 0;

        vScrollAccum = 0.0f;
        hScrollAccum = 0.0f;
        dxScrollFilt = 0.0f;
        dyScrollFilt = 0.0f;
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

  unsigned long now = millis();
  bool shouldSend = false;

  if (cmd != lastSentCmd) {
    shouldSend = true;
  } else if (now - lastSendMs >= TCP_HEARTBEAT_MS) {
    shouldSend = true;
  }

  if (shouldSend && (now - lastSendMs < TCP_SEND_INTERVAL_MS)) {
    shouldSend = false;
  }

  if (shouldSend) {
    if (sendLineToRDK(cmd)) {
      lastSentCmd = cmd;
      lastSendMs = now;
    } else {
      Serial.println("Manual command send failed.");
    }
  }
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

  TPWire.begin(TP_SDA_PIN, TP_SCL_PIN);
  TPWire.setClock(100000);

  IMUWire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  IMUWire.setClock(100000);

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
  handlePowerButton();
  handleButton1();
  handleButton2();

  if (currentMode == AUTO_MODE) {
    handleAutoModeTouchpad();
  } else {
    if (skipManualCmdThisLoop) {
      skipManualCmdThisLoop = false;
    } else {
      handleManualModeIMU();
    }

    if (manualNetworkReady) {
      unsigned long now = millis();
      if ((now - lastSendMs >= TCP_HEARTBEAT_MS) &&
          (now - lastHeartbeatMs >= TCP_HEARTBEAT_MS)) {
        if (sendLineToRDK("PING")) {
          lastHeartbeatMs = now;
          lastSendMs = now;
        }
      }
    }
  }

  delay(10);
}