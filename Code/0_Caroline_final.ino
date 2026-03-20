#include <Wire.h>
#include <BleCombo.h>     // ESP32_BLE_Combo_Keyboard_Mouse
#include <esp_sleep.h>
#include <esp_gap_ble_api.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <stdlib.h>

// ===== BLE 组合设备：先键盘，再把鼠标关联上 =====
BleComboKeyboard bleKB("HeadsUp-Remote KBM v5", "Caroline-496-remote", 100);
BleComboMouse    bleMouse(&bleKB);

// ===== 引脚 / I2C 地址 =====
#define I2C_ADDR     0x74
#define SDA_PIN      21
#define SCL_PIN      22
#define RST_PIN      23
#define RDY_PIN      19
#define LED_PIN      2
#define BUTTON_BOOT  0    // BOOT（按下=LOW）
#define BUTTON1_PIN  33   // 外接按钮 button1（按下=LOW）
#define BUTTON2_PIN  32   // 外接按钮 button2（按下=LOW）

// ===== IQS5xx 常用寄存器 =====
#define REG_NUM_FINGERS      0x0011
#define REG_ABS_X_F1         0x0017
#define REG_ABS_Y_F1         0x0019
#define REG_GESTURE_EVENTS0  0x000D   // bit0: SINGLE_TAP
#define REG_END_WINDOW       0xEEEE

// ===== 鼠标移动参数 =====
const int MOVE_DIV = 30;
const int MOVE_CAP = 7;
const int MOTION_THRESH = 8;
bool INVERT_Y = false;  // 上下反就改成 true

// ===== BOOT 参数 =====
const unsigned long CLEAR_BOND_LONG_MS = 5000;  // 长按5秒：清除旧 BLE 配对并重启
bool btnPrev = false;
unsigned long btnDownAt = 0;
bool clearBondTriggered = false;

// ===== button1 / button2 状态 =====
bool button1Prev = false;
bool button2Prev = false;

// ===== 上一帧坐标 =====
uint16_t lastX = 0, lastY = 0;
bool haveLast = false;

// ===== I2C helpers =====
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

// ===== 触控板硬复位 =====
static void hardResetTP() {
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(20);
  digitalWrite(RST_PIN, HIGH);
  delay(50);
}

// ===== 清除所有 BLE bonded devices =====
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

// ===== 处理“长按5秒清配对”的按钮逻辑 =====
void handlePowerButton() {
  bool pressed = (digitalRead(BUTTON_BOOT) == LOW);

  // 刚按下：记录时间
  if (pressed && !btnPrev) {
    btnDownAt = millis();
    clearBondTriggered = false;
  }

  // 按住超过5秒：清除旧配对并重启
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

// ===== 处理 button1：目前只打印 =====
void handleButton1() {
  bool pressed = (digitalRead(BUTTON1_PIN) == LOW);

  if (pressed && !button1Prev) {
    Serial.println("button1 pressed");
  }

  button1Prev = pressed;
}

// ===== 处理 button2：目前只打印 =====
void handleButton2() {
  bool pressed = (digitalRead(BUTTON2_PIN) == LOW);

  if (pressed && !button2Prev) {
    Serial.println("button2 pressed");
  }

  button2Prev = pressed;
}


/************************************************* Set up *************************************************/

void setup() {
  Serial.begin(115200);
  delay(100);

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
  Serial.println("BLE Combo (Keyboard+Mouse) started");
  Serial.println("BOOT hold 5s -> clear BLE bonds and restart");
  Serial.println("BUTTON1 ready on GPIO33");
  Serial.println("BUTTON2 ready on GPIO32");
}


/************************************************* Main *************************************************/

void loop() {
  bool connected = bleKB.isConnected();

  // —— 1) 读手势事件（单击）+ 坐标（鼠标移动） ——
  if (digitalRead(RDY_PIN) != LOW) { // 极性不对就改成 == LOW
    bool anyData = false;

    // 手势：SINGLE_TAP → 左键点击
    uint8_t ge0 = 0;
    if (iqs_read(REG_GESTURE_EVENTS0, &ge0, 1)) {
      anyData = true;
      if (ge0 & (1 << 0)) {
        if (connected) bleMouse.click(MOUSE_LEFT);
        Serial.println("[gesture] SINGLE_TAP -> left click");
      }
    }

    // 坐标 → 鼠标移动
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
          } else {
            int16_t dx = (int16_t)x - (int16_t)lastX;
            int16_t dy = (int16_t)y - (int16_t)lastY;

            if (abs(dx) > MOTION_THRESH || abs(dy) > MOTION_THRESH) {
              int mx = dx / MOVE_DIV;
              int my = dy / MOVE_DIV;
              if (mx >  MOVE_CAP) mx =  MOVE_CAP;
              if (mx < -MOVE_CAP) mx = -MOVE_CAP;
              if (my >  MOVE_CAP) my =  MOVE_CAP;
              if (my < -MOVE_CAP) my = -MOVE_CAP;
              if (INVERT_Y) my = -my;

              if (connected) bleMouse.move((int8_t)mx, (int8_t)my);
              lastX = x;
              lastY = y;
            }
          }
        }
      } else {
        haveLast = false;
      }
    }

    digitalWrite(LED_PIN, anyData ? HIGH : LOW);
    iqs_write1(REG_END_WINDOW, 0x00);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // —— 2) 处理“长按5秒清配对” ——
  handlePowerButton();

  // —— 3) 处理 button1 / button2 ——
  handleButton1();
  handleButton2();

  delay(1);
}
