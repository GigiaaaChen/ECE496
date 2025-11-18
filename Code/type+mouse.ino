#include <Wire.h>
#include <BleCombo.h>     // ESP32_BLE_Combo_Keyboard_Mouse
#include <esp_sleep.h>    // 深度睡眠

// ===== BLE 组合设备：先键盘，再把鼠标关联上 =====
BleComboKeyboard bleKB("HeadsUp-Remote KBM v5", "Espressif", 100);
BleComboMouse    bleMouse(&bleKB);

// ===== 引脚 / I2C 地址 =====
#define I2C_ADDR  0x74
#define SDA_PIN   21
#define SCL_PIN   22
#define RST_PIN   23
#define RDY_PIN   19
#define LED_PIN   2
#define BUTTON_BOOT 0  // BOOT（按下=LOW），我们把它当“电源键”

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

// ===== BOOT 长按关机参数 =====
const unsigned long POWER_LONG_MS = 2000;  // 长按阈值（2s）
bool btnPrev = false;
unsigned long btnDownAt = 0;

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
static void hardResetTP() {
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(20);
  digitalWrite(RST_PIN, HIGH);
  delay(50);
}

// ===== 进入“软关机”（深度睡眠），BOOT 再按一次开机 =====
static void enterSoftPowerOff() {
  Serial.println("Entering Deep Sleep (soft power off) ...");
  digitalWrite(LED_PIN, LOW);
  // 用 EXT0 在 BOOT=LOW 时唤醒（GPIO0 是低电平触发）
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // level 0 唤醒
  delay(50);
  esp_deep_sleep_start();
}

// ===== 处理“长按关机 / 短按打字”的按钮逻辑 =====
void handlePowerButton(bool bleConnected) {
  bool pressed = (digitalRead(BUTTON_BOOT) == LOW);

  // 刚按下：记录时间
  if (pressed && !btnPrev) {
    btnDownAt = millis();
  }

  // 按住超过阈值：关机
  if (pressed && btnPrev) {
    unsigned long held = millis() - btnDownAt;
    if (held >= POWER_LONG_MS) {
      // 长按期间常亮 LED 提示（可选）
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      enterSoftPowerOff();  // 不再返回
    }
  }

  // 松开：如果按得不够长，当成“短按”→ 打字 'A'
  if (!pressed && btnPrev) {
    unsigned long held = millis() - btnDownAt;
    if (held < POWER_LONG_MS) {
      if (bleConnected) {
        bleKB.print("A");      // 短按 → 打字
        Serial.println("BOOT short press -> 'A'");
      }
    }
  }

  btnPrev = pressed;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(RDY_PIN, INPUT);          // 如需上拉可改 INPUT_PULLUP
  pinMode(BUTTON_BOOT, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  hardResetTP();

  bleKB.begin();
  Serial.println("BLE Combo (Keyboard+Mouse) started; BOOT long-press=Power, short-press=type 'A'");
}

void loop() {
  bool connected = bleKB.isConnected();

  // —— 1) 读手势事件（单击）+ 坐标（鼠标移动） ——
  if (digitalRead(RDY_PIN) != LOW) { // 极性不对就改成 == LOW
    bool anyData = false;

    // 手势：SINGLE_TAP → 左键点击
    uint8_t ge0 = 0;
    if (iqs_read(REG_GESTURE_EVENTS0, &ge0, 1)) {
      anyData = true;
      if (ge0 & (1 << 0)) { // SINGLE_TAP
        if (connected) bleMouse.click(MOUSE_LEFT);
        Serial.println("[gesture] SINGLE_TAP -> left click");
      }
    }

    // 坐标 → 鼠标移动
    uint8_t nf = 0;
    if (iqs_read(REG_NUM_FINGERS, &nf, 1)) {
      if (nf > 0) {
        uint16_t x=0, y=0;
        bool okx = iqs_read_u16(REG_ABS_X_F1, x);
        bool oky = iqs_read_u16(REG_ABS_Y_F1, y);
        if (okx && oky) {
          anyData = true;
          if (!haveLast) { lastX = x; lastY = y; haveLast = true; }
          else {
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
              lastX = x; lastY = y;
            }
          }
        }
      } else {
        haveLast = false;
      }
    }

    digitalWrite(LED_PIN, anyData ? HIGH : LOW);
    iqs_write1(REG_END_WINDOW, 0x00); // 结束通信窗口
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // —— 2) 处理“长按关机 / 短按打字” —— 
  handlePowerButton(connected);

  delay(1);
}
