#include <Wire.h>
#include <BleCombo.h>  // 你的库：ESP32_BLE_Combo_Keyboard_Mouse

// ---------- BLE 合体设备（先键盘，再让鼠标附着在键盘上） ----------
BleComboKeyboard bleKeyboard("HeadsUp-Remote", "Espressif", 100);
BleComboMouse    bleMouse(&bleKeyboard);   // 关键：把键盘指针传给鼠标
int buttonPin = 0; // BOOT 按键，按下 = LOW

// ---------- 触控板 IQS5xx 定义 ----------
#define I2C_ADDR 0x74
#define SDA_PIN 21
#define SCL_PIN 22
#define RST_PIN 23
#define RDY_PIN 19
#define LED_PIN 2

#define REG_NUM_FINGERS   0x0011
#define REG_ABS_X_F1      0x0017
#define REG_ABS_Y_F1      0x0019
#define REG_X_RES         0x066F
#define REG_Y_RES         0x0671
#define REG_END_WINDOW    0xEEEE

// 上一次坐标，用来算相对位移
uint16_t lastX = 0;
uint16_t lastY = 0;
bool haveLast = false;

// ===== IQS5xx I2C helpers =====
bool iqs_read(uint16_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission((uint8_t)I2C_ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)len);
  if (n != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool iqs_write1(uint16_t reg, uint8_t val) {
  Wire.beginTransmission((uint8_t)I2C_ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool iqs_read_u16(uint16_t reg, uint16_t &out) {
  uint8_t b[2];
  if (!iqs_read(reg, b, 2)) return false;
  out = ((uint16_t)b[0] << 8) | b[1];
  return true;
}

void hardResetTP() {
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(20);
  digitalWrite(RST_PIN, HIGH);
  delay(50);
}

// ===== 参数：灵敏度/阈值/限幅 =====
const int MOVE_DIV = 30;     // 越小越灵敏（dx/MOVE_DIV）
const int MOVE_CAP = 7;      // 每次发送的最大步进
const int MOTION_THRESH = 8; // 触发移动的最小位移

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(buttonPin, INPUT_PULLUP); // BOOT
  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(RDY_PIN, INPUT);          // 若未接，可换 INPUT_PULLUP

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // 触控板复位
  hardResetTP();

  // 读分辨率（可选）
  uint16_t xRes = 0, yRes = 0;
  if (iqs_read_u16(REG_X_RES, xRes) && iqs_read_u16(REG_Y_RES, yRes)) {
    Serial.print("X_RES="); Serial.print(xRes);
    Serial.print("  Y_RES="); Serial.println(yRes);
  } else {
    Serial.println("未读到分辨率，继续使用原始坐标。");
  }

  // 启动 BLE（先键盘）
  bleKeyboard.begin();
  Serial.println("BLE HID started (keyboard+mouse), advertising as HeadsUp-Remote");
}

void loop() {
  bool connected = false;
  // 某些分支把 isConnected() 放在键盘对象上：
  #if defined(ARDUINO)
    connected = bleKeyboard.isConnected();
  #endif

  // ===== 1) 读触控板 → 发送鼠标移动，同时点亮 LED =====
  // 如果 RDY 为低表示没新数据（若你的板子相反，就把 != LOW 改成 == LOW）
  if (digitalRead(RDY_PIN) != LOW) {
    uint8_t nf = 0;
    if (iqs_read(REG_NUM_FINGERS, &nf, 1)) {
      if (nf > 0) {
        uint16_t x = 0, y = 0;
        bool okx = iqs_read_u16(REG_ABS_X_F1, x);
        bool oky = iqs_read_u16(REG_ABS_Y_F1, y);

        if (okx && oky) {
          // 有有效触摸数据 → 点亮 LED
          digitalWrite(LED_PIN, HIGH);

          // 串口输出坐标
          Serial.print("Finger1: X="); Serial.print(x);
          Serial.print("  Y="); Serial.println(y);

          if (connected) {
            if (!haveLast) {
              lastX = x;
              lastY = y;
              haveLast = true;
            } else {
              int16_t dx = (int16_t)x - (int16_t)lastX;
              int16_t dy = (int16_t)y - (int16_t)lastY;

              if (abs(dx) > MOTION_THRESH || abs(dy) > MOTION_THRESH) {
                // 缩放成鼠标相对位移
                int mx = dx / MOVE_DIV;
                int my = dy / MOVE_DIV;

                // 限幅
                if (mx >  MOVE_CAP) mx =  MOVE_CAP;
                if (mx < -MOVE_CAP) mx = -MOVE_CAP;
                if (my >  MOVE_CAP) my =  MOVE_CAP;
                if (my < -MOVE_CAP) my = -MOVE_CAP;

                // 若 Y 方向感觉反了，解注释下一行（或删掉反向）
                my = -my;

                // 发送鼠标移动（相对位移）；该库常用 API：move(x, y)
                bleMouse.move((int8_t)mx, (int8_t)my);

                lastX = x;
                lastY = y;
              }
            }
          }
        } else {
          // 读失败 → 熄灭
          digitalWrite(LED_PIN, LOW);
        }
      } else {
        // 没有手指
        digitalWrite(LED_PIN, LOW);
        haveLast = false;
      }
    }

    // 结束一次 I2C 通信窗口
    iqs_write1(REG_END_WINDOW, 0x00);
  }

  // ===== 2) BOOT 键发送 'A'（键盘） =====
  if (connected) {
    if (digitalRead(buttonPin) == LOW) {
      Serial.println("BOOT pressed, sending 'A'...");
      bleKeyboard.print("A"); // 你的库没有 KEY_A，用 print 最稳
      delay(500); // 简易防抖
    }
  }

  delay(1);
}
