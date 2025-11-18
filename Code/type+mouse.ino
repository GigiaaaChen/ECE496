#include <Wire.h>
#include <BleKeyboard.h>

// ---------- BLE 键盘（保持和你原来一样） ----------
BleKeyboard bleKeyboard("ESP32-Keyboard", "Espressif", 100);
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

// ---------- IQS5xx I2C helper ----------
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

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(buttonPin, INPUT_PULLUP); // BOOT 按键
  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(RDY_PIN, INPUT);          // 如果没接 RDY，可以改成 INPUT_PULLUP

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

  // 启动 BLE 键盘（和你原来一样）
  bleKeyboard.begin();
  Serial.println("BLE keyboard started, advertising as ESP32-Keyboard");
}

// ---------- loop ----------
void loop() {
  bool connected = bleKeyboard.isConnected();

  // ===== 1. 读触控板，把位移变成方向键 =====
  // 如果 RDY 引脚是低电平，表示没有新数据（如果你板子上是相反逻辑，可以把 LOW 改成 HIGH）
  if (digitalRead(RDY_PIN) != LOW) {
    uint8_t nf = 0;
    if (iqs_read(REG_NUM_FINGERS, &nf, 1)) {
      if (nf > 0) {
        uint16_t x = 0, y = 0;
        bool okx = iqs_read_u16(REG_ABS_X_F1, x);
        bool oky = iqs_read_u16(REG_ABS_Y_F1, y);

        if (okx && oky) {
          digitalWrite(LED_PIN, HIGH);
          // 串口看坐标
          Serial.print("Finger1: X="); Serial.print(x);
          Serial.print("  Y="); Serial.println(y);

          if (connected) {
            if (!haveLast) {
              // 第一次触摸，只记录不移动
              lastX = x;
              lastY = y;
              haveLast = true;
            } else {
              int16_t dx = (int16_t)x - (int16_t)lastX;
              int16_t dy = (int16_t)y - (int16_t)lastY;

              // 只有移动超过阈值才触发
              const int THRESH = 8;
              if (dx > THRESH || dx < -THRESH || dy > THRESH || dy < -THRESH) {

                int stepsX = dx / 30;  // 调这个缩放系数可以改变灵敏度
                int stepsY = dy / 30;

                // 限幅
                if (stepsX > 5) stepsX = 5;
                if (stepsX < -5) stepsX = -5;
                if (stepsY > 5) stepsY = 5;
                if (stepsY < -5) stepsY = -5;

                // Y 方向如果觉得反了，可以注释掉下一行或改成 stepsY = -stepsY;
                stepsY = -stepsY;

                // X 方向 → 左右方向键
                if (stepsX > 0) {
                  for (int i = 0; i < stepsX; i++) {
                    bleKeyboard.write(KEY_RIGHT_ARROW);
                    delay(5);
                  }
                } else if (stepsX < 0) {
                  for (int i = 0; i < -stepsX; i++) {
                    bleKeyboard.write(KEY_LEFT_ARROW);
                    delay(5);
                  }
                }

                // Y 方向 → 上下方向键
                if (stepsY > 0) {
                  for (int i = 0; i < stepsY; i++) {
                    bleKeyboard.write(KEY_DOWN_ARROW);
                    delay(5);
                  }
                } else if (stepsY < 0) {
                  for (int i = 0; i < -stepsY; i++) {
                    bleKeyboard.write(KEY_UP_ARROW);
                    delay(5);
                  }
                }

                lastX = x;
                lastY = y;
              }
            }
          }
        } else {
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

  // ===== 2. 保持你原来的 BOOT → 发送 'A' 功能 =====
  if (connected) {
    if (digitalRead(buttonPin) == LOW) {
      Serial.println("BOOT pressed, sending A...");
      bleKeyboard.print("A");
      delay(500); // 简单防抖
    }
  }

  delay(1);
}
