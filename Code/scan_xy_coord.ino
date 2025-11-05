#include <Wire.h>

#define I2C_ADDR 0x74      // 你扫到的地址
#define SDA_PIN 21
#define SCL_PIN 22
#define RST_PIN 23
#define RDY_PIN 19         // 可选：若未接，可注释所有相关代码
#define LED_PIN 2

// ---- IQS5xx(B000) 常用寄存器 ----
#define REG_NUM_FINGERS   0x0011
#define REG_ABS_X_F1      0x0017
#define REG_ABS_Y_F1      0x0019
#define REG_X_RES         0x066F
#define REG_Y_RES         0x0671
#define REG_END_WINDOW    0xEEEE  // 写任意1字节结束通信窗口

// 读 len 字节（16位寄存器地址）
bool iqs_read(uint16_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission((uint8_t)I2C_ADDR);
  Wire.write((uint8_t)(reg >> 8));   // 高字节
  Wire.write((uint8_t)(reg & 0xFF)); // 低字节
  if (Wire.endTransmission(false) != 0) return false; // 重启，不发stop

  uint8_t n = Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)len); // ★ 强制uint8_t
  if (n != len) return false;
  for (uint8_t i=0; i<len; i++) buf[i] = Wire.read();
  return true;
}

// 写 1 字节到16位寄存器
bool iqs_write1(uint16_t reg, uint8_t val) {
  Wire.beginTransmission((uint8_t)I2C_ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

// 读取 16 位无符号（MSB:LSB）
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

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(RDY_PIN, INPUT); // 若没接，可不读它

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 不稳再降 50k/20k

  hardResetTP();

  // 读一下分辨率（可选）
  uint16_t xRes=0, yRes=0;
  if (iqs_read_u16(REG_X_RES, xRes) && iqs_read_u16(REG_Y_RES, yRes)) {
    Serial.print("X_RES="); Serial.print(xRes);
    Serial.print("  Y_RES="); Serial.println(yRes);
  } else {
    Serial.println("未读到分辨率，继续输出原始坐标。");
  }
}

void loop() {
  // 若未接RDY，直接删掉下面两行
  if (digitalRead(RDY_PIN) == LOW) { // 低=无新数据（常见配置）
    delay(1);
    return;
  }

  // 读取手指数
  uint8_t nf = 0;
  if (!iqs_read(REG_NUM_FINGERS, &nf, 1)) {
    iqs_write1(REG_END_WINDOW, 0x00); // 尝试结束窗口
    return;
  }

  if (nf > 0) {
    uint16_t x=0, y=0;
    bool okx = iqs_read_u16(REG_ABS_X_F1, x);
    bool oky = iqs_read_u16(REG_ABS_Y_F1, y);
    if (okx && oky) {
      digitalWrite(LED_PIN, HIGH);
      Serial.print("Finger1: X="); Serial.print(x);
      Serial.print("  Y="); Serial.println(y);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // 结束通信窗口，进入下一处理周期
  iqs_write1(REG_END_WINDOW, 0x00);
  delay(1);
}
