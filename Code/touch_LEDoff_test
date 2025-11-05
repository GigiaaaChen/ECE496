#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 2
#define RST_PIN 23
#define RDY_PIN 19    // 连接到触控板RDY引脚

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(RDY_PIN, INPUT);

  // 复位触控板
  digitalWrite(RST_PIN, LOW);
  delay(20);
  digitalWrite(RST_PIN, HIGH);
  delay(50);
  
  Serial.println("Touchpad test start...");
}

void loop() {
  int rdy = digitalRead(RDY_PIN);

  // 如果RDY引脚拉低（一般代表有触摸），点亮LED
  if (rdy == LOW) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Touch detected!");
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  delay(10);
}
