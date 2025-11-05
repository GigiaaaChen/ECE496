// 测试能不能读到连接的I2C device

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 2

// 为了在没有外部上拉的情况下更稳一点
void setupI2C() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 降速：100 kHz（不稳可再降到 50000/20000）
}

uint32_t scan_no = 0;          // 扫描轮次计数
uint32_t scans_with_74 = 0;    // 统计发现 0x74 的轮次
uint32_t consecutive_miss = 0; // 连续未发现 0x74 的次数

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setupI2C();
  Serial.println("\n[INIT] I2C stability scanner (idx & stats)");
}

void loop() {
  scan_no++;
  bool found_any = false;
  bool found_74 = false;
  int  first_idx_74 = -1;

  unsigned long t_scan0 = millis();

  Serial.printf("\n[SCAN #%lu] start\n", (unsigned long)scan_no);

  int idx = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    idx++;
    unsigned long t0 = micros();
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    unsigned long dt = micros() - t0;

    if (err == 0) {
      found_any = true;
      Serial.printf("  -> found: 0x%02X (idx=%d, ack=%lu us)\n", addr, idx, dt);
      if (addr == 0x74 && first_idx_74 < 0) {
        found_74 = true;
        first_idx_74 = idx;
      }
    }
  }

  unsigned long scan_ms = millis() - t_scan0;

  if (!found_any) {
    Serial.printf("[SCAN #%lu] NO I2C devices. time=%lums\n", (unsigned long)scan_no, scan_ms);
  } else {
    Serial.printf("[SCAN #%lu] done. time=%lums\n", (unsigned long)scan_no, scan_ms);
  }

  if (found_74) {
    scans_with_74++;
    consecutive_miss = 0;
    digitalWrite(LED_PIN, HIGH);
    Serial.printf("  >> 0x74 PRESENT (first idx=%d). presence rate=%.1f%%\n",
                  first_idx_74,
                  100.0 * scans_with_74 / scan_no);
  } else {
    consecutive_miss++;
    digitalWrite(LED_PIN, LOW);
    Serial.printf("  >> 0x74 NOT found. consecutive_miss=%lu, presence rate=%.1f%%\n",
                  (unsigned long)consecutive_miss,
                  100.0 * scans_with_74 / scan_no);
  }

  // 扫描间隔；不稳时可调大一些
  delay(700);
}
