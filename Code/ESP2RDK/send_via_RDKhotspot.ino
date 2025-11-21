#include <WiFi.h>
#include <WiFiClient.h>

// ====== 改成你的 RDK 热点 ======
const char* ssid     = "RDKX5-Hotspot";   // 你的热点 SSID（例）
const char* password = "ECE49666";          // 你的热点密码
#include <WiFi.h>
#include <WiFiClient.h>

// ====== RDK 作为“服务器”的地址 ======
// ① 如果是 NetworkManager Hotspot（ipv4.method=shared 的常见默认）：RDK IP 通常为 10.42.0.1
// ② 如果是 hostapd + isc-dhcp-server（我们之前示例）：RDK IP 为 10.5.5.1
const char* host = "10.42.0.1";             // 如果你用的是 NM 热点，用这个
// const char* host = "10.5.5.1";           // 如果你用的是 hostapd 方案，改成这个
const int   port = 5000;                    // RDK 端监听的端口（你自己开的服务）

WiFiClient client;

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                     // 降低延迟，连接更稳定
  Serial.printf("Connecting to SSID: %s\n", ssid);
  WiFi.begin(ssid, password);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if (millis() - t0 > 20000) {            // 20s 超时重试
      Serial.println("\nWiFi connect timeout, retrying...");
      WiFi.disconnect(true);
      delay(1000);
      WiFi.begin(ssid, password);
      t0 = millis();
    }
  }
  Serial.printf("\nWiFi connected. ESP32 IP: %s  RSSI:%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

void setup() {
  Serial.begin(115200);
  delay(100);
  connectWiFi();
}

void loop() {
  // 若 Wi-Fi 断开，自动重连
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi dropped. Reconnecting...");
    connectWiFi();
    delay(500);
    return;
  }

  // 若 TCP 未连上服务器，则尝试连接
  if (!client.connected()) {
    Serial.printf("Connecting to RDK server %s:%d ...\n", host, port);
    if (client.connect(host, port)) {
      client.setNoDelay(true);
      Serial.println("Connected to RDK!");
    } else {
      Serial.println("TCP connect failed. Retry in 1s...");
      delay(1000);
      return;
    }
  }

  // 发送数据
  const char* msg = "Hi I'm Caroline\n";
  client.print(msg);
  Serial.printf("Sent: %s", msg);

  // 也可以读取 RDK 端回显
  while (client.available() > 0) {
    String line = client.readStringUntil('\n');
    Serial.printf("Rcv: %s\n", line.c_str());
  }

  delay(1000);
}
