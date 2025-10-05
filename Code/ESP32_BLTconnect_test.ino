#include <BleKeyboard.h>

BleKeyboard bleKeyboard("ESP32-Keyboard", "Espressif", 100);

int buttonPin = 0; // BOOT button

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // press = LOW
  Serial.begin(115200);

  bleKeyboard.begin(); // enable BLE keyboard
}

void loop() {
  if (bleKeyboard.isConnected()) {   // comfirm connection
    if (digitalRead(buttonPin) == LOW) {
      Serial.println("BOOT pressed, sending A...");
      bleKeyboard.print("A"); // test: A will appear if successfully connected
      delay(500); 
    }
  }
}
