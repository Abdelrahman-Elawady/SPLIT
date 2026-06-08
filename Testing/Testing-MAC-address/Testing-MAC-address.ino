#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  // Set the Wi-Fi mode to Station to activate the antenna
  WiFi.mode(WIFI_MODE_STA); 
}

void loop() {
  Serial.println("=================================");
  Serial.print("THIS ESP32 MAC ADDRESS: ");
  Serial.println(WiFi.macAddress());
  Serial.println("=================================\n");
  delay(3000); // Wait 3 seconds and print again
}