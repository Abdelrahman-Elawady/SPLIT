#include <esp_now.h>
#include <WiFi.h>

// The exact same data structure as the Transceiver
typedef struct struct_message {
  char text[32];
} struct_message;

struct_message myData;

// This function fires automatically whenever a packet arrives
void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  Serial.print(F("INCOMING COMMAND: "));
  Serial.println(myData.text);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }
  
  // Tell the ESP32 to run the OnDataRecv function when data hits the antenna
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println(F("Robot Receiver Ready. Listening..."));
}

void loop() {
  // The ESP32 handles the radio reception in the background!
  delay(10);
}