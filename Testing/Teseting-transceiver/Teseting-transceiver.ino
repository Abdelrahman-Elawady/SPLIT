#include <esp_now.h>
#include <WiFi.h>

// REPLACE THIS with your Robot's MAC Address
// Example: If MAC is 00:00:00:00:00:00, write it like below:
uint8_t broadcastAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// The data structure we will send (a 32-character string)
typedef struct struct_message {
  char text[32];
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }

  // Register the Robot as a peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println(F("Failed to add peer"));
    return;
  }
  
  Serial.println(F("Transceiver Ready. Type a command and hit enter..."));
}

void loop() {
  if (Serial.available()) {
    // Read the text you type into the Serial Monitor
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    
    // Copy the text into our ESP-NOW packet
    input.toCharArray(myData.text, 32);
    
    // Blast it out over the air
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
      Serial.print(F("Sent: "));
      Serial.println(myData.text);
    } else {
      Serial.println(F("Transmission Failed."));
    }
  }
}