#include <Arduino.h> // essential for platform io
#include <esp_now.h> 
#include <WiFi.h>
#include <ESP32Servo.h> //necessary for esp32 to use servos 

// ================= HARDWARE DEFINITIONS =================
const int SERVO_PIN = 4;
Servo gripper;

// ================= COMMUNICATIONS HUB =================
uint8_t bridgeAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // MAC address of the bridge MCU's ESP-NOW interface, which we will send telemetry data to and receive commands from. then we forward these commands to the Drive MCU over Serial2, and send telemetry data back to the bridge MCU over ESP-NOW.
esp_now_peer_info_t peerInfo; 

typedef struct
{
  float ang;
  float targ;
  float err;
  float p_term;
  float i_term;
  float d_term;
  float spd_in;
  float pos_err;
  float mot_l;
  float mot_r;
  float bat;
  float kp_a;
  float kd_a;
  float kp_s;
  float ki_s;
  float kp_y;
  float base_ang;
} TelemetryData;
TelemetryData telem;

typedef struct
{
  char text[32];
} CommandMsg;
CommandMsg incomingCmd;

// ================= ESP-NOW RECEIVER =================
//void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) // for platform io version2 of arduino framework
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len)
{
  const uint8_t *mac_addr = recv_info->src_addr;
  memcpy(&incomingCmd, incomingData, sizeof(incomingCmd));

  // Isolate and execute the servo logic directly on the Cortex
  if (incomingCmd.text[0] == 'G')
  {
    gripper.write(atoi(incomingCmd.text + 2));
  }
  // Forward all physics, joystick, and EEPROM commands to the Drive MCU
  else
  {
    Serial2.print(incomingCmd.text);
    Serial2.print('\n');
  }
}

void setup()
{
  //Serial.begin(115200);

  // Initialize the high-speed hardware UART link to the Drive MCU
  Serial2.begin(500000, SERIAL_8N1, 16, 17);

  // Initialize the Gripper Servo safely away from the physics timers
  ESP32PWM::allocateTimer(0);
  gripper.setPeriodHertz(50);
  gripper.attach(SERVO_PIN, 500, 2500); //mg995MG 500-2400us pulse width range

  // Initialize ESP-NOW on Channel 1
  WiFi.mode(WIFI_STA); // ESP-NOW requires WiFi to be in Station mode, but we won't actually connect to any WiFi network. This also frees up the WiFi hardware for ESP-NOW communication.
  WiFi.setSleep(false);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv); // Register the receive callback function to handle incoming ESP-NOW messages.

  memcpy(peerInfo.peer_addr, bridgeAddress, 6); // memcpy is used to copy the MAC address of the Drive MCU into the peerInfo structure. 6 stands for the 6 bytes in a MAC address.
  peerInfo.channel = 1; // ESP-NOW operates on a specific WiFi channel. Both devices must be on the same channel to communicate. Channel 1 is a common choice, but you can choose any channel from 1 to 13 depending on your environment and potential interference.
  peerInfo.encrypt = false; // We are not using encryption for this simple telemetry and command exchange, but ESP-NOW does support encrypted communication if needed for security.
  peerInfo.ifidx = WIFI_IF_STA; // Specify that this peer is on the Station interface, which is the only interface available when WiFi is in Station mode.
  esp_now_add_peer(&peerInfo); // Add the Drive MCU as a peer so we can send telemetry data to it.
}

void loop()
{
  // Check if we have enough bytes for the header (2) + the payload
  while (Serial2.available() >= sizeof(TelemetryData) + 2) 
  {
    // Check if the first two bytes are our synchronization header
    if (Serial2.read() == 0xAA && Serial2.peek() == 0xAA) 
    {
      Serial2.read(); // Consume the second 0xAA byte
      
      // Now we know the next bytes are perfectly aligned!
      Serial2.readBytes((uint8_t *)&telem, sizeof(TelemetryData));
      esp_now_send(bridgeAddress, (uint8_t *)&telem, sizeof(TelemetryData));
    }
  }
}