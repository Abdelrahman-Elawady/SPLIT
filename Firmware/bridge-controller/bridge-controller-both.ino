#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h> 
#include <WebSocketsServer.h> 
#include <SPIFFS.h> 

const char *ssid = "Robot_AP"; 
const char *password = "12345678"; 
const float DEFAULT_TURN_SPEED = 35.0; //adjust this to change how fast the robot turns

WebServer server(80); 
WebSocketsServer webSocket = WebSocketsServer(81); 

// ================= DUAL ROBOT MAC ADDRESSES =================
//use the mac address testing code to find the mac address of your specific MCU printed at the loop
uint8_t mac_robot1[] = {0x04, 0xB2, 0x47, 0x9C, 0x7E, 0x10}; //04:B2:47:9C:7E:10
uint8_t mac_robot2[] = {0x28, 0x05, 0xA5, 0x2C, 0xF0, 0x80}; //28:05:A5:2C:F0:80
esp_now_peer_info_t peerInfo;

typedef struct {
  char text[32]; 
} CommandMsg;
CommandMsg myData;

typedef struct {
  float ang; float targ; float err; float p_term; float i_term; float d_term;
  float spd_in; float pos_err; float mot_l; float mot_r; float bat;
  float kp_a; float kd_a; float kp_s; float ki_s; float kp_y; float base_ang;
} TelemetryData;

// Arrays to hold separate data for Robot 1 and Robot 2
TelemetryData holdingBuffer[2];
volatile bool newDataReady[2] = {false, false};

void sendEspNowCommand(const uint8_t *targetMac, const String &command)
{
  memset(myData.text, 0, sizeof(myData.text));
  command.substring(0, 31).toCharArray(myData.text, sizeof(myData.text));
  esp_now_send(targetMac, (uint8_t *)&myData, sizeof(myData));
}

void sendDriveCommand(const uint8_t *targetMac, float fwd, float spin)
{
  sendEspNowCommand(targetMac, "B:" + String(fwd, 1));
  delay(20);
  sendEspNowCommand(targetMac, "T:" + String(spin, 1));
}

bool sendMoveCommand(const uint8_t *targetMac, String move, float speed, float turnSpeed)
{
  move.toUpperCase();

  if (move == "FORWARD")
  {
    sendDriveCommand(targetMac, -speed, 0);
  }
  else if (move == "BACKWARD")
  {
    sendDriveCommand(targetMac, speed, 0);
  }
  else if (move == "LEFT")
  {
    sendDriveCommand(targetMac, 0, turnSpeed);
  }
  else if (move == "RIGHT")
  {
    sendDriveCommand(targetMac, 0, -turnSpeed);
  }
  else if (move == "STOP" || move == "IDLE")
  {
    sendDriveCommand(targetMac, 0, 0);
  }
  else
  {
    return false;
  }

  return true;
}

// ================= ESP-NOW COMMUNICATIONS =================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  if (type == WStype_TEXT)
  {
    // The JS will now send payloads formatted as "1|B:5.0" or "2|P:200"
    if (length > 2 && payload[1] == '|') 
    {
      uint8_t targetRobot = payload[0] - '0'; // Convert char '1' or '2' to integer
      
      memset(myData.text, 0, sizeof(myData.text));
      size_t copy_len = length - 2;
      if (copy_len > 31) copy_len = 31;
      
      memcpy(myData.text, payload + 2, copy_len); // Copy everything after the '|'

      if (targetRobot == 1) esp_now_send(mac_robot1, (uint8_t *)&myData, sizeof(myData));
      else if (targetRobot == 2) esp_now_send(mac_robot2, (uint8_t *)&myData, sizeof(myData));
    }
  }
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len)
{
  if (len == sizeof(TelemetryData))
  {
    int robot_id = 0;
    if (memcmp(info->src_addr, mac_robot1, 6) == 0) robot_id = 1;
    else if (memcmp(info->src_addr, mac_robot2, 6) == 0) robot_id = 2;

    if (robot_id > 0) {
      // Safely copy data into the respective robot's buffer (Index 0 for R1, Index 1 for R2)
      memcpy(&holdingBuffer[robot_id - 1], incomingData, sizeof(TelemetryData));
      newDataReady[robot_id - 1] = true;
    }
  }
}

void setup()
{
  if (!SPIFFS.begin(true)) return;

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false); //to prevent disconnection
  WiFi.softAP(ssid, password, 1); 

  server.on("/", []() {
    File file = SPIFFS.open("/index.html", "r");
    if (!file) { server.send(500, "text/plain", "SPIFFS error"); return; }
    server.streamFile(file, "text/html");
    file.close(); 
  });

  server.on("/cmd", HTTP_GET, []() {
    if (!server.hasArg("c") && !server.hasArg("move")) {
      server.send(400, "text/plain", "Missing c or move query parameter");
      return;
    }

    String robot = server.hasArg("robot") ? server.arg("robot") : "1";
    String command = server.hasArg("c") ? server.arg("c") : "";
    String move = server.hasArg("move") ? server.arg("move") : "";
    float speed = server.hasArg("speed") ? server.arg("speed").toFloat() : 4.0;
    float turnSpeed = server.hasArg("turn") ? server.arg("turn").toFloat() : DEFAULT_TURN_SPEED;

    if (robot == "1") {
      if (server.hasArg("move")) {
        if (!sendMoveCommand(mac_robot1, move, speed, turnSpeed)) {
          server.send(400, "text/plain", "Invalid move query parameter");
          return;
        }
      } else {
        sendEspNowCommand(mac_robot1, command);
      }
    } else if (robot == "2") {
      if (server.hasArg("move")) {
        if (!sendMoveCommand(mac_robot2, move, speed, turnSpeed)) {
          server.send(400, "text/plain", "Invalid move query parameter");
          return;
        }
      } else {
        sendEspNowCommand(mac_robot2, command);
      }
    } else if (robot == "all") {
      if (server.hasArg("move")) {
        if (!sendMoveCommand(mac_robot1, move, speed, turnSpeed) || !sendMoveCommand(mac_robot2, move, speed, turnSpeed)) {
          server.send(400, "text/plain", "Invalid move query parameter");
          return;
        }
      } else {
        sendEspNowCommand(mac_robot1, command);
        sendEspNowCommand(mac_robot2, command);
      }
    } else {
      server.send(400, "text/plain", "Invalid robot query parameter");
      return;
    }

    server.send(200, "text/plain", "OK");
  });

  server.serveStatic("/", SPIFFS, "/");
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  esp_now_init();
  peerInfo.channel = 1; //this channel must match that of the cortex MCU
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_AP; 

  // Register Robot 1
  memcpy(peerInfo.peer_addr, mac_robot1, 6);
  esp_now_add_peer(&peerInfo);

  // Register Robot 2
  memcpy(peerInfo.peer_addr, mac_robot2, 6);
  esp_now_add_peer(&peerInfo);

  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  webSocket.loop();
  server.handleClient();

  // Check both buffers and broadcast whichever one has fresh data
  for (int i = 0; i < 2; i++) 
  {
    if (newDataReady[i])
    {
      TelemetryData safeBuffer;
      noInterrupts(); 
      memcpy(&safeBuffer, (const void*)&holdingBuffer[i], sizeof(TelemetryData));
      newDataReady[i] = false;
      interrupts(); 

      char msg[200];
      snprintf(msg, sizeof(msg), "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
               i + 1, safeBuffer.ang, safeBuffer.targ, safeBuffer.err,
               safeBuffer.p_term, safeBuffer.i_term, safeBuffer.d_term,
               safeBuffer.spd_in, safeBuffer.pos_err,
               safeBuffer.mot_l, safeBuffer.mot_r, safeBuffer.bat,
               safeBuffer.kp_a, safeBuffer.kd_a, safeBuffer.kp_s, safeBuffer.ki_s, safeBuffer.kp_y, safeBuffer.base_ang);

      webSocket.broadcastTXT(msg);
      yield(); // <-Let the ESP32 breathe and process incoming WiFi packets!
    }
  }
  yield(); // <-- Prevent Watchdog crashes
}