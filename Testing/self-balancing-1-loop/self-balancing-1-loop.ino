#include <esp_now.h>
#include <WiFi.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

// ================= L298N PINS =================
const int ENA = 25; 
const int IN1 = 27;
const int IN2 = 14;
const int IN3 = 12;
const int IN4 = 13;
const int ENB = 26;

const int freq = 5000;
const int resolution = 8; 

// ================= ENCODER PINS (Ready for Phase 4) =================
const int ENC_L_A = 34; const int ENC_L_B = 35;
const int ENC_R_A = 32; const int ENC_R_B = 33;
volatile long count_left = 0; volatile long count_right = 0;

void IRAM_ATTR isr_left() { 
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) count_left++; else count_left--; 
}
void IRAM_ATTR isr_right() { 
  // Flipped logic to match physical symmetry
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) count_right--; else count_right++; 
}

// ================= ADAPTIVE TIMING =================
unsigned long last_time = 0;
double dt = 0.005; 

// ================= KALMAN FILTER =================
float angle = 0, angle_speed = 0, q_bias = 0;
float P[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.5; 

// ================= PID VARIABLES =================
float Kp_ang = 90.0; // Start at 0 for safety!
float Kd_ang = 8.0;
float Ki_ang = 0.0;
float angle_target = 0.0; // The mechanical zero point
float angle_integral = 0.0;

// ================= ESP-NOW RECEIVER =================
typedef struct struct_message {
  char text[32];
} struct_message;
struct_message myData;

void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  // PARSE THE COMMANDS (e.g., "P:35.5", "D:0.8", "S:-2.5")
  if (myData.text[0] == 'P' && myData.text[1] == ':') {
    Kp_ang = atof(myData.text + 2);
    Serial.print(F("New Kp: ")); Serial.println(Kp_ang);
  } 
  else if (myData.text[0] == 'D' && myData.text[1] == ':') {
    Kd_ang = atof(myData.text + 2);
    Serial.print(F("New Kd: ")); Serial.println(Kd_ang);
  }
  else if (myData.text[0] == 'I' && myData.text[1] == ':') {
    Ki_ang = atof(myData.text + 2);
    Serial.print(F("New Ki: ")); Serial.println(Ki_ang);
  }
  else if (myData.text[0] == 'S' && myData.text[1] == ':') {
    angle_target = atof(myData.text + 2);
    Serial.print(F("New Setpoint: ")); Serial.println(angle_target);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Setup ESP-NOW
  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
  }

  // Setup Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);

  // Setup Encoders
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_right, RISING);

  // Setup MPU6050
// Setup MPU6050
  Wire.begin(21, 22); 
  Wire.setClock(400000); 
  mpu.initialize();
  
  // REPLACE THESE NUMBERS WITH YOUR CALIBRATION RESULTS
  mpu.setXAccelOffset(6230); 
  mpu.setYAccelOffset(4750); 
  mpu.setZAccelOffset(8938);
  mpu.setXGyroOffset(-223); 
  mpu.setYGyroOffset(-215); 
  mpu.setZGyroOffset(16);

  last_time = micros();
}

void loop() {
  // 1. ADAPTIVE TIME
  unsigned long current_time = micros();
  dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;
  if (dt > 0.1) dt = 0.005; // Prevent massive spikes

  // 2. SENSOR READING & KALMAN
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float angle_m = atan2(ax, az) * (180 / PI); 
  float gyro_y = -gy / 131.0; 
  updateKalman(angle_m, gyro_y);

  // 3. ANGLE PID
  float current_error = angle - angle_target;
  
  // Anti-Windup Integral
  angle_integral += current_error * dt;
  if (angle_integral > 300) angle_integral = 300;
  if (angle_integral < -300) angle_integral = -300;

  float motor_out = (Kp_ang * current_error) + (Ki_ang * angle_integral) + (Kd_ang * angle_speed);

  // 4. DRIVE MOTORS
  driveMotors(motor_out);
}

void driveMotors(float output) {
  // Crash Protection - cut power if tilted past 45 degrees
  if (angle > 45 || angle < -45) {
    output = 0;
    angle_integral = 0; // Wipe memory
  }

  int speed = abs(output);
  if (speed > 255) speed = 255;
  // L298N Voltage deadband punch-through
  if (speed < 30 && speed > 0) speed = 30; 

  if (output > 0) { // Forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (output < 0) { // Backward
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else { // Stop
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }

  ledcWrite(ENA, speed);
  ledcWrite(ENB, speed);
}

void updateKalman(float angle_m, float gyro_m) {
  angle += (gyro_m - q_bias) * dt;
  float angle_err = angle_m - angle;
  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * dt;
  P[0][1] -= P[1][1] * dt;
  P[1][0] -= P[1][1] * dt;
  P[1][1] += Q_gyro * dt;
  float S = R_angle + P[0][0];
  float K_0 = P[0][0] / S;
  float K_1 = P[1][0] / S;
  P[0][0] -= K_0 * P[0][0];
  P[0][1] -= K_0 * P[0][1];
  P[1][0] -= K_1 * P[0][0];
  P[1][1] -= K_1 * P[0][1];
  q_bias += K_1 * angle_err;
  angle_speed = gyro_m - q_bias;
  angle += K_0 * angle_err;
}