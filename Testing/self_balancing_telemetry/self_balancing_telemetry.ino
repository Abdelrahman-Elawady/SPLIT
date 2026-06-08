#include <esp_now.h>
#include <WiFi.h>
#include <MPU6050.h>
#include <Wire.h>

const int BATTERY_PIN = 36; // to get battery status from voltage divider

int robot_state = 0; // 0 = Fallen, 1 = Balancing

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

// ================= L298N motor driver pins =================
const int ENA = 25; const int IN1 = 27; const int IN2 = 14;
const int IN3 = 12; const int IN4 = 13; const int ENB = 26;

const int freq = 5000; //switching frequency for the L298N 
const int resolution = 8; 

// ================= ENCODER PINS =================
const int ENC_L_A = 34; const int ENC_L_B = 35;
const int ENC_R_A = 32; const int ENC_R_B = 33;
volatile long count_left = 0; volatile long count_right = 0;

void IRAM_ATTR isr_left() { 
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) count_left++; else count_left--; }

void IRAM_ATTR isr_right() { 
  // Flipped logic to match physical symmetry of the encoder and motor position -- ++
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) count_right--; else count_right++; }

// ================= ADAPTIVE TIMING & PID =================
unsigned long last_time = 0;
unsigned long last_speed_time = 0;
double dt = 0.005; 

// ================= KALMAN FILTER =================
float angle = 0, angle_speed = 0, q_bias = 0;
float P[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.5; 

// ================= INNER LOOP (ANGLE PID) =================
float Kp_ang = 200.0; // first value to be tuned until aggresive swinging
float Kd_ang = 4.0; // damp the oscillation by predicting future value
float Ki_ang = 0.0; // not used here but written for standards
float angle_target = 0.0; 
float angle_integral = 0.0;
float base_angle = 0.0; // The mechanical zero

// ================= MIDDLE LOOP (SPEED PID) =================
float Kp_spd = 0.2; // Start at 0 for tuning
float Ki_spd = 0.01; 
float speed_filter = 0, speed_filter_old = 0;
float position_error = 0;

// ================= OUTER LOOP (Yaw/ROTATION PID) =================
float Kp_yaw = 2.4; 
float Kd_yaw = 0.0;
float yaw_target = 0.0; // 0 means "fight all twisting to stay perfectly straight"
//The multiplier for cross-axis compensation
float cross_axis_tune = 0.01;
// ================= ESP-NOW RECEIVER/TELEMETRY & COMMS =================
unsigned long last_telem_time = 0;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

typedef struct {
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
} TelemetryData;
TelemetryData telem;

typedef struct {
  char text[32];
} CommandMsg;
CommandMsg myData;

void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  if (myData.text[0] == 'P') Kp_ang = atof(myData.text + 2); // adjust p angle gain
  else if (myData.text[0] == 'D') Kd_ang = atof(myData.text + 2); //adjust d angle gain
  else if (myData.text[0] == 'B') base_angle = atof(myData.text + 2); // adjust base angle
  else if (myData.text[0] == 'V') Kp_spd = atof(myData.text + 2); // adjust p speed gain
  else if (myData.text[0] == 'U') Ki_spd = atof(myData.text + 2); //adjust i speed gain
  else if (myData.text[0] == 'Y') Kp_yaw = atof(myData.text + 2); //adjust p yaw gain
  else if (myData.text[0] == 'T') yaw_target = atof(myData.text + 2); //adjust yaw target
  else if (myData.text[0] == 'C') cross_axis_tune = atof(myData.text + 2);
}

float readBatteryVoltage() {
  int raw_adc = analogRead(BATTERY_PIN);
  float pin_voltage = (raw_adc / 4095.0) * 3.3; // 12 bit ADC and pin voltage of esp32
  return pin_voltage * 6.457; // voltage divider (1970+9780)/1970 resistance values
}

//MAIN SETUP FUNCTION

void setup() {
  Serial.begin(115200);

  // 3. Network Setup
  WiFi.mode(WIFI_MODE_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv); 

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1; peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // 4. L298N Setup (This will now safely use Timer 1 without destroying the servo)
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);

  // 5. Encoders & Battery
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_right, RISING);
  pinMode(BATTERY_PIN, INPUT);

  // 6. MPU6050 Setup
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
  last_speed_time = millis();
}

//MAIN LOOP FUNCTION

void loop() {
  // 1. ADAPTIVE TIME
  unsigned long current_time = micros();
  dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;
  if (dt > 0.1) dt = 0.005;

  // 2. SENSOR READING
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float angle_m = atan2(ax, az) * (180 / PI); 
  float gyro_y = -gy / 131.0; 
  // --- THE SPIN FIX (Cross-Axis Compensation) ---
  // Calculate the raw degrees per second of the yaw spin
  float gyro_z_deg = gz / 131.0; 
  
  // Subtract a tiny fraction of that spin from the pitch gyroscope
  // to cancel out the physical tilt of the silicon chip
  gyro_y -= (gyro_z_deg * cross_axis_tune); 
  // ----------------------------------------------
  updateKalman(angle_m, gyro_y);
  
  if (robot_state == 0) {
    driveMotors(0, 0);
    angle_integral = 0; position_error = 0;
    count_left = 0; count_right = 0;
    speed_filter = 0; speed_filter_old = 0;
    
    // Auto-Wake when placed upright
    if (angle > -20 && angle < 20) {
      robot_state = 1;
      last_speed_time = millis();
    }
  } 
  else if (robot_state == 1) {
    if (angle > 45 || angle < -45) {
      robot_state = 0; 
      return; 
    }

    if (millis() - last_speed_time >= 100) {
      long left_pulse = count_left; long right_pulse = count_right;
      count_left = 0; count_right = 0; 
      
      float current_speed = -(left_pulse + right_pulse) / 2.0;
      speed_filter = (speed_filter_old * 0.4) + (current_speed * 0.6);
      speed_filter_old = speed_filter;
      
      if (base_angle == 0.0 && yaw_target == 0.0) position_error += speed_filter;
      else position_error = 0; 
      
      if(position_error > 300) position_error = 300;
      if(position_error < -300) position_error = -300;

      float speed_output = (Kp_spd * speed_filter) + (Ki_spd * position_error);
      if(speed_output > 15) speed_output = 15; else if(speed_output < -15) speed_output = -15;

      angle_target = base_angle - speed_output;
      last_speed_time = millis();
    }

    float current_error = angle - angle_target;
    angle_integral += current_error * dt;
    if (angle_integral > 300) angle_integral = 300;
    if (angle_integral < -300) angle_integral = -300;

    // Isolate the terms for telemetry
    float p_out = Kp_ang * current_error;
    float i_out = Ki_ang * angle_integral;
    float d_out = Kd_ang * angle_speed;
    float motor_out = p_out + i_out + d_out;

    float gyro_z = gz / 131.0; 
    float yaw_error = yaw_target - gyro_z; 
    float yaw_out = (Kp_yaw * yaw_error);

    float left_motor_final = motor_out - yaw_out;
    float right_motor_final = motor_out + yaw_out;

    driveMotors(left_motor_final, right_motor_final);

    // Cast Expanded Telemetry at 20Hz
    if (millis() - last_telem_time > 50) {
      telem.ang = angle;
      telem.targ = angle_target;
      telem.err = current_error;
      telem.p_term = p_out;
      telem.i_term = i_out;
      telem.d_term = d_out;
      telem.spd_in = speed_filter;
      telem.pos_err = position_error;
      telem.mot_l = left_motor_final;
      telem.mot_r = right_motor_final;
      telem.bat = readBatteryVoltage();
      
      esp_now_send(broadcastAddress, (uint8_t *) &telem, sizeof(TelemetryData));
      last_telem_time = millis();
    }
  }

}

// differential drive
void driveMotors(float speed_left, float speed_right) {
  if (speed_left > 255) speed_left = 255; else if (speed_left < -255) speed_left = -255;
  if (speed_right > 255) speed_right = 255; else if (speed_right < -255) speed_right = -255;

  if (abs(speed_left) < 45 && abs(speed_left) > 0) speed_left = (speed_left > 0) ? 45 : -45;
  if (abs(speed_right) < 45 && abs(speed_right) > 0) speed_right = (speed_right > 0) ? 45 : -45;

  if (speed_left > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); } 
  else if (speed_left < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); } 
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }

  if (speed_right > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); } 
  else if (speed_right < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); } 
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

  ledcWrite(ENA, abs(speed_left));
  ledcWrite(ENB, abs(speed_right));
}

void updateKalman(float angle_m, float gyro_m) {
  angle += (gyro_m - q_bias) * dt;
  float angle_err = angle_m - angle;
  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * dt; P[0][1] -= P[1][1] * dt;
  P[1][0] -= P[1][1] * dt; P[1][1] += Q_gyro * dt;
  float S = R_angle + P[0][0];
  float K_0 = P[0][0] / S; float K_1 = P[1][0] / S;
  P[0][0] -= K_0 * P[0][0]; P[0][1] -= K_0 * P[0][1];
  P[1][0] -= K_1 * P[0][0]; P[1][1] -= K_1 * P[0][1];
  q_bias += K_1 * angle_err; angle_speed = gyro_m - q_bias;
  angle += K_0 * angle_err;
}