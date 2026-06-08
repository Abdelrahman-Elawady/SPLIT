#include <esp_now.h>
#include <WiFi.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

// ================= L298N motor driver pins =================
const int ENA = 25; 
const int IN1 = 27;
const int IN2 = 14;
const int IN3 = 18;
const int IN4 = 13;
const int ENB = 26;

const int freq = 5000; //switching frequency for the L298N 
const int resolution = 8; 

// ================= ENCODER PINS =================
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
unsigned long last_speed_time = 0;
double dt = 0.005; 

// ================= KALMAN FILTER =================
float angle = 0, angle_speed = 0, q_bias = 0;
float P[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.5; 

// ================= INNER LOOP (ANGLE PID) =================

float Kp_ang = 200.0; 
float Kd_ang = 4.0;
float Ki_ang = 0.0;
float angle_target = 0.0; 
float angle_integral = 0.0;
float base_angle = 0.0; // The mechanical zero

// ================= OUTER LOOP (SPEED PID) =================
float Kp_spd = 0.2; // Start at 0 for tuning
float Ki_spd = 0.01; 
float speed_filter = 0, speed_filter_old = 0;
float position_error = 0;

// ================= 3rd OUTER LOOP (Yaw PID) =================
float Kp_yaw = 2.4; 
float Kd_yaw = 0.0;
float yaw_target = 0.0; // 0 means "fight all twisting to stay perfectly straight"

// ================= ESP-NOW RECEIVER =================
typedef struct struct_message {
  char text[32];
} struct_message;
struct_message myData;

void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  if (myData.text[0] == 'P' && myData.text[1] == ':') {
    Kp_ang = atof(myData.text + 2);
    Serial.print(F("Angle Kp: ")); Serial.println(Kp_ang);
  } 
  else if (myData.text[0] == 'D' && myData.text[1] == ':') {
    Kd_ang = atof(myData.text + 2);
    Serial.print(F("Angle Kd: ")); Serial.println(Kd_ang);
  }
  else if (myData.text[0] == 'B' && myData.text[1] == ':') {
    base_angle = atof(myData.text + 2);
    Serial.print(F("Base Angle: ")); Serial.println(base_angle);
  }
  else if (myData.text[0] == 'V' && myData.text[1] == ':') {
    Kp_spd = atof(myData.text + 2);
    Serial.print(F("Speed Kp: ")); Serial.println(Kp_spd);
  }
  else if (myData.text[0] == 'U' && myData.text[1] == ':') {
    Ki_spd = atof(myData.text + 2);
    Serial.print(F("Speed Ki: ")); Serial.println(Ki_spd);
  }
  else if (myData.text[0] == 'Y' && myData.text[1] == ':') {
    Kp_yaw = atof(myData.text + 2);
    Serial.print(F("Yaw Kp: ")); Serial.println(Kp_yaw);
  }
  else if (myData.text[0] == 'T' && myData.text[1] == ':') {
    yaw_target = atof(myData.text + 2);
    Serial.print(F("Yaw Target: ")); Serial.println(yaw_target);
  }
}

void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
  }

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);

  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_right, RISING);

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
  updateKalman(angle_m, gyro_y);

  // 3. SPEED & POSITION LOOP (Runs at 20Hz)
  if (millis() - last_speed_time >= 100) {
    long left_pulse = count_left;
    long right_pulse = count_right;
    count_left = 0; count_right = 0; 
    
    // Calculate forward speed
    float current_speed = -(left_pulse + right_pulse) / 2.0;
    
    // Low-Pass Filter to smooth out encoder jitter
    speed_filter = (speed_filter_old * 0.4) + (current_speed * 0.6);
    speed_filter_old = speed_filter;
    
    // Accumulate position error ONLY if we are standing still
    if (base_angle == 0.0 && yaw_target == 0.0) {
      position_error += speed_filter;
    } else {
      position_error = 0; // Cut the rubber band while driving
    }
    
    // Anti-Windup (Stop the math from spiraling out of control if picked up)
    if(position_error > 300) position_error = 300;
    if(position_error < -300) position_error = -300;

    // Outer Loop Output
    float speed_output = (Kp_spd * speed_filter) + (Ki_spd * position_error);
    
    // Limit how far the robot is allowed to lean to stop itself
    if(speed_output > 15) speed_output = 15;
    if(speed_output < -15) speed_output = -15;

    // Shift the target angle
    angle_target = base_angle - speed_output;
    
    last_speed_time = millis();
  }

  // 4. ANGLE LOOP
  float current_error = angle - angle_target;
  
  angle_integral += current_error * dt;
  if (angle_integral > 300) angle_integral = 300;
  if (angle_integral < -300) angle_integral = -300;

  float motor_out = (Kp_ang * current_error) + (Ki_ang * angle_integral) + (Kd_ang * angle_speed);

// 1. Convert raw gz to degrees per second
  float gyro_z = gz / 131.0; 

  // 2. Calculate Yaw Error using the scaled value
  float yaw_error = yaw_target - gyro_z; 
  float yaw_out = (Kp_yaw * yaw_error);

  // 3. Differential Motor Mixing
  float left_motor_final = motor_out - yaw_out;
  float right_motor_final = motor_out + yaw_out;

  // Change your driveMotors function to accept TWO inputs now:
  driveMotors(left_motor_final, right_motor_final);

}

// differential drive
void driveMotors(float speed_left, float speed_right) {
  // 1. Crash Protection - cut power if tilted past 40 degrees
  if (angle > 40 || angle < -40) {
    speed_left = 0;
    speed_right = 0;
    angle_integral = 0;
    position_error = 0; // Wipe memory if it crashes
  }

  // 2. Constrain Maximums (8-bit resolution)
  if (speed_left > 255) speed_left = 255;
  if (speed_left < -255) speed_left = -255;
  if (speed_right > 255) speed_right = 255;
  if (speed_right < -255) speed_right = -255;

  // 3. L298N Voltage Deadband Punch-Through
  if (abs(speed_left) < 45 && abs(speed_left) > 0) speed_left = (speed_left > 0) ? 45 : -45;
  if (abs(speed_right) < 45 && abs(speed_right) > 0) speed_right = (speed_right > 0) ? 45 : -45;

  // 4. Drive Left Motor (IN1 / IN2)
  if (speed_left > 0) { 
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    
  } else if (speed_left < 0) { 
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  } else { 
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }

  // 5. Drive Right Motor (IN3 / IN4)
  if (speed_right > 0) { 
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (speed_right < 0) { 
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else { 
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }

  // 6. Send the final PWM pulses
  ledcWrite(ENA, abs(speed_left));
  ledcWrite(ENB, abs(speed_right));
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