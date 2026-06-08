#include <Arduino.h>     // for basic Arduino functions and types on platformio
#include <WiFi.h>        // for disabling WiFi on the physics chip and freeing up pins for I2C
#include <MPU6050.h>     // IMU sensor library installed from https://github.com/jrowberg/i2cdevlib & also available as a PlatformIO library
#include <Wire.h>        // for I2C communication with MPU6050
#include <Preferences.h> // The modern ESP32 replacement for EEPROM

Preferences preferences; // Create a Preferences object to handle non-volatile storage of PID parameters

const int BATTERY_PIN = 36; // ADC pin for battery voltage monitoring
int robot_state = 0;        // Hardware Safety Switch: 0 = Fallen, 1 = Balancing

MPU6050 mpu;                    // Create an MPU6050 object for interfacing with the IMU sensor
int16_t ax, ay, az, gx, gy, gz; // Variables to hold raw sensor data from the MPU6050
// here we chose to not use the dmp features of the MPU6050, as they can introduce additional latency and complexity that may not be ideal for a high-speed balancing robot.
// Instead, we will read the raw accelerometer and gyroscope data directly and implement our own sensor fusion algorithm (like a Kalman filter) to estimate the robot's angle
// and angular velocity. This approach allows for more control over the data processing and can be optimized for our specific use case & the use of the much powerful esp32 controller.

//  ================= HARDWARE PINS =================
// Motor Driver Pins
const int ENA = 25;
const int IN1 = 27;
const int IN2 = 14;
const int IN3 = 18;
const int IN4 = 13;
const int ENB = 26;
const int freq = 5000;    // PWM frequency for motor control - 5k to 10k is good
const int resolution = 8; // PWM resolution (8 bits = 0-255)

// Encoder Pins
const int ENC_L_A = 34;
const int ENC_L_B = 35;
const int ENC_R_A = 32;
const int ENC_R_B = 33;
// Volatile variables to hold encoder counts, updated in the ISR (Interrupt Service Routine) for accurate speed measurement without blocking the main loop
// Using interrupts allows us to capture every encoder pulse even at high speeds, which is crucial for precise control of the balancing robot.
volatile long count_left = 0;  
volatile long count_right = 0; 

void IRAM_ATTR isr_left() // Interrupt Service Routine for the left encoder, marked with IRAM_ATTR to ensure it runs from IRAM for faster execution
{ // This logic determines the direction of rotation based on the state of the encoder channels. 
  //If both channels are the same, we consider it as forward rotation and increment the count; otherwise, it's reverse rotation and we decrement the count.
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) 
    count_left++;
  else
    count_left--;
}
void IRAM_ATTR isr_right()
{
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B))
    count_right--;
  else
    count_right++;
}

// ================= TIMING & KALMAN FILTER =================
unsigned long last_time = 0; // Variable to keep track of the last time we read the sensors and updated the control loop, used for calculating delta time (dt) for the Kalman filter and PID calculations
unsigned long last_speed_time = 0; // Variable to track the last time we calculated the speed from the encoders, allowing us to run the speed control loop at a fixed interval (e.g., every 100 ms) independent of the main loop frequency
unsigned long last_telem_time = 0; // Variable to track the last time we sent telemetry data, allowing us to send updates at a fixed interval (e.g., every 100 ms) without overwhelming the communication channel
double dt = 0.005; // Initial guess for delta time (dt) between sensor readings, will be updated in the loop based on actual timing to ensure accurate calculations in the Kalman filter and PID controller, especially important if the loop execution time varies due to processing load or other factors.
unsigned long last_cmd_time = 0; // Tracks the last time a command arrived

float angle = 0, angle_speed = 0, q_bias = 0; // Variables for the Kalman filter: 'angle' is the estimated angle of the robot, 'angle_speed' is the estimated angular velocity, and 'q_bias' is the estimated bias in the gyroscope readings. The Kalman filter will
float P[2][2] = {{1, 0}, {0, 1}}; // Error covariance matrix for the Kalman filter, initialized to identity matrix. This matrix will be updated in the Kalman filter algorithm to reflect the uncertainty in our angle and bias estimates, allowing the filter to optimally combine the accelerometer and gyroscope data for accurate angle estimation.
float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.5; // Kalman filter tuning parameters: Q_angle is the process noise variance for the angle, Q_gyro is the process noise variance for the gyroscope bias, and R_angle is the measurement noise variance for the angle. 
//These values can be adjusted to improve the performance of the Kalman filter based on the characteristics of the sensors and the dynamics of the robot.

// ================= PID VARIABLES =================
float Kp_ang, Kd_ang, Ki_ang = 0.0; // inner (angle) loop PID variables, running at 200 hz.
float angle_target = 0.0, angle_integral = 0.0; // 'angle_target' is the desired angle we want the robot to maintain (usually around 0 degrees for upright balancing), and 'angle_integral' is the accumulated integral of the angle error over time, used in the integral term of the PID controller to help correct for any steady-state error in the angle control loop.
float base_angle = 0.0; // 'base_angle' is an adjustable offset that can be used to set the robot's neutral position.
// float joy_fwd = 0.0; // 'joy_fwd' is a variable that can be used to add a forward or backward bias to the angle target based on joystick input.
float joy_target = 0.0;
float joy_current = 0.0;
float slew_rate = 0.02; // How fast it accelerates. Smaller = smoother.

float Kp_spd, Ki_spd; //middle (sped) loop PID variables, running at 10 hz.
float speed_filter = 0, speed_filter_old = 0, position_error = 0;
float speed_output = 0.0;

float Kp_yaw, Kd_yaw = 0.0, yaw_target = 0.0; // outer (yaw) loop running at the same time as the inner (angle) loop (200 hz), since it is reading instantaneous gyro for balancing.

// ================= TELEMETRY STRUCTURE =================
// Define a structure to hold all the telemetry data we want to send back to the campanion MCU that handles the wireless esp32-now communication with the remote controller.
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

// ================= DUAL-MODE UART PARSER =================
String UART_Buffer = ""; 
void parseUART() // This function reads incoming data from Serial2, which is connected to the companion MCU.
{
  while (Serial2.available())
  {
    char c = Serial2.read();
    if (c == '\n')
    {
      if (UART_Buffer.length() >= 2) // We expect at least 2 characters for a valid command (e.g., "P:1.5" or "EX:1.5")
      {
        char prefix = UART_Buffer[0]; 

        // --- MODE 1: SAVE TO FLASH MEMORY (EEPROM) ---
        if (prefix == 'E')
        {
          char param = UART_Buffer[1];                    // Get the actual parameter letter
          float val = UART_Buffer.substring(3).toFloat(); // Skip "EX:"

          if (param == 'P')
          {
            Kp_ang = val;
            preferences.putFloat("Kp_ang", val);
          }
          else if (param == 'D')
          {
            Kd_ang = val;
            preferences.putFloat("Kd_ang", val);
          }
          else if (param == 'V')
          {
            Kp_spd = val;
            preferences.putFloat("Kp_spd", val);
          }
          else if (param == 'U')
          {
            Ki_spd = val;
            preferences.putFloat("Ki_spd", val);
          }
          else if (param == 'Y')
          {
            Kp_yaw = val;
            preferences.putFloat("Kp_yaw", val);
          }
          else if (param == 'O')
          {
            base_angle = val;
            preferences.putFloat("base_angle", val);
          }
        }
        // --- MODE 2: TEMPORARY RAM TUNING & JOYSTICK ---
        else
        {
          float val = UART_Buffer.substring(2).toFloat(); // Skip "X:"

          if (prefix == 'P')
            Kp_ang = val;
          else if (prefix == 'D')
            Kd_ang = val;
          else if (prefix == 'V')
            Kp_spd = val;
          else if (prefix == 'U')
            Ki_spd = val;
          else if (prefix == 'Y')
            Kp_yaw = val;
          else if (prefix == 'B') 
            {
            joy_target = val; // Store the requested speed here, do not apply it directly
            last_cmd_time = millis(); // Reset the safety timer!
            }
          else if (prefix == 'O')
            base_angle = val;
          else if (prefix == 'T') 
            {
            yaw_target = val;
          last_cmd_time = millis(); // Reset the safety timer!
            }
        }
      }
      UART_Buffer = "";
    }
    else
    {
      UART_Buffer += c;
    }
  }
}

float readBatteryVoltage() 
{
  // Read the raw ADC value from the battery voltage divider.
  int raw_adc = analogRead(BATTERY_PIN);
  float pin_voltage = (raw_adc / 4095.0) * 3.3;
  return pin_voltage * 6.01; //(1970+ 9870) / 1970 voltage divider ratio
}

// --- Function Prototypes ---
void driveMotors(float speed_left, float speed_right);
void updateKalman(float angle_m, float gyro_m);

void setup()
{
  UART_Buffer.reserve(64); // Pre-allocate 64 bytes of RAM for the UART string
  //Serial.begin(115200);
  WiFi.mode(WIFI_OFF); // Disable radio on the physics chip 
  // This frees up the pins used for WiFi (GPIO 21 and 22) for I2C communication with the MPU6050 IMU sensor, and also reduces power consumption and potential interference from the WiFi radio, which is beneficial for a real-time control application like a balancing robot.
  Serial2.begin(500000, SERIAL_8N1, 16, 17); 


  // Initialize Preferences and load saved values (or defaults if first time)
  preferences.begin("pid_tune", false); // "pid_tune" is the namespace for our stored preferences, and 'false' means we want read/write access (not read-only).
  Kp_ang = preferences.getFloat("Kp_ang", 0.0);
  Kd_ang = preferences.getFloat("Kd_ang", 0.0);
  Kp_spd = preferences.getFloat("Kp_spd", 0.0);
  Ki_spd = preferences.getFloat("Ki_spd", 0.0);
  Kp_yaw = preferences.getFloat("Kp_yaw", 0.0);
  base_angle = preferences.getFloat("base_angle", 0.0);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PlatformIO Version 2 LEDC Setup
  // ledcSetup(0, freq, resolution); 
  // ledcSetup(1, freq, resolution);
  // ledcAttachPin(ENA, 0);
  // ledcAttachPin(ENB, 1);

  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_right, RISING);
  pinMode(BATTERY_PIN, INPUT);

  Wire.begin(21, 22); 
  Wire.setClock(400000); //set I2C clock to 400 khz.
  Wire.setTimeOut(10); //prevent I2C bus lockup by setting a timeout for I2C transactions. If a transaction takes longer than 10 ms, it will be aborted and the bus will be released, allowing the robot to recover from potential communication issues with the MPU6050 sensor without freezing the entire system.
  mpu.initialize(); // Initialize the MPU6050 sensor

  // Manually set the accelerometer and gyroscope offsets via calibration script.
  //6329	4760	9055	-219	-209	13
  mpu.setXAccelOffset(6329); 
  mpu.setYAccelOffset(4760);
  mpu.setZAccelOffset(9055);
  mpu.setXGyroOffset(-219);
  mpu.setYGyroOffset(-209);
  mpu.setZGyroOffset(13);

  last_time = micros(); 
  last_speed_time = millis(); 
}

void loop()
{
  parseUART(); 
  
  unsigned long current_time = micros();
  dt = (current_time - last_time) / 1000000.0; //we convert microseconds to seconds for the dt variable used in the Kalman filter and PID calculations. 
  last_time = current_time;
  if (dt > 0.1) 
    dt = 0.005; 

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // In the real world, gravity ensures these 6 axes will NEVER all be exactly zero at the same time.
  // If they are all 0, the I2C wire is disconnected or the MPU6050 has crashed.
  if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
    robot_state = 0;      // Immediately kill the physics loop
    driveMotors(0, 0);    // Hard stop the motors
    return;               // Skip the rest of the loop until the sensor comes back online
  }
  float angle_m = atan2(ax, az) * (180 / PI); 
  float gyro_y = -gy / 131.0; // -ve sign to align gyro direction with angle direction, and divide by sensitivity scale factor (131 LSB/°/s for the default ±250 °/s range) to convert raw gyro reading to degrees per second.
  updateKalman(angle_m, gyro_y);

  //diagnosis:
  // Serial.print("Angle: "); 
  // Serial.println(angle);

  if (robot_state == 0) 
  {
    driveMotors(0, 0);
    angle_integral = 0; // prevent windup
    position_error = 0;
    count_left = 0;
    count_right = 0;
    speed_filter = 0;
    speed_filter_old = 0;

    if (angle > -20 && angle < 20) // If the robot is within a reasonable angle range (not too far fallen), we can allow it to start balancing again when it detects movement. This helps prevent the robot from trying to balance when it's completely tipped over, which could lead to erratic behavior or damage.    
    {
      robot_state = 1;
      last_speed_time = millis();
    }
  }
  else if (robot_state == 1)
  {
    if (angle > 45 || angle < -45) // protection when fallen
    {
      robot_state = 0;
      return;
    }

    // ==========================================
    // THE DEADMAN'S SWITCH (500ms Timeout)
    // ==========================================
    if (millis() - last_cmd_time > 500) 
    {
      joy_target = 0.0;
      yaw_target = 0.0;
    }

    // ==========================================
    // 10 HZ SPEED LOOP (Runs every 100ms)
    // ==========================================
    if (millis() - last_speed_time >= 100) 
    {
      long left_pulse = count_left; 
      long right_pulse = count_right;
      count_left = 0;
      count_right = 0;

      float current_speed = (left_pulse + right_pulse) / 2.0; 
      speed_filter = (speed_filter_old * 0.4) + (current_speed * 0.6); 
      speed_filter_old = speed_filter;

      // USE joy_target INSTEAD OF joy_fwd
      if (joy_target == 0.0 && yaw_target == 0.0)
        position_error += speed_filter;
      else
        position_error = 0;
      
      if (position_error > 300) position_error = 300;
      if (position_error < -300) position_error = -300;

      speed_output = (Kp_spd * speed_filter) + (Ki_spd * position_error); 
      if (speed_output > 15) speed_output = 15;
      else if (speed_output < -15) speed_output = -15;

      last_speed_time = millis();
    } // <-- END OF 10HZ LOOP

    // ==========================================
    // 200 HZ FAST LOOP (Runs every 5ms)
    // ==========================================
    
    // 1. The Trajectory Smoother (Executes rapidly for butter-smooth acceleration)
    if (joy_current < joy_target) {
      joy_current += slew_rate;
      if (joy_current > joy_target) joy_current = joy_target;
    } else if (joy_current > joy_target) {
      joy_current -= slew_rate;
      if (joy_current < joy_target) joy_current = joy_target;
    }

    // 2. Combine the targets
    angle_target = base_angle - speed_output - joy_current;

    // 3. Standard PID Math
    float current_error = angle - angle_target;
    //angle_integral += current_error * dt;

    // Only accumulate the integral if we are NOT forcefully driving via joystick
    if (joy_target == 0.0) {
      angle_integral += current_error * dt;
    } else {
      // Bleed off the accumulated integral quickly so it doesn't fight the joystick
      angle_integral *= 0.8; 
    }
    if (angle_integral > 300) angle_integral = 300;
    if (angle_integral < -300) angle_integral = -300;

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

    if (millis() - last_telem_time > 100)
    {
      telem.ang = angle;
      telem.targ = angle_target;
      telem.err = current_error;
      telem.p_term = p_out;
      telem.i_term = (Ki_spd * position_error); //i_out;
      telem.d_term = d_out;
      telem.spd_in = speed_filter;
      telem.pos_err = position_error;
      telem.mot_l = left_motor_final;
      telem.mot_r = right_motor_final;
      telem.bat = readBatteryVoltage();

      // Attach the active PID values to the payload
      telem.kp_a = Kp_ang;
      telem.kd_a = Kd_ang;
      telem.kp_s = Kp_spd;
      telem.ki_s = Ki_spd;
      telem.kp_y = Kp_yaw;
      telem.base_ang = base_angle;
      
      Serial2.write(0xAA);
      Serial2.write(0xAA);

      Serial2.write((uint8_t *)&telem, sizeof(TelemetryData));
      last_telem_time = millis();
    }
  }
}

void driveMotors(float speed_left, float speed_right)
{ 
  if (speed_left > 255)
    speed_left = 255;
  else if (speed_left < -255)
    speed_left = -255;
  if (speed_right > 255)
    speed_right = 255;
  else if (speed_right < -255)
    speed_right = -255;
  // We set a minimum speed threshold to ensure the motors receive enough power to overcome static friction and start moving. ((deadband))
  if (abs(speed_left) < 45 && abs(speed_left) > 0)
    speed_left = (speed_left > 0) ? 45 : -45;
  if (abs(speed_right) < 45 && abs(speed_right) > 0)
    speed_right = (speed_right > 0) ? 45 : -45;

  if (speed_left > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (speed_left < 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  if (speed_right > 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if (speed_right < 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // PlatformIO Version 2 PWM execution
  // ledcWrite(0, abs(speed_left));
  // ledcWrite(1, abs(speed_right));

  ledcWrite(ENA, abs(speed_left));
  ledcWrite(ENB, abs(speed_right));
}

void updateKalman(float angle_m, float gyro_m)
{
  // just a bunch of math :)
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