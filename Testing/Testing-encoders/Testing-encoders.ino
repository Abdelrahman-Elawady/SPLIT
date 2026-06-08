#include <Arduino.h>

// Motor Driver Pins
const int ENA = 25;
const int IN1 = 27;
const int IN2 = 14;
const int IN3 = 18;
const int IN4 = 13;
const int ENB = 26;

// Encoder Pins
const int ENC_L_A = 34;
const int ENC_L_B = 35;
const int ENC_R_A = 32;
const int ENC_R_B = 33;

volatile long count_left = 0;
volatile long count_right = 0;

void IRAM_ATTR isr_left() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) count_left++; else count_left--;
}

void IRAM_ATTR isr_right() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) count_right--; else count_right++;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, 5000, 8);
  ledcAttach(ENB, 5000, 8);

  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_right, RISING);

  Serial.println("STARTING MOTOR & ENCODER TEST...");
  delay(2000);
}

void loop() {
  // 1. Force both motors to drive FORWARD at a slow speed (100 out of 255)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 160);
  ledcWrite(ENB, 160);

  // 2. Print the encoder counts every 100ms
  Serial.print("Left Encoder: ");
  Serial.print(count_left);
  Serial.print("  |  Right Encoder: ");
  Serial.println(count_right);
  
  delay(100);
}