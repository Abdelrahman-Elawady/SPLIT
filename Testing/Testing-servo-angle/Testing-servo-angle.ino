#include <Servo.h>

Servo myServo;

// Pin 9 is a standard PWM pin commonly used for servos on Arduino
const int servoPin = 9; 

void setup() {
  // 115200 baud rate (make sure your Serial Monitor is set to this)
  Serial.begin(115200); 
  Serial.setTimeout(50);

  // Write the 0 position BEFORE attaching the servo to prevent jumping
  myServo.write(0);
  
  // Attach the servo. The MG995 usually needs a pulse width 
  // between 500us and 2400us for a full 180 degree sweep.
  myServo.attach(servoPin, 500, 2400);

  Serial.println("Arduino MG995 Calibration Tool");
  Serial.println("Type 0, 60, 90, 170, or 180 and press Enter:");
}

void loop() {
  if (Serial.available() > 0) {
    int targetAngle = Serial.parseInt();

    // Clear any remaining characters in the buffer (like newline or carriage return)
    while(Serial.available() > 0) {
      Serial.read();
    }

    // Check against the allowed test angles
    if (targetAngle == 0 || targetAngle == 60 || targetAngle == 90 || targetAngle == 170 || targetAngle == 180) {
      myServo.write(targetAngle);
      Serial.print("Moving to ");
      Serial.println(targetAngle);
    } else {
      Serial.println("Invalid input. Please send exactly 0, 60, 90, 170, or 180.");
    }
  }
}