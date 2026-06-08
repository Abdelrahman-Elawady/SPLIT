// Motor A pins
const int ENA = 25; 
const int IN1 = 27;
const int IN2 = 14;

// Motor B pins
const int ENB = 26;
const int IN3 = 18;
const int IN4 = 13;


void setup() {
  // Set all the motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // ----------------------------------------
  // Direction 1 (Forward)
  // ----------------------------------------
  
  // Set Motor A direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Set Motor B direction
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Turn ON both motors at full speed
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  
  // Keep them running for 3 seconds
  delay(3000); 

  // ----------------------------------------
  // Direction 2 (Backward)
  // ----------------------------------------
  
  // Reverse Motor A direction
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  // Reverse Motor B direction
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Keep them running in reverse for 3 seconds
  delay(3000); 
}