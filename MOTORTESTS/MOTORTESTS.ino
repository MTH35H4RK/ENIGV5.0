
const int MOTOR_A_ENA = 4; // Right motor speed control
const int MOTOR_A_IN1 = 5;  // Right motor direction control 1
const int MOTOR_A_IN2 = 6;  // Right motor direction control 2
const int MOTOR_B_ENB = 9;  // Left motor speed control
const int MOTOR_B_IN3 = 10;  // Left motor direction control 1
const int MOTOR_B_IN4 = 11;  // Left motor direction control 2

// Forward movement function
void forward(int speed) {
  analogWrite(MOTOR_A_ENA, speed);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_B_ENB, speed);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
}

// Function to set motor speeds with differential for turning
void setMotors(int speedA, int speedB) {
  if (speedA < 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    speedA = -speedA; // Make speed a positive quantity
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  }
  analogWrite(MOTOR_A_ENA, speedA); // Set right motor speed

  if (speedB < 0) {
    digitalWrite(MOTOR_B_IN3, HIGH);
    digitalWrite(MOTOR_B_IN4, LOW);
    speedB = -speedB; // Make speed a positive quantity
  } else {
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, HIGH);
  }
  analogWrite(MOTOR_B_ENB, speedB); // Set left motor speed
}

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_A_ENA, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_ENB, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);

}

void loop() {
 forward(255);
}
