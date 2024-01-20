#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 13;
uint16_t sensorValues[SensorCount];

const int MOTOR_A_ENA = 4; // Right motor speed control
const int MOTOR_A_IN1 = 5;  // Right motor direction control 1
const int MOTOR_A_IN2 = 6;  // Right motor direction control 2
const int MOTOR_B_ENB = 9;  // Left motor speed control
const int MOTOR_B_IN3 = 10;  // Left motor direction control 1
const int MOTOR_B_IN4 = 11;  // Left motor direction control 2

// Motor speed constants
const int BASE_SPEED = 100; // base speed of the motors
const int MAX_SPEED = 255;  // max motor speed
float Kp = 0.66;             // proportional gain for P controller

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

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A8, A7, A9, A6, A10, A5, A11, A4, A12, A3, A13, A2, A14
  }, SensorCount);
  qtr.setEmitterPin(13);

  // Calibrate sensors during setup
  Serial.println("Starting calibration...");
  for (uint16_t i = 0; i < 400; i++) { // 400 * 25 ms = 10 seconds
    qtr.calibrate();
  }
  Serial.println("Calibration complete");
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Compute the error from the center
  int error = position - (SensorCount - 1) * 1000 / 2;

  // Calculate the motor speed difference using the P controller
  int speedDifference = error * Kp;

  // Calculate the speeds for each motor
  int speedA = BASE_SPEED + speedDifference;
  int speedB = BASE_SPEED - speedDifference;

  // Constrain the speeds to the [0, MAX_SPEED] interval
  speedA = constrain(speedA, 0, MAX_SPEED);
  speedB = constrain(speedB, 0, MAX_SPEED);

  // Set the motor speeds
  setMotors(speedA, speedB);
}
