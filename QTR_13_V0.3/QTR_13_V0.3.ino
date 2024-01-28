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
const int BASE_SPEED = 10; // base speed of the motors
const int MAX_SPEED = 80;  // max motor speed

// PID constants
float Kp = 0.3; // Proportional gain
float Ki = 0;  // Integral gain
float Kd = 0;  // Derivative gain

// Error variables
int lastError = 0;
int integral = 0;

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
  for (uint16_t i = 0; i < 200; i++) { // 400 * 25 ms = 10 seconds
    qtr.calibrate();
  }
  Serial.println("Calibration complete");
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Compute the error from the center with a bias towards the right
  // This will make the robot think it's more to the right, causing left turns
  int error = position - ((SensorCount-1)*1000/2 + 500); // Adding a constant bias

  // Rest of the PID calculations remains the same

  // Integral term calculation
  integral += error;

  // Prevent integral windup by constraining the integral term
  integral = constrain(integral, -10000, 10000);

  // Derivative term calculation
  int derivative = error - lastError;

  // Calculate the motor speed difference using the PID controller
  int speedDifference = Kp * error + Ki * integral + Kd * derivative;

  // Adjust the speeds for each motor to prefer left turn
  int speedA = BASE_SPEED + speedDifference; // Right motor
  int speedB = BASE_SPEED - speedDifference; // Left motor

  // Additional bias towards the left turn
  speedA = constrain(speedA + 10, 0, MAX_SPEED); // Increase right motor speed slightly
  speedB = constrain(speedB - 10, 0, MAX_SPEED); // Decrease left motor speed slightly

  // Set the motor speeds
  setMotors(speedA, speedB);

  // Remember the last error for the next loop iteration
  lastError = error;
}
