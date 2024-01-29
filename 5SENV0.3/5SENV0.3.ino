#include<Ultrasonic.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
unsigned long startTime;
float Kp =100; // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 0.15; // Derivative gain

float previous_error = 0;
float integral = 0;

int sensorPin1 = A15;
int sensorPin2 = A14;
int sensorPin3 = A13;
int sensorPin4 = A12;
int sensorPin5 = A11;

Ultrasonic leftus(23, 25, 7000UL);
const int MOTOR_A_ENA = 13; // Right motor speed control
const int MOTOR_A_IN1 = 12;  // Right motor direction control 1
const int MOTOR_A_IN2 = 11;  // Right motor direction control 2
const int MOTOR_B_ENB = 8;  // Left motor speed control
const int MOTOR_B_IN3 = 9;  // Left motor direction control 1
const int MOTOR_B_IN4 = 10;  // Left motor direction control 2
int distanceL;
// Define the threshold values for the sensors
int threshold = 500;

void move(int speed1, int speed2) {
  if (speed2 > 0) {
    analogWrite(MOTOR_A_ENA, speed2);
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  } else {
    analogWrite(MOTOR_A_ENA, -speed2);
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  }
  if (speed1 > 0) {
    analogWrite(MOTOR_B_ENB, speed1);
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, HIGH);
  } else {
    analogWrite(MOTOR_B_ENB, -speed1);
    digitalWrite(MOTOR_B_IN3, HIGH);
    digitalWrite(MOTOR_B_IN4, LOW);
  }
}


void right() {
  // Read the sensor values
  int sensorValue1 = analogRead(sensorPin1);
  int sensorValue2 = analogRead(sensorPin2);
  int sensorValue3 = analogRead(sensorPin3);
  int sensorValue4 = analogRead(sensorPin4);
  int sensorValue5 = analogRead(sensorPin5);
  //move()
  // Check if the robot is on the line
  if (sensorValue1 < threshold && sensorValue2 < threshold && sensorValue3 < threshold && sensorValue4 < threshold && sensorValue5 < threshold) {
    // Stop the robot
    move(80, 0);
  }
  if (sensorValue1 > threshold && sensorValue2 > threshold && sensorValue3 > threshold && sensorValue4 > threshold && sensorValue5 > threshold) {
    // Stop the robot
    move(90, -50); //turn left slightly
  }
  else {
    // Calculate the position of the robot on the line

    int position = 0;


    if (sensorValue3 < threshold) {
      position = 3;
    }
    if (sensorValue4 < threshold) {
      position = 4;
    }
    if (sensorValue5 < threshold) {
      position = 5;
    }
    if (sensorValue2 < threshold) {
      position = 2;
    }

    if (sensorValue1 < threshold) {
      position = 1;
    }

    float error = position - 3; // Assuming the center position is 3
    integral = integral + error;
    float derivative = error - previous_error;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    int baseSpeed = 60; // Base speed for both motors
    int leftMotorSpeed = baseSpeed + output;
    int rightMotorSpeed = baseSpeed - output;
    // Constrain the speeds to avoid negative values or values greater than max speed
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
    move(leftMotorSpeed, rightMotorSpeed);

  }
}
void left() {
  // Read the sensor values
  int sensorValue1 = analogRead(sensorPin1);
  int sensorValue2 = analogRead(sensorPin2);
  int sensorValue3 = analogRead(sensorPin3);
  int sensorValue4 = analogRead(sensorPin4);
  int sensorValue5 = analogRead(sensorPin5);
  // Check if the robot is on the line
  if (sensorValue1 < threshold && sensorValue2 < threshold && sensorValue3 < threshold && sensorValue4 < threshold && sensorValue5 < threshold) {
    // Stop the robot
    move(70, 0);
  }
  if (sensorValue1 > threshold && sensorValue2 > threshold && sensorValue3 > threshold && sensorValue4 > threshold && sensorValue5 > threshold) {
    move(70, 0);
    //    turn(255, 100); //turn left slightly
  }
  else {
    // Calculate the position of the robot on the line

    int position = 0;


    if (sensorValue3 < threshold) {
      position = 3;
    }
    if (sensorValue2 < threshold) {
      position = 2;
    }

    if (sensorValue1 < threshold) {
      position = 1;
    }
    if (sensorValue4 < threshold) {
      position = 4;
    }
    if (sensorValue5 < threshold) {
      position = 5;
    }

    float error = position - 3; // Assuming the center position is 3
    integral = integral + error;
    float derivative = error - previous_error;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    int baseSpeed = 60; // Base speed for both motors
    int leftMotorSpeed = baseSpeed + output;
    int rightMotorSpeed = baseSpeed - output;
    // Constrain the speeds to avoid negative values or values greater than max speed
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
    move(leftMotorSpeed, rightMotorSpeed);

  }
}
void printOnLcd(String c ) {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(c);

}
void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_A_ENA, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_ENB, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);


  // Set the sensor pins as input
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(sensorPin4, INPUT);
  pinMode(sensorPin5, INPUT);
  lcd.init();
  startTime = millis(); // Initialize startTime to the current time
}
int wallCount = 0;
void loop0() {
  unsigned long currentTime = millis(); // Get the current time
  distanceL = leftus.read();
  // Serial.print("Distance in CM: ");
  // Serial.println(distanceL);
  Serial.println("ana mazal madkholt lwalo");
  if (currentTime - startTime >= 2000 && currentTime - startTime <= 4000 && wallCount == 0) { // Check if 1000 ms have passed
    printOnLcd("RED");

  }
  if (distanceL != 0 && distanceL < 20 && wallCount == 0) {
    for (int i = 0; i < 200; i++) {
      left();
      delay(1);
      Serial.println("ana f for");
    }
    Serial.println("ana khrjt mn for");
    wallCount++;
  }
  else if (distanceL != 0 && distanceL < 20 && wallCount == 1) {
    move(0, 0);
    delay(5000);
    wallCount++;
    startTime = currentTime; // Reset startTime to the current time
  }
  if (wallCount == 2) {
    for (int i = 0; i < 600; i++) {
      right();
      delay(1);

    }
    Serial.println("ana ghadi right");
  } else {
    left();
    Serial.println("ana ghadi left");
  }

  if (currentTime - startTime >= 900 && wallCount == 2) { // Check if 1000 ms have passed
    printOnLcd("LOL");
  }

}
void loop() {
  left();
}
