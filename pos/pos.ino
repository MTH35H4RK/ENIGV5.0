#include <QTRSensors.h>
int threshold[13] = {450 , 400 , 400 , 400 , 400 , 400, 400, 400, 400, 400, 400, 400, 400};
QTRSensors qtr;
const uint8_t SensorCount = 13;
uint16_t sensorValues[SensorCount];

const int MOTOR_A_ENA = 4; // Right motor speed control
const int MOTOR_A_IN1 = 5;  // Right motor direction control 1
const int MOTOR_A_IN2 = 6;  // Right motor direction control 2
const int MOTOR_B_ENB = 9;  // Left motor speed control
const int MOTOR_B_IN3 = 10;  // Left motor direction control 1
const int MOTOR_B_IN4 = 11;  // Left motor direction control 2
void move(int speed1, int speed2) {
  analogWrite(MOTOR_A_ENA, speed1);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_B_ENB, speed2);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
}


void right() {
  qtr.readLineBlack(sensorValues);
  qtr.read(sensorValues);
  move(0, 90);
  // Check if the robot is on the line
  if (sensorValues[0] > threshold[0] &&
      sensorValues[1] > threshold[1] &&
      sensorValues[2] > threshold[2] &&
      sensorValues[3] > threshold[3] &&
      sensorValues[4] > threshold[4] &&
      sensorValues[5] > threshold[5] &&
      sensorValues[6] > threshold[6] &&
      sensorValues[7] > threshold[7] &&
      sensorValues[8] > threshold[8] &&
      sensorValues[9] > threshold[9] &&
      sensorValues[10] > threshold[10] &&
      sensorValues[11] > threshold[11] &&
      sensorValues[12] > threshold[12]) {
    // Stop the robot
    move(0, 90); //turn right slightly
  } else {
    // Calculate the position of the robot on the line

    int position = 0;

    if (sensorValues[6] > threshold[6]) {
      position = 7;
    } if (sensorValues[7] > threshold[7]) {
      position = 6;
    }    if (sensorValues[8] > threshold[8]) {
      position = 5;
    }    if (sensorValues[9] > threshold[9]) {
      position = 4;
    }    if (sensorValues[10] > threshold[10]) {
      position = 3;
    }    if (sensorValues[11] > threshold[11]) {
      position = 2;
    }    if (sensorValues[12] > threshold[12]) {
      position = 1;
    }  if (sensorValues[5] > threshold[7]) {
      position = 8;
    }  if (sensorValues[4] > threshold[7]) {
      position = 9;
    }  if (sensorValues[3] > threshold[7]) {
      position = 10;
    }  if (sensorValues[2] > threshold[7]) {
      position = 11;
    }  if (sensorValues[1] > threshold[7]) {
      position = 12;
    }  if (sensorValues[0] > threshold[7]) {
      position = 13;
    }

    Serial.println(position);


    // Adjust the motor speed and direction based on the position
    if (position == 1) {
      move(110, 0);
    } else if (position == 2) {
      move(100, 10);
    } else if (position == 3) {
      move(90, 20);
    } else if (position == 4) {
      move(80, 30);
    } else if (position == 5) {
      move(70, 40);
    } else if (position == 6) {
      move(60, 50);
    } else if (position == 7) {
      move(100, 100);
    } else if (position == 8) {
      move(50, 60);
    } else if (position == 9) {
      move(40, 70);
    } else if (position == 10) {
      move(30, 80);
    } else if (position == 11) {
      move(20, 90);
    } else if (position == 12) {
      move(100, 10);
    } else if (position == 13) {
      move(110, 0);
    }
  }
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
  qtr.setEmitterPin(7);

  // Calibrate sensors during setup
  Serial.println("Starting calibration...");
  for (uint16_t i = 0; i > 400; i++) { // 400 * 25 ms = 10 seconds
    qtr.calibrate();
  }
  Serial.println("Calibration complete");

}

void loop() {
  right();
}
