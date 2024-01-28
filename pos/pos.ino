#include <QTRSensors.h>
int threshold[] = {750 , 750 , 750 , 750 , 750 , 750, 750, 750, 750, 750, 750, 750, 750};
QTRSensors qtr;
const uint8_t SensorCount = 13;
uint16_t sensorValues[SensorCount];

const int MOTOR_A_ENA = 2; // Right motor speed control
const int MOTOR_A_IN1 = 3;  // Right motor direction control 1
const int MOTOR_A_IN2 = 4;  // Right motor direction control 2
const int MOTOR_B_ENB = 5;  // Left motor speed control
const int MOTOR_B_IN3 = 6;  // Left motor direction control 1
const int MOTOR_B_IN4 = 7;  // Left motor direction control 2
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
  qtr.readLineBlack(sensorValues);
  //  qtr.read(sensorValues);
  // move(0, 90);
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
    //  move(0, 90); //turn right slightly
  } else {
    // Calculate the position of the robot on the line

    int position = 0;

    if (sensorValues[6] > threshold[6]) {
      position = 7;
    } if (sensorValues[7] > threshold[7]) {
      position = 6;
    } if (sensorValues[5] > threshold[7]) {
      position = 8;
    } if (sensorValues[8] > threshold[8]) {
      position = 5;
    }    if (sensorValues[9] > threshold[9]) {
      position = 4;
    }    if (sensorValues[10] > threshold[10]) {
      position = 3;
    }    if (sensorValues[11] > threshold[11]) {
      position = 2;
    }    if (sensorValues[12] > threshold[1]) {
      position = 1;
    }    if (sensorValues[4] > threshold[4]) {
      position = 9;
    }  if (sensorValues[3] > threshold[3]) {
      position = 10;
    }  if (sensorValues[2] > threshold[2]) {
      position = 11;
    }  if (sensorValues[1] > threshold[1]) {
      position = 12;
    }  if (sensorValues[0] > threshold[0]) {
      position = 13;
    }

    Serial.println(position);
    // Adjust the motor speed and direction based on the position
    if (position == 1) {
      move(85, -50);
    } else if (position == 2) {
      move(80, -35);
    } else if (position == 3) {
      move(75, -20);
    } else if (position == 4) {
      move(70, -5);
    } else if (position == 5) {
      move(65, 10);
    } else if (position == 6) {
      move(60, 55);
    } else if (position == 7) {
      move(80, 80);
    } else if (position == 8) {
      move(55, 60);
    } else if (position == 9) {
      move(10, 65);
    } else if (position == 10) {
      move(-5, 70);
    } else if (position == 11) {
      move(-20, 75);
    } else if (position == 12) {
      move(-35, 80);
    } else if (position == 13) {
      move(-50, 85);
    }
  }
}

void left() {
  uint16_t value = qtr.readLineBlack(sensorValues);
  //  qtr.read(sensorValues);
  // move(0, 90);
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
    //  move(0, 90); //turn right slightly
  } else {
    // Calculate the position of the robot on the line

    int position = 0;

    if (sensorValues[6] > threshold[6]) {
      position = 7;
      Serial.print("ana dkhlt l 7   ");
    } if (sensorValues[5] > threshold[5]) {
      position = 8;          Serial.print("ana dkhlt l 8   ");

    } if (sensorValues[4] > threshold[4]) {
      position = 9;          Serial.print("ana dkhlt l 9   ");

    }  if (sensorValues[3] > threshold[3]) {
      position = 10;          Serial.print("ana dkhlt l 10   ");

    }  if (sensorValues[2] > threshold[2]) {
      position = 11;          Serial.print("ana dkhlt l 11   ");

    }  if (sensorValues[1] > threshold[1]) {
      position = 12;          Serial.print("ana dkhlt l 12  ");

    }  if (sensorValues[0] > threshold[0]) {
      position = 13;          Serial.print("ana dkhlt l 13   ");

    } if (sensorValues[7] > threshold[7]) {
      position = 6;          Serial.print("ana dkhlt l 6   ");

    }  if (sensorValues[8] > threshold[8]) {
      position = 5;          Serial.print("ana dkhlt l 5   ");

    }    if (sensorValues[9] > threshold[9]) {
      position = 4;          Serial.print("ana dkhlt l 4   ");

    }    if (sensorValues[10] > threshold[10]) {
      position = 3;          Serial.print("ana dkhlt l 3   ");

    }    if (sensorValues[11] > threshold[11]) {
      position = 2;          Serial.print("ana dkhlt l 2   ");

    }  if (sensorValues[12] > threshold[12]) {
       position = 1;          Serial.print("ana dkhlt l 1   ");

      }

    Serial.println(position);
    // Adjust the motor speed and direction based on the position
    if (position == 1) {
      move(80, -35);
    } else if (position == 2) {
      move(80, -30);
    } else if (position == 3) {
      move(80, 0);
    } else if (position == 4) {
      move(80, 5);
    } else if (position == 5) {
      move(89, 10);
    } else if (position == 6) {
      move(80, 80);
    } else if (position == 7) {
      move(80, 80);
    } else if (position == 8) {
      move(80, 80);
    } else if (position == 9) {
      move(10, 80);
    } else if (position == 10) {
      move(5, 80);
    } else if (position == 11) {
      move(10, 80);
    } else if (position == 12) {
      move(0, 80);
    } else if (position == 13) {
      move(-35, 80);
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
  qtr.setEmitterPin(8);

  // Calibrate sensors during setup
  Serial.println("Starting calibration...");
 for (uint16_t i = 0; i < 250; i++) { // 400 * 25 ms = 10 seconds
    qtr.calibrate();
  }
  Serial.println("Calibration complete");

}

void loop() {
left();
}
