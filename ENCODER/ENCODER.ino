const int MOTOR_A_ENA = 13; // Right motor speed control
const int MOTOR_A_IN1 = 12;  // Right motor direction control 1
const int MOTOR_A_IN2 = 11;  // Right motor direction control 2
const int MOTOR_B_ENB = 8;  // Left motor speed control
const int MOTOR_B_IN3 = 9;  // Left motor direction control 1
const int MOTOR_B_IN4 = 10;  // Left motor direction control 2
// Encoder Sensor
int sensorRightPin = A0;
int sensorLeftPin = A1;

int sensorRightThreshold = 250;
int sensorLeftThreshold = 250;

int sensorRightLastState = 0;
int sensorLeftLastState = 0;

// Motors

// Ultrasons


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
void setup() {
  pinMode(MOTOR_A_ENA, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_ENB, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);

  pinMode(sensorRightPin, INPUT);
  pinMode(sensorLeftPin, INPUT);

  Serial.begin(9600);
}

int x, y, z = 0;

int i = 0;
int j = 0;
void moveL(int L, int R) {
  int turnL = L * 40;
  int turnR = R * 40;
  if (i <= turnR) {
    if (EncoderRightSeeHole()) {
      i++;
    }
    move(40, 0);
    Serial.println(i);
  } else {
    move(0, 0);
  }
  if (j <= turnL) {
    if (EncoderLeftSeeHole()) {
      j++;
    }
    move(0, 40);
    Serial.println(i);
  } else {
    move(0, 0);
  }


}
void loop() {
  moveL(0, 2);


  //  x = analogRead(sensorPin1);
  //  y = analogRead(sensorPin2);
  //
  //  Serial.print("sensorPin1 = ");
  //
  //  Serial.print(x);
  //
  //  Serial.print(" , sensorPin2 = ");
  //
  //  Serial.println(y);
  //  z++;
  //  Serial.println(z);
}



boolean SensorState(int sensorPin, int sensorThreshold) {
  int value = analogRead(sensorPin);

  return value > sensorThreshold;
}

boolean EncoderRightSeeHole() {
  int sensorCurrentState = SensorState(sensorRightPin, sensorRightThreshold);

  if (sensorCurrentState == 1 && sensorRightLastState != sensorCurrentState) {
    sensorRightLastState = sensorCurrentState;
    return true;
  }
  else if (sensorCurrentState == 0 && sensorRightLastState != sensorCurrentState) {
    sensorRightLastState = sensorCurrentState;
    return false ;
  }

  return false ;
}

boolean EncoderLeftSeeHole() {
  int sensorCurrentState = SensorState(sensorLeftPin, sensorLeftThreshold);

  if (sensorCurrentState == 1 && sensorLeftLastState != sensorCurrentState) {
    sensorLeftLastState = sensorCurrentState;
    return true;
  }
  else if (sensorCurrentState == 0 && sensorLeftLastState != sensorCurrentState) {
    sensorLeftLastState = sensorCurrentState;
    return false ;
  }

  return false ;
}
