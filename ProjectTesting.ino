#include <Servo.h>

const uint8_t F_ECHO_PIN = 2;
const uint8_t F_TRIG_PIN = 3;
const uint8_t MOTOR_PIN = 4;
const uint8_t SERVO_PIN = 5;
const float SPEED_OF_SOUND = 0.0343;  // cm/us
const float WALL_DIST = 200;          // cm
const uint8_t LEFT_MOTOR_PIN = 6;
const uint8_t RIGHT_MOTOR_PIN = 7;
const int MAX_MOTOR_SPEED = 255;
uint8_t leftMotorSpeed = 255;
uint8_t rightMotorSpeed = 255;
Servo servo;
int servoPos = 90;
const int SERVO_CF = 0;
const int TURN_DURATION = 500;  // ms
const uint8_t L_ECHO_PIN = 8;
const uint8_t L_TRIG_PIN = 9;
const uint8_t R_ECHO_PIN = 10;
const uint8_t R_TRIG_PIN = 11;
const float LR_TOL = 35;
const uint8_t LEFT_INDICATOR_PIN = 40;
const uint8_t RIGHT_INDICATOR_PIN = 41;
float prevDiff = 0;

void setup() {
  pinMode(F_ECHO_PIN, INPUT);
  pinMode(L_ECHO_PIN, INPUT);
  pinMode(R_ECHO_PIN, INPUT);
  pinMode(F_TRIG_PIN, OUTPUT);
  pinMode(L_TRIG_PIN, OUTPUT);
  pinMode(R_TRIG_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_INDICATOR_PIN, OUTPUT);
  pinMode(RIGHT_INDICATOR_PIN, OUTPUT);

  Serial.begin(9600);

  servo.attach(SERVO_PIN);
  digitalWrite(MOTOR_PIN, HIGH);
}

void loop() {
  servo.write(servoPos);
  analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
  analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);

  float forwardDist = getForwardDistance();

  float lrDiff = getLRDiff();

  if (forwardDist < WALL_DIST) {                           // If obstacle of wall detected, turn
    String turnDirection = lrDiff < 0 ? "left" : "right";  // Decide turn direction based on difference between left (negative) and right (positive)
    if (lrDiff > 0) {
      turnLeft();
    } else {
      turnRight();
    }
  } else {
    if (abs(lrDiff) > LR_TOL) {  // Vehicle is not centered, so center it
      centerVehicle();
    } else {
      continueStraight();
    }
  }

  delay(100);
  digitalWrite(LEFT_INDICATOR_PIN, LOW);
  digitalWrite(RIGHT_INDICATOR_PIN, LOW);

  prevDiff = forwardDist;
}

// Gets the proximity readings for the forward ultrasonic sensor
float getForwardDistance() {
  digitalWrite(F_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(F_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(F_TRIG_PIN, LOW);

  float time = pulseIn(F_ECHO_PIN, HIGH);
  float distance = (SPEED_OF_SOUND * time) / 2;

  return distance;
}

// Maintains the current heading
void continueStraight() {
  servoPos = 90 + SERVO_CF;
  leftMotorSpeed = MAX_MOTOR_SPEED;
  rightMotorSpeed = MAX_MOTOR_SPEED;

  servo.write(servoPos);
  analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
  analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);

  delay(100);
}

// Get distance difference between left and right sensors (left - right)
float getLRDiff() {
  digitalWrite(L_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(L_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(L_TRIG_PIN, LOW);

  float lTime = pulseIn(L_ECHO_PIN, HIGH);

  delay(100);

  digitalWrite(R_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(R_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(R_TRIG_PIN, LOW);

  float rTime = pulseIn(R_ECHO_PIN, HIGH);

  float lDist = (SPEED_OF_SOUND * lTime) / 2;
  float rDist = (SPEED_OF_SOUND * rTime) / 2;

  return lDist - rDist;
}

void turnLeft() {
  servoPos = 120 + SERVO_CF;
  leftMotorSpeed = MAX_MOTOR_SPEED;
  rightMotorSpeed = 0.5 * MAX_MOTOR_SPEED;

  servo.write(servoPos);
  analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
  analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);

  for (int i = 0; i < TURN_DURATION; i += 100) {
    bool indicatorState = digitalRead(LEFT_INDICATOR_PIN);
    digitalWrite(LEFT_INDICATOR_PIN, !indicatorState);
    delay(100);
  }
}

void turnRight() {
  servoPos = 60 + SERVO_CF;
  leftMotorSpeed = 0.5 * MAX_MOTOR_SPEED;
  rightMotorSpeed = MAX_MOTOR_SPEED;

  servo.write(servoPos);
  analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
  analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);

  for (int i = 0; i < TURN_DURATION; i += 100) {
    bool indicatorState = digitalRead(RIGHT_INDICATOR_PIN);
    digitalWrite(RIGHT_INDICATOR_PIN, !indicatorState);
    delay(100);
  }
}

void centerVehicle() {
  float initDiff = getLRDiff();

  delay(100);

  float lrDiff = getLRDiff();  // get new vehicle position

  if (initDiff < 0) {          // if vehicle on left of hall
    if (lrDiff <= initDiff) {  // if vehicle moving further left or straight
      servoPos = 80 + SERVO_CF;
      leftMotorSpeed = MAX_MOTOR_SPEED;
      rightMotorSpeed = 0.80 * MAX_MOTOR_SPEED;

      servo.write(servoPos);
      analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
      analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);

      lrDiff = getLRDiff();
      prevDiff = lrDiff + 1;

      while (lrDiff < 0 && lrDiff < prevDiff) {
        prevDiff = lrDiff;
        lrDiff = getLRDiff();
        blinkRightIndicator();

        delay(500);
      }
    }

    servoPos = 80 + SERVO_CF;
    leftMotorSpeed = 0.90 * MAX_MOTOR_SPEED;
    rightMotorSpeed = MAX_MOTOR_SPEED;

    servo.write(servoPos);
    analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
    analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);

    lrDiff = getLRDiff();

    while (lrDiff < 0 && abs(lrDiff) > LR_TOL) {
      lrDiff = getLRDiff();
      blinkRightIndicator();

      delay(100);
    }
  } else {                     // if vehicle on right of hall
    if (lrDiff >= initDiff) {  // if vehicle moving further right or straight
      servoPos = 100 + SERVO_CF;
      leftMotorSpeed = 0.80 * MAX_MOTOR_SPEED;
      rightMotorSpeed = MAX_MOTOR_SPEED;

      servo.write(servoPos);
      analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
      analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);

      lrDiff = getLRDiff();
      prevDiff = lrDiff - 1;

      while (lrDiff > 0 && lrDiff > prevDiff) {
        prevDiff = lrDiff;
        lrDiff = getLRDiff();
        blinkLeftIndicator();

        delay(500);
      }
    }

    servoPos = 100 + SERVO_CF;
    leftMotorSpeed = 0.90 * MAX_MOTOR_SPEED;
    rightMotorSpeed = MAX_MOTOR_SPEED;

    servo.write(servoPos);
    analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
    analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);

    lrDiff = getLRDiff();

    while (lrDiff > 0 && abs(lrDiff) > LR_TOL) {
      lrDiff = getLRDiff();
      blinkLeftIndicator();

      delay(100);
    }
  }

  servoPos = 90 + SERVO_CF;
  leftMotorSpeed = MAX_MOTOR_SPEED;
  rightMotorSpeed = MAX_MOTOR_SPEED;

  servo.write(servoPos);
  analogWrite(LEFT_MOTOR_PIN, leftMotorSpeed);
  analogWrite(RIGHT_MOTOR_PIN, rightMotorSpeed);
  digitalWrite(LEFT_INDICATOR_PIN, LOW);
  digitalWrite(RIGHT_INDICATOR_PIN, LOW);
}

// Toggle left indicator state
void blinkLeftIndicator() {
  bool indicatorState = digitalRead(LEFT_INDICATOR_PIN);
  digitalWrite(LEFT_INDICATOR_PIN, !indicatorState);
}

// Toggle right indicator state
void blinkRightIndicator() {
  bool indicatorState = digitalRead(RIGHT_INDICATOR_PIN);
  digitalWrite(RIGHT_INDICATOR_PIN, !indicatorState);
}
