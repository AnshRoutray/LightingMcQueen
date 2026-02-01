#include <Servo.h>

// Pin Definitions - YOUR ACTUAL WIRING
#define LEFT_IR_SENSOR 3
#define RIGHT_IR_SENSOR 12
#define COLOR_SENSOR_S0 11
#define COLOR_SENSOR_S1 9
#define COLOR_SENSOR_S2 6
#define COLOR_SENSOR_S3 7
#define COLOR_SENSOR_OUT 8
#define ULTRASONIC_TRIG 10
#define ULTRASONIC_ECHO 2
// Note: You mentioned ultrasonic OUT is 13, but typically ultrasonic has TRIG and ECHO
// If you need pin 13 for ultrasonic, let me know which signal it is

// Motors - NOT YET CONNECTED (placeholder pins)
#define LEFT_MOTOR_PWM 11
#define LEFT_MOTOR_DIR1 12
#define LEFT_MOTOR_DIR2 13
#define RIGHT_MOTOR_PWM A0
#define RIGHT_MOTOR_DIR1 A1
#define RIGHT_MOTOR_DIR2 A2
#define SERVO_PIN A3

// Motor speeds
#define BASE_SPEED 150
#define TURN_SPEED 120
#define SLOW_SPEED 100

// IR sensor logic (TEST THIS: does your sensor output HIGH on white or LOW on white?)
#define IR_ON_WHITE HIGH  // Change to LOW if your sensor outputs LOW on white

// Color thresholds (NEED CALIBRATION)
#define BLACK_THRESHOLD 400
#define RED_R_MIN 150
#define RED_R_MAX 255
#define RED_G_MIN 0
#define RED_G_MAX 100
#define BLUE_B_MIN 150
#define BLUE_B_MAX 255
#define BLUE_R_MAX 100
#define GREY_MIN 300
#define GREY_MAX 600

Servo gripperServo;

enum RobotState {
  FOLLOW_BLACK,
  FOLLOW_RED,
  REACHED_GREY
};

RobotState currentState = FOLLOW_BLACK;
bool blueDetected = false;

struct Color {
  int red;
  int green;
  int blue;
};

void setup() {
  Serial.begin(9600);
  
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  
  pinMode(COLOR_SENSOR_S0, OUTPUT);
  pinMode(COLOR_SENSOR_S1, OUTPUT);
  pinMode(COLOR_SENSOR_S2, OUTPUT);
  pinMode(COLOR_SENSOR_S3, OUTPUT);
  pinMode(COLOR_SENSOR_OUT, INPUT);
  
  digitalWrite(COLOR_SENSOR_S0, HIGH);
  digitalWrite(COLOR_SENSOR_S1, LOW);
  
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);
  
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(90);
  
  delay(2000);
  Serial.println("Module 1: FollowBlackOnRed initialized!");
}

void loop() {
  Color detectedColor = readColor();
  int leftIR = digitalRead(LEFT_IR_SENSOR);
  int rightIR = digitalRead(RIGHT_IR_SENSOR);
  
  switch(currentState) {
    case FOLLOW_BLACK:
      if (isRed(detectedColor)) {
        Serial.println("Red detected! Switching to red line following.");
        currentState = FOLLOW_RED;
        delay(200);
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case FOLLOW_RED:
      if (isBlue(detectedColor) && !blueDetected) {
        Serial.println("Blue detected! Rotating servo.");
        rotateServo();
        blueDetected = true;
        delay(500);
      }
      
      if (isGrey(detectedColor)) {
        Serial.println("Grey detected! Stopping at reupload point.");
        stopMotors();
        currentState = REACHED_GREY;
        delay(1000);
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case REACHED_GREY:
      stopMotors();
      Serial.println("Waiting at reupload point...");
      delay(5000);
      break;
  }
  
  delay(10);
}

Color readColor() {
  Color c;
  
  digitalWrite(COLOR_SENSOR_S2, LOW);
  digitalWrite(COLOR_SENSOR_S3, LOW);
  c.red = pulseIn(COLOR_SENSOR_OUT, LOW);
  delay(10);
  
  digitalWrite(COLOR_SENSOR_S2, HIGH);
  digitalWrite(COLOR_SENSOR_S3, HIGH);
  c.green = pulseIn(COLOR_SENSOR_OUT, LOW);
  delay(10);
  
  digitalWrite(COLOR_SENSOR_S2, LOW);
  digitalWrite(COLOR_SENSOR_S3, HIGH);
  c.blue = pulseIn(COLOR_SENSOR_OUT, LOW);
  
  return c;
}

bool isRed(Color c) {
  return (c.red > RED_R_MIN && c.red < RED_R_MAX && 
          c.green < RED_G_MAX && c.blue < RED_G_MAX);
}

bool isBlue(Color c) {
  return (c.blue > BLUE_B_MIN && c.blue < BLUE_B_MAX && 
          c.red < BLUE_R_MAX);
}

bool isGrey(Color c) {
  int avg = (c.red + c.green + c.blue) / 3;
  return (avg > GREY_MIN && avg < GREY_MAX);
}

void followLine(int leftIR, int rightIR) {
  bool leftOnWhite = (leftIR == IR_ON_WHITE);
  bool rightOnWhite = (rightIR == IR_ON_WHITE);
  
  if (!leftOnWhite && !rightOnWhite) {
    moveForward(BASE_SPEED);
  } else if (leftOnWhite && !rightOnWhite) {
    while (leftOnWhite) {
      turnRightDegrees(5);
      leftIR = digitalRead(LEFT_IR_SENSOR);
      leftOnWhite = (leftIR == IR_ON_WHITE);
    }
  } else if (!leftOnWhite && rightOnWhite) {
    while (rightOnWhite) {
      turnLeftDegrees(5);
      rightIR = digitalRead(RIGHT_IR_SENSOR);
      rightOnWhite = (rightIR == IR_ON_WHITE);
    }
  } else {
    moveForward(SLOW_SPEED);
  }
}

void moveForward(int speed) {
  digitalWrite(LEFT_MOTOR_DIR1, HIGH);
  digitalWrite(LEFT_MOTOR_DIR2, LOW);
  digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR2, LOW);
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}

void turnLeft(int speed) {
  digitalWrite(LEFT_MOTOR_DIR1, LOW);
  digitalWrite(LEFT_MOTOR_DIR2, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR2, LOW);
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}

void turnRight(int speed) {
  digitalWrite(LEFT_MOTOR_DIR1, HIGH);
  digitalWrite(LEFT_MOTOR_DIR2, LOW);
  digitalWrite(RIGHT_MOTOR_DIR1, LOW);
  digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_DIR1, LOW);
  digitalWrite(LEFT_MOTOR_DIR2, LOW);
  digitalWrite(RIGHT_MOTOR_DIR1, LOW);
  digitalWrite(RIGHT_MOTOR_DIR2, LOW);
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

void turnLeftDegrees(int degrees) {
  int turnTime = degrees * 10;
  turnLeft(TURN_SPEED);
  delay(turnTime);
  stopMotors();
  delay(50);
}

void turnRightDegrees(int degrees) {
  int turnTime = degrees * 10;
  turnRight(TURN_SPEED);
  delay(turnTime);
  stopMotors();
  delay(50);
}

void rotateServo() {
  int currentPos = gripperServo.read();
  int targetPos = currentPos - 40;
  if (targetPos < 0) targetPos = 0;
  gripperServo.write(targetPos);
  delay(500);
  Serial.print("Servo rotated to: ");
  Serial.println(targetPos);
}

int getDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}
