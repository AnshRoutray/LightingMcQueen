#include <Servo.h>

// Pin Definitions - YOUR ACTUAL WIRING
#define LEFT_IR_SENSOR A5      // Digital read on analog pin
#define RIGHT_IR_SENSOR A0     // Digital read on analog pin
#define COLOR_SENSOR_S0 11
#define COLOR_SENSOR_S1 9
#define COLOR_SENSOR_S2 6
#define COLOR_SENSOR_S3 7
#define COLOR_SENSOR_OUT 8
#define ULTRASONIC_TRIG 10
#define ULTRASONIC_ECHO 2

// Motor Driver Pins (L298N or similar)
#define MOTOR_IN1 5    // Left motor direction
#define MOTOR_IN2 3    // Left motor direction
#define MOTOR_IN3 1    // Right motor direction
#define MOTOR_IN4 0    // Right motor direction

// Servo
#define SERVO_PIN 4

// Motor speeds (0-255)
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
  
  // Initialize IR sensors
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  
  // Initialize color sensor
  pinMode(COLOR_SENSOR_S0, OUTPUT);
  pinMode(COLOR_SENSOR_S1, OUTPUT);
  pinMode(COLOR_SENSOR_S2, OUTPUT);
  pinMode(COLOR_SENSOR_S3, OUTPUT);
  pinMode(COLOR_SENSOR_OUT, INPUT);
  
  digitalWrite(COLOR_SENSOR_S0, HIGH);
  digitalWrite(COLOR_SENSOR_S1, LOW);
  
  // Initialize ultrasonic sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  // Initialize motor driver
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  
  // Initialize servo
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
    // Both on line - go straight
    moveForward(BASE_SPEED);
  } else if (leftOnWhite && !rightOnWhite) {
    // Left on white - turn right until back on line
    while (leftOnWhite) {
      turnRightDegrees(5);
      leftIR = digitalRead(LEFT_IR_SENSOR);
      leftOnWhite = (leftIR == IR_ON_WHITE);
    }
  } else if (!leftOnWhite && rightOnWhite) {
    // Right on white - turn left until back on line
    while (rightOnWhite) {
      turnLeftDegrees(5);
      rightIR = digitalRead(RIGHT_IR_SENSOR);
      rightOnWhite = (rightIR == IR_ON_WHITE);
    }
  } else {
    // Both on white - lost line
    moveForward(SLOW_SPEED);
  }
}

void moveForward(int speed) {
  // Left motor forward
  analogWrite(MOTOR_IN1, speed);
  analogWrite(MOTOR_IN2, 0);
  // Right motor forward
  analogWrite(MOTOR_IN3, speed);
  analogWrite(MOTOR_IN4, 0);
}

void moveBackward(int speed) {
  // Left motor backward
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, speed);
  // Right motor backward
  analogWrite(MOTOR_IN3, 0);
  analogWrite(MOTOR_IN4, speed);
}

void turnLeft(int speed) {
  // Left motor backward
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, speed);
  // Right motor forward
  analogWrite(MOTOR_IN3, speed);
  analogWrite(MOTOR_IN4, 0);
}

void turnRight(int speed) {
  // Left motor forward
  analogWrite(MOTOR_IN1, speed);
  analogWrite(MOTOR_IN2, 0);
  // Right motor backward
  analogWrite(MOTOR_IN3, 0);
  analogWrite(MOTOR_IN4, speed);
}

void stopMotors() {
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
  analogWrite(MOTOR_IN3, 0);
  analogWrite(MOTOR_IN4, 0);
}

void turnLeftDegrees(int degrees) {
  int turnTime = degrees * 10; // Calibrate this!
  turnLeft(TURN_SPEED);
  delay(turnTime);
  stopMotors();
  delay(50);
}

void turnRightDegrees(int degrees) {
  int turnTime = degrees * 10; // Calibrate this!
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
