#include <Servo.h>

// Pin Definitions
#define LEFT_IR_SENSOR A0
#define RIGHT_IR_SENSOR A1
#define COLOR_SENSOR_S0 2
#define COLOR_SENSOR_S1 3
#define COLOR_SENSOR_S2 4
#define COLOR_SENSOR_S3 5
#define COLOR_SENSOR_OUT 6
#define ULTRASONIC_TRIG 7
#define ULTRASONIC_ECHO 8
#define LEFT_MOTOR_PWM 9
#define LEFT_MOTOR_DIR1 10
#define LEFT_MOTOR_DIR2 11
#define RIGHT_MOTOR_PWM 12
#define RIGHT_MOTOR_DIR1 13
#define RIGHT_MOTOR_DIR2 A2
#define SERVO_PIN A3

// Motor speeds
#define BASE_SPEED 150
#define TURN_SPEED 120
#define SLOW_SPEED 100

// Color thresholds (calibrate these for your sensor)
#define BLACK_THRESHOLD 400
#define WHITE_THRESHOLD 800
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
  
  // Set frequency scaling to 20%
  digitalWrite(COLOR_SENSOR_S0, HIGH);
  digitalWrite(COLOR_SENSOR_S1, LOW);
  
  // Initialize ultrasonic sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  // Initialize motors
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);
  
  // Initialize servo
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(90); // Initial position
  
  delay(2000); // Startup delay
  Serial.println("Robot initialized!");
}

void loop() {
  Color detectedColor = readColor();
  int leftIR = analogRead(LEFT_IR_SENSOR);
  int rightIR = analogRead(RIGHT_IR_SENSOR);
  
  // State machine
  switch(currentState) {
    case FOLLOW_BLACK:
      // Check if red is detected to transition
      if (isRed(detectedColor)) {
        Serial.println("Red detected! Switching to red line following.");
        currentState = FOLLOW_RED;
        delay(200);
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case FOLLOW_RED:
      // Check for blue to rotate servo
      if (isBlue(detectedColor) && !blueDetected) {
        Serial.println("Blue detected! Rotating servo.");
        rotateServo();
        blueDetected = true;
        delay(500);
      }
      
      // Check for grey to stop
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
      // You can add code here to wait for new instructions
      break;
  }
  
  delay(10);
}

Color readColor() {
  Color c;
  
  // Read RED
  digitalWrite(COLOR_SENSOR_S2, LOW);
  digitalWrite(COLOR_SENSOR_S3, LOW);
  c.red = pulseIn(COLOR_SENSOR_OUT, LOW);
  
  delay(10);
  
  // Read GREEN
  digitalWrite(COLOR_SENSOR_S2, HIGH);
  digitalWrite(COLOR_SENSOR_S3, HIGH);
  c.green = pulseIn(COLOR_SENSOR_OUT, LOW);
  
  delay(10);
  
  // Read BLUE
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
  // White surface = high value, Black line = low value
  bool leftOnWhite = (leftIR > WHITE_THRESHOLD);
  bool rightOnWhite = (rightIR > WHITE_THRESHOLD);
  
  if (!leftOnWhite && !rightOnWhite) {
    // Both sensors on the line - keep going straight continuously!
    moveForward(BASE_SPEED);
  } else if (leftOnWhite && !rightOnWhite) {
    // Left sees white - turn right until back on line
    while (leftOnWhite) {
      turnRightDegrees(5);
      leftIR = analogRead(LEFT_IR_SENSOR);
      leftOnWhite = (leftIR > WHITE_THRESHOLD);
    }
  } else if (!leftOnWhite && rightOnWhite) {
    // Right sees white - turn left until back on line
    while (rightOnWhite) {
      turnLeftDegrees(5);
      rightIR = analogRead(RIGHT_IR_SENSOR);
      rightOnWhite = (rightIR > WHITE_THRESHOLD);
    }
  } else {
    // Both on white - lost line, move slowly
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
  int turnTime = degrees * 10; // 10ms per degree (adjust as needed)
  
  turnLeft(TURN_SPEED);
  delay(turnTime);
  stopMotors();
  delay(50); // Short pause after turn
}

void turnRightDegrees(int degrees) {
  int turnTime = degrees * 10; // 10ms per degree (adjust as needed)
  
  turnRight(TURN_SPEED);
  delay(turnTime);
  stopMotors();
  delay(50); // Short pause after turn
}

void rotateServo() {
  int currentPos = gripperServo.read();
  int targetPos = currentPos - 40; // 40 degrees counterclockwise
  
  if (targetPos < 0) targetPos = 0;
  
  gripperServo.write(targetPos);
  delay(500); // Wait for servo to complete movement
  
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
