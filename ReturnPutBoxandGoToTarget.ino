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

// Distance threshold for obstacle detection (in cm)
#define OBSTACLE_DISTANCE 15

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
#define GREEN_G_MIN 150
#define GREEN_G_MAX 255
#define GREEN_R_MAX 100
#define GREY_MIN 300
#define GREY_MAX 600

Servo gripperServo;

enum RobotState {
  FOLLOW_RED_WITH_OBSTACLES,
  BLUE_PICKUP_SEQUENCE,
  FOLLOW_RED_TO_BLACK,
  TURN_TO_GREEN,
  FOLLOW_GREEN_TO_BLUE,
  GREEN_PICKUP_SEQUENCE,
  FOLLOW_GREEN_AFTER_PICKUP,
  FOLLOW_BLACK_TO_TARGET,
  REACHED_TARGET
};

RobotState currentState = FOLLOW_RED_WITH_OBSTACLES;

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
  
  delay(2000);
  Serial.println("Robot initialized!");
}

void loop() {
  Color detectedColor = readColor();
  int leftIR = analogRead(LEFT_IR_SENSOR);
  int rightIR = analogRead(RIGHT_IR_SENSOR);
  int distance = getDistance();
  
  // Debug output
  Serial.print("State: ");
  Serial.print(currentState);
  Serial.print(" | Distance: ");
  Serial.println(distance);
  
  switch(currentState) {
    case FOLLOW_RED_WITH_OBSTACLES:
      if (isBlue(detectedColor)) {
        Serial.println("Blue detected! Starting pickup sequence.");
        currentState = BLUE_PICKUP_SEQUENCE;
        bluePickupSequence();
      } else if (distance < OBSTACLE_DISTANCE && distance > 0) {
        Serial.println("Obstacle detected! Avoiding...");
        avoidObstacle();
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case BLUE_PICKUP_SEQUENCE:
      currentState = FOLLOW_RED_TO_BLACK;
      break;
      
    case FOLLOW_RED_TO_BLACK:
      if (isBlack(detectedColor)) {
        Serial.println("Black detected! Turning to green line.");
        currentState = TURN_TO_GREEN;
        turnToGreen();
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case TURN_TO_GREEN:
      currentState = FOLLOW_GREEN_TO_BLUE;
      break;
      
    case FOLLOW_GREEN_TO_BLUE:
      if (isBlue(detectedColor)) {
        Serial.println("Blue detected on green path! Starting green pickup.");
        currentState = GREEN_PICKUP_SEQUENCE;
        greenPickupSequence();
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case GREEN_PICKUP_SEQUENCE:
      currentState = FOLLOW_GREEN_AFTER_PICKUP;
      break;
      
    case FOLLOW_GREEN_AFTER_PICKUP:
      if (isBlack(detectedColor)) {
        Serial.println("Black detected after green! Switching to black line.");
        currentState = FOLLOW_BLACK_TO_TARGET;
        delay(200);
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case FOLLOW_BLACK_TO_TARGET:
      if (isBlue(detectedColor)) {
        Serial.println("Blue target detected! Moving to target.");
        moveForward(BASE_SPEED);
        delay(500);
        stopMotors();
        currentState = REACHED_TARGET;
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case REACHED_TARGET:
      stopMotors();
      Serial.println("Mission complete! Target reached.");
      while(true) {
        delay(1000);
      }
      break;
  }
  
  delay(10);
}

// ========== COLOR SENSING FUNCTIONS ==========

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

bool isGreen(Color c) {
  return (c.green > GREEN_G_MIN && c.green < GREEN_G_MAX && 
          c.red < GREEN_R_MAX && c.blue < GREEN_R_MAX);
}

bool isBlack(Color c) {
  int avg = (c.red + c.green + c.blue) / 3;
  return (avg < BLACK_THRESHOLD);
}

bool isGrey(Color c) {
  int avg = (c.red + c.green + c.blue) / 3;
  return (avg > GREY_MIN && avg < GREY_MAX);
}

// ========== MOVEMENT FUNCTIONS ==========

void followLine(int leftIR, int rightIR) {
  bool leftOnWhite = (leftIR > WHITE_THRESHOLD);
  bool rightOnWhite = (rightIR > WHITE_THRESHOLD);
  
  if (!leftOnWhite && !rightOnWhite) {
    // Both sensors on the line - keep going straight!
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
    // Both on white - lost line
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

void moveBackward(int speed) {
  digitalWrite(LEFT_MOTOR_DIR1, LOW);
  digitalWrite(LEFT_MOTOR_DIR2, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR1, LOW);
  digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
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

// ========== ANGLE-BASED TURNING FUNCTIONS ==========

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

// ========== OBSTACLE AVOIDANCE ==========

void avoidObstacle() {
  stopMotors();
  delay(200);
  
  Serial.println("Turning 45° left...");
  turnLeftDegrees(45);
  
  Serial.println("Moving forward...");
  moveForward(BASE_SPEED);
  delay(800);
  stopMotors();
  
  Serial.println("Turning 90° right...");
  turnRightDegrees(90);
  
  Serial.println("Moving forward past obstacle...");
  moveForward(BASE_SPEED);
  delay(1000);
  stopMotors();
  
  Serial.println("Turning 45° left...");
  turnLeftDegrees(45);
  
  Serial.println("Searching for red line...");
  moveForward(BASE_SPEED);
  
  unsigned long startTime = millis();
  bool redFound = false;
  
  while (millis() - startTime < 5000 && !redFound) {
    Color c = readColor();
    if (isRed(c)) {
      redFound = true;
      stopMotors();
      Serial.println("Red line found!");
      delay(200);
      
      Serial.println("Turning 45° right to realign...");
      turnRightDegrees(45);
    }
    delay(50);
  }
  
  if (!redFound) {
    Serial.println("Warning: Red line not found in 5 seconds!");
    stopMotors();
  }
  
  Serial.println("Obstacle avoided! Resuming line following.");
}

// ========== PICKUP SEQUENCES ==========

void bluePickupSequence() {
  Serial.println("=== BLUE PICKUP SEQUENCE ===");
  
  stopMotors();
  delay(300);
  
  Serial.println("Turning 90° left...");
  turnLeftDegrees(90);
  
  Serial.println("Moving forward...");
  moveForward(BASE_SPEED);
  delay(600);
  stopMotors();
  delay(200);
  
  Serial.println("Releasing object (servo +40°)...");
  int currentPos = gripperServo.read();
  gripperServo.write(currentPos + 40);
  delay(800);
  
  Serial.println("Moving backward...");
  moveBackward(BASE_SPEED);
  delay(600);
  stopMotors();
  delay(200);
  
  Serial.println("Turning 90° right...");
  turnRightDegrees(90);
  
  Serial.println("Blue pickup complete!");
  delay(300);
}

void greenPickupSequence() {
  Serial.println("=== GREEN PICKUP SEQUENCE ===");
  
  stopMotors();
  delay(300);
  
  Serial.println("Grabbing object (servo -40°)...");
  int currentPos = gripperServo.read();
  gripperServo.write(currentPos - 40);
  delay(800);
  
  Serial.println("Turning 110° left...");
  turnLeftDegrees(110);
  
  Serial.println("Releasing object (servo +40°)...");
  currentPos = gripperServo.read();
  gripperServo.write(currentPos + 40);
  delay(800);
  
  Serial.println("Backing up...");
  moveBackward(BASE_SPEED);
  delay(600);
  stopMotors();
  delay(200);
  
  Serial.println("Turning 110° right...");
  turnRightDegrees(110);
  
  Serial.println("Green pickup complete!");
  delay(300);
}

void turnToGreen() {
  Serial.println("=== TURNING TO GREEN LINE ===");
  
  stopMotors();
  delay(200);
  
  Serial.println("Turning 90° left to find green...");
  turnLeftDegrees(90);
  
  Serial.println("Moving forward to find green line...");
  moveForward(BASE_SPEED);
  
  unsigned long startTime = millis();
  bool greenFound = false;
  
  while (millis() - startTime < 5000 && !greenFound) {
    Color c = readColor();
    if (isGreen(c)) {
      greenFound = true;
      stopMotors();
      Serial.println("Green line found!");
    }
    delay(50);
  }
  
  if (!greenFound) {
    Serial.println("Warning: Green line not found in 5 seconds!");
    stopMotors();
  }
  
  delay(300);
}

// ========== ULTRASONIC SENSOR ==========

int getDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  
  if (duration == 0) {
    return -1;
  }
  
  int distance = duration * 0.034 / 2;
  return distance;
}
