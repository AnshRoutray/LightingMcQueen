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

// Distance thresholds
#define WALL_DETECTION_MAX 100  // 1 meter
#define WALL_DETECTION_MIN 10

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

Servo gripperServo;

enum RobotState {
  SCAN_FOR_WALL,
  MOVE_AWAY_FROM_WALL,
  FIND_BLACK_LINE,
  FOLLOW_BLACK_TO_GREEN,
  FOLLOW_GREEN_TO_RED,
  TURN_45_AND_FIND_BLACK,
  FOLLOW_BLACK_TO_END,
  MISSION_COMPLETE
};

RobotState currentState = SCAN_FOR_WALL;

struct Color {
  int red;
  int green;
  int blue;
};

struct WallData {
  int startAngle;
  int endAngle;
  int middleAngle;
  bool detected;
};

WallData wallInfo = {0, 0, 0, false};

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
  gripperServo.write(90);
  
  delay(2000);
  Serial.println("Module 4 - Wall Navigation Initialized!");
}

void loop() {
  Color detectedColor = readColor();
  int leftIR = analogRead(LEFT_IR_SENSOR);
  int rightIR = analogRead(RIGHT_IR_SENSOR);
  
  Serial.print("State: ");
  Serial.println(currentState);
  
  switch(currentState) {
    case SCAN_FOR_WALL:
      scanForWall();
      if (wallInfo.detected) {
        Serial.print("Wall detected! Middle angle: ");
        Serial.println(wallInfo.middleAngle);
        currentState = MOVE_AWAY_FROM_WALL;
      }
      break;
      
    case MOVE_AWAY_FROM_WALL:
      moveAwayFromWall();
      currentState = FIND_BLACK_LINE;
      break;
      
    case FIND_BLACK_LINE:
      // Keep moving forward, ignoring colors until black is found
      if (isBlack(detectedColor)) {
        Serial.println("Black line found!");
        stopMotors();
        delay(500);
        currentState = FOLLOW_BLACK_TO_GREEN;
      } else {
        // Ignore all colors, just move forward
        moveForward(BASE_SPEED);
      }
      break;
      
    case FOLLOW_BLACK_TO_GREEN:
      if (isGreen(detectedColor)) {
        Serial.println("Green line found!");
        stopMotors();
        delay(500);
        currentState = FOLLOW_GREEN_TO_RED;
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case FOLLOW_GREEN_TO_RED:
      if (isRed(detectedColor)) {
        Serial.println("Red line found!");
        stopMotors();
        delay(500);
        currentState = TURN_45_AND_FIND_BLACK;
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case TURN_45_AND_FIND_BLACK:
      turn45AndFindBlack();
      currentState = FOLLOW_BLACK_TO_END;
      break;
      
    case FOLLOW_BLACK_TO_END:
      // Follow black until it ends (both sensors see white)
      bool leftOnWhite = (analogRead(LEFT_IR_SENSOR) > WHITE_THRESHOLD);
      bool rightOnWhite = (analogRead(RIGHT_IR_SENSOR) > WHITE_THRESHOLD);
      
      if (leftOnWhite && rightOnWhite) {
        Serial.println("Black line ended!");
        stopMotors();
        currentState = MISSION_COMPLETE;
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case MISSION_COMPLETE:
      stopMotors();
      Serial.println("Mission complete!");
      while(true) {
        delay(1000);
      }
      break;
  }
  
  delay(10);
}

// ========== WALL DETECTION ==========

void scanForWall() {
  Serial.println("=== SCANNING 360° FOR WALL ===");
  
  stopMotors();
  delay(500);
  
  int currentAngle = 0;
  bool inWall = false;
  int wallStartAngle = -1;
  int wallEndAngle = -1;
  
  // Rotate 360 degrees and check distance at each degree
  while (currentAngle < 360) {
    // Turn 1 degree
    turnRightDegrees(1);
    currentAngle++;
    
    // Check distance
    int distance = getDistance();
    
    // Check if we're seeing the wall
    bool seeingWall = (distance > WALL_DETECTION_MIN && distance < WALL_DETECTION_MAX);
    
    if (seeingWall && !inWall) {
      // Started seeing wall
      wallStartAngle = currentAngle;
      inWall = true;
      Serial.print("Wall start detected at angle: ");
      Serial.println(currentAngle);
    } else if (!seeingWall && inWall) {
      // Stopped seeing wall
      wallEndAngle = currentAngle - 1;
      inWall = false;
      Serial.print("Wall end detected at angle: ");
      Serial.println(wallEndAngle);
      
      // Calculate middle of wall
      int wallSpan = wallEndAngle - wallStartAngle;
      if (wallSpan > 5) { // Only consider if wall is at least 5 degrees wide
        wallInfo.startAngle = wallStartAngle;
        wallInfo.endAngle = wallEndAngle;
        wallInfo.middleAngle = wallStartAngle + (wallSpan / 2);
        wallInfo.detected = true;
        
        Serial.print("Wall span: ");
        Serial.print(wallSpan);
        Serial.println(" degrees");
        
        break; // Found wall, stop scanning
      }
    }
    
    delay(50); // Small delay between measurements
  }
  
  if (!wallInfo.detected) {
    Serial.println("Warning: No wall detected in 360° scan!");
  }
}

void moveAwayFromWall() {
  Serial.println("=== MOVING 180° AWAY FROM WALL ===");
  
  stopMotors();
  delay(300);
  
  // Calculate angle 180 degrees away from wall middle
  int targetAngle = (wallInfo.middleAngle + 180) % 360;
  
  Serial.print("Turning to angle: ");
  Serial.println(targetAngle);
  
  // Turn to face away from wall
  turnRightDegrees(targetAngle);
  
  Serial.println("Now facing away from wall, searching for black line...");
  delay(300);
}

void turn45AndFindBlack() {
  Serial.println("=== TURNING 45° LEFT ===");
  
  stopMotors();
  delay(300);
  
  // Turn 45 degrees left
  turnLeftDegrees(45);
  
  Serial.println("Moving forward to find black line...");
  moveForward(BASE_SPEED);
  
  unsigned long startTime = millis();
  bool blackFound = false;
  
  // Search for black line for up to 5 seconds
  while (millis() - startTime < 5000 && !blackFound) {
    Color c = readColor();
    if (isBlack(c)) {
      blackFound = true;
      stopMotors();
      Serial.println("Black line found after 45° turn!");
      delay(300);
    }
    delay(50);
  }
  
  if (!blackFound) {
    Serial.println("Warning: Black line not found after 45° turn!");
    stopMotors();
  }
}

// ========== COLOR SENSING FUNCTIONS ==========

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

bool isGreen(Color c) {
  return (c.green > GREEN_G_MIN && c.green < GREEN_G_MAX && 
          c.red < GREEN_R_MAX && c.blue < GREEN_R_MAX);
}

bool isBlack(Color c) {
  int avg = (c.red + c.green + c.blue) / 3;
  return (avg < BLACK_THRESHOLD);
}

// ========== MOVEMENT FUNCTIONS ==========

void followLine(int leftIR, int rightIR) {
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
  int turnTime = degrees * 10; // 10ms per degree (CALIBRATE THIS!)
  
  turnLeft(TURN_SPEED);
  delay(turnTime);
  stopMotors();
  delay(50);
}

void turnRightDegrees(int degrees) {
  int turnTime = degrees * 10; // 10ms per degree (CALIBRATE THIS!)
  
  turnRight(TURN_SPEED);
  delay(turnTime);
  stopMotors();
  delay(50);
}

// ========== ULTRASONIC SENSOR ==========

int getDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return -1; // No echo received
  }
  
  int distance = duration * 0.034 / 2;
  return distance;
}
