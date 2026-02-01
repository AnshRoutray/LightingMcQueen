#include <Servo.h>

// Pin Definitions - YOUR ACTUAL WIRING
#define LEFT_IR_SENSOR A5
#define RIGHT_IR_SENSOR A0
#define COLOR_SENSOR_S0 11
#define COLOR_SENSOR_S1 9
#define COLOR_SENSOR_S2 6
#define COLOR_SENSOR_S3 7
#define COLOR_SENSOR_OUT 8
#define ULTRASONIC_TRIG 10
#define ULTRASONIC_ECHO 2

// Motor Driver
#define MOTOR_IN1 5
#define MOTOR_IN2 3
#define MOTOR_IN3 1
#define MOTOR_IN4 0

// Servo
#define SERVO_PIN 4

// Motor speeds
#define BASE_SPEED 150
#define TURN_SPEED 120
#define SLOW_SPEED 100

// Distance thresholds
#define WALL_DETECTION_MAX 100
#define WALL_DETECTION_MIN 10

// IR sensor logic
#define IR_ON_WHITE HIGH

// Color thresholds (NEED CALIBRATION)
#define BLACK_THRESHOLD 400
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
  
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(90);
  
  delay(2000);
  Serial.println("Module 4: GoFromTargetToBegin initialized!");
}

void loop() {
  Color detectedColor = readColor();
  int leftIR = digitalRead(LEFT_IR_SENSOR);
  int rightIR = digitalRead(RIGHT_IR_SENSOR);
  
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
      if (isBlack(detectedColor)) {
        Serial.println("Black line found!");
        stopMotors();
        delay(500);
        currentState = FOLLOW_BLACK_TO_GREEN;
      } else {
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
      {
        bool leftOnWhite = (digitalRead(LEFT_IR_SENSOR) == IR_ON_WHITE);
        bool rightOnWhite = (digitalRead(RIGHT_IR_SENSOR) == IR_ON_WHITE);
        
        if (leftOnWhite && rightOnWhite) {
          Serial.println("Black line ended!");
          stopMotors();
          currentState = MISSION_COMPLETE;
        } else {
          followLine(leftIR, rightIR);
        }
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

void scanForWall() {
  Serial.println("=== SCANNING 360° FOR WALL ===");
  
  stopMotors();
  delay(500);
  
  int currentAngle = 0;
  bool inWall = false;
  int wallStartAngle = -1;
  int wallEndAngle = -1;
  
  while (currentAngle < 360) {
    turnRightDegrees(1);
    currentAngle++;
    
    int distance = getDistance();
    bool seeingWall = (distance > WALL_DETECTION_MIN && distance < WALL_DETECTION_MAX);
    
    if (seeingWall && !inWall) {
      wallStartAngle = currentAngle;
      inWall = true;
      Serial.print("Wall start detected at angle: ");
      Serial.println(currentAngle);
    } else if (!seeingWall && inWall) {
      wallEndAngle = currentAngle - 1;
      inWall = false;
      Serial.print("Wall end detected at angle: ");
      Serial.println(wallEndAngle);
      
      int wallSpan = wallEndAngle - wallStartAngle;
      if (wallSpan > 5) {
        wallInfo.startAngle = wallStartAngle;
        wallInfo.endAngle = wallEndAngle;
        wallInfo.middleAngle = wallStartAngle + (wallSpan / 2);
        wallInfo.detected = true;
        
        Serial.print("Wall span: ");
        Serial.print(wallSpan);
        Serial.println(" degrees");
        
        break;
      }
    }
    
    delay(50);
  }
  
  if (!wallInfo.detected) {
    Serial.println("Warning: No wall detected in 360° scan!");
  }
}

void moveAwayFromWall() {
  Serial.println("=== MOVING 180° AWAY FROM WALL ===");
  
  stopMotors();
  delay(300);
  
  int targetAngle = (wallInfo.middleAngle + 180) % 360;
  
  Serial.print("Turning to angle: ");
  Serial.println(targetAngle);
  
  turnRightDegrees(targetAngle);
  
  Serial.println("Now facing away from wall, searching for black line...");
  delay(300);
}

void turn45AndFindBlack() {
  Serial.println("=== TURNING 45° LEFT ===");
  
  stopMotors();
  delay(300);
  
  turnLeftDegrees(45);
  
  Serial.println("Moving forward to find black line...");
  moveForward(BASE_SPEED);
  
  unsigned long startTime = millis();
  bool blackFound = false;
  
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

Color readColor() {
  Color c;
  
  digitalWrite(COLOR_SENSOR_S2, LOW);
  digitalWrite(COLOR_SENSOR_S3, LOW);
  c.red = pulseIn(COLOR_SENSOR_OUT, LOW);
  delay(10);
  
  digitalWrite(COLOR_SENSOR_S2, HIGH);
  digitalW
