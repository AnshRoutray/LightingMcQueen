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

#define CIRCLE_RADIUS 9.0
#define TARGET_DISTANCE 18.0

Servo gripperServo;

enum RobotState {
  SCAN_FOR_WALL,
  FIND_RED_LINE,
  FOLLOW_RED_TO_OPPOSITE,
  PUSH_BALL_TO_CENTER,
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
float distanceTraveled = 0.0;

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
  Serial.println("Module 3: NavigateTargetandPushBall initialized!");
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
        currentState = FIND_RED_LINE;
      }
      break;
      
    case FIND_RED_LINE:
      findAndAlignToRedLine();
      currentState = FOLLOW_RED_TO_OPPOSITE;
      distanceTraveled = 0.0;
      break;
      
    case FOLLOW_RED_TO_OPPOSITE:
      if (isRed(detectedColor) || isBlack(detectedColor)) {
        followLineAndTrackDistance(leftIR, rightIR);
        
        if (distanceTraveled >= TARGET_DISTANCE) {
          Serial.println("Reached opposite side of wall!");
          stopMotors();
          delay(500);
          currentState = PUSH_BALL_TO_CENTER;
        }
      } else {
        followLine(leftIR, rightIR);
      }
      break;
      
    case PUSH_BALL_TO_CENTER:
      pushBallToCenter();
      currentState = MISSION_COMPLETE;
      break;
      
    case MISSION_COMPLETE:
      stopMotors();
      Serial.println("Ball pushed to wall! Mission complete.");
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

void findAndAlignToRedLine() {
  Serial.println("=== FINDING RED LINE ===");
  
  stopMotors();
  delay(300);
  
  int targetAngle = (wallInfo.middleAngle + 180) % 360;
  
  Serial.print("Turning away from wall to angle: ");
  Serial.println(targetAngle);
  
  turnRightDegrees(targetAngle);
  
  Serial.println("Searching for red line...");
  moveForward(BASE_SPEED);
  
  unsigned long startTime = millis();
  bool redFound = false;
  
  while (millis() - startTime < 8000 && !redFound) {
    Color c = readColor();
    if (isRed(c)) {
      redFound = true;
      stopMotors();
      Serial.println("Red line found!");
      delay(300);
      
      Serial.println("Aligning to follow red line...");
      turnLeftDegrees(90);
    }
    delay(50);
  }
  
  if (!redFound) {
    Serial.println("Warning: Red line not found! Trying alternative search...");
    stopMotors();
    for (int i = 0; i < 8; i++) {
      turnLeftDegrees(45);
      delay(200);
      Color c = readColor();
      if (isRed(c)) {
        Serial.println("Red line found during rotation!");
        turnLeftDegrees(90);
        break;
      }
    }
  }
}

void followLineAndTrackDistance(int leftIR, int rightIR) {
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  
  bool leftOnWhite = (leftIR == IR_ON_WHITE);
  bool rightOnWhite = (rightIR == IR_ON_WHITE);
  
  if (!leftOnWhite && !rightOnWhite) {
    moveForward(BASE_SPEED);
    distanceTraveled += 10.0 * deltaTime;
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
    distanceTraveled += 5.0 * deltaTime;
  }
  
  lastTime = currentTime;
  
  if ((int)distanceTraveled % 2 == 0) {
    Serial.print("Distance traveled: ");
    Serial.print(distanceTraveled);
    Serial.println(" cm");
  }
}

void pushBallToCenter() {
  Serial.println("=== PUSHING BALL TO CENTER ===");
  
  stopMotors();
  delay(500);
  
  Serial.println("Turning toward center...");
  turnLeftDegrees(90);
  
  delay(300);
  
  Serial.println("Moving to push ball...");
  moveForward(BASE_SPEED);
  delay(2000);
  
  stopMotors();
  Serial.println("Ball should be pushed toward wall!");
  
  delay(500);
  
  Serial.println("Backing up...");
  moveBackward(BASE_SPEED);
  delay(1000);
  
  stopMotors();
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

bool isGreen(Color c) {
  return (c.green > GREEN_G_MIN && c.green < GREEN_G_MAX && 
          c.red < GREEN_R_MAX && c.blue < GREEN_R_MAX);
}

bool isBlack(Color c) {
  int avg = (c.red + c.green + c.blue) / 3;
  return (avg < BLACK_THRESHOLD);
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
  analogWrite(MOTOR_IN1, speed);
  analogWrite(MOTOR_IN2, 0);
  analogWrite(MOTOR_IN3, speed);
  analogWrite(MOTOR_IN4, 0);
}

void moveBackward(int speed) {
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, speed);
  analogWrite(MOTOR_IN3, 0);
  analogWrite(MOTOR_IN4, speed);
}

void turnLeft(int speed) {
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, speed);
  analogWrite(MOTOR_IN3, speed);
  analogWrite(MOTOR_IN4, 0);
}

void turnRight(int speed) {
  analogWrite(MOTOR_IN1, speed);
  analogWrite(MOTOR_IN2, 0);
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
