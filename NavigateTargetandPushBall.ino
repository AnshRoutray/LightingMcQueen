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
#define WALL_DETECTION_MAX 100  // 1 meter = 100cm
#define WALL_DETECTION_MIN 10   // Minimum valid distance

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

// Circle dimensions from image
#define CIRCLE_RADIUS 9.0  // 9cm from center to red line
#define TARGET_DISTANCE 18.0  // Diameter = 18cm to reach opposite side

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
  Serial.println("Wall Detection Module Initialized!");
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
        // Follow the line and track distance
        followLineAndTrackDistance(leftIR, rightIR);
        
        // Check if we've traveled half the circle (reached opposite point)
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

void findAndAlignToRedLine() {
  Serial.println("=== FINDING RED LINE ===");
  
  stopMotors();
  delay(300);
  
  // Calculate angle away from wall (perpendicular to wall middle)
  // We want to move away from the wall to find the red line
  int targetAngle = (wallInfo.middleAngle + 180) % 360; // Opposite direction from wall
  
  Serial.print("Turning away from wall to angle: ");
  Serial.println(targetAngle);
  
  // Turn to face away from wall
  turnRightDegrees(targetAngle);
  
  // Move forward while searching for red line
  Serial.println("Searching for red line...");
  moveForward(BASE_SPEED);
  
  unsigned long startTime = millis();
  bool redFound = false;
  
  while (millis() - startTime < 8000 && !redFound) { // 8 second timeout
    Color c = readColor();
    if (isRed(c)) {
      redFound = true;
      stopMotors();
      Serial.println("Red line found!");
      delay(300);
      
      // Now align perpendicular to continue around circle
      // Turn 90 degrees to follow the red circle
      Serial.println("Aligning to follow red line...");
      turnLeftDegrees(90);
    }
    delay(50);
  }
  
  if (!redFound) {
    Serial.println("Warning: Red line not found! Trying alternative search...");
    stopMotors();
    // Try rotating to find it
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
  // Record starting time for distance calculation
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  
  // Follow the line
  bool leftOnWhite = (leftIR > WHITE_THRESHOLD);
  bool rightOnWhite = (rightIR > WHITE_THRESHOLD);
  
  if (!leftOnWhite && !rightOnWhite) {
    moveForward(BASE_SPEED);
    // Estimate distance: speed * time (rough approximation)
    // Assuming BASE_SPEED 150 corresponds to ~10 cm/s (CALIBRATE THIS!)
    distanceTraveled += 10.0 * deltaTime;
  } else if (leftOnWhite && !rightOnWhite) {
    turnRightDegrees(5);
  } else if (!leftOnWhite && rightOnWhite) {
    turnLeftDegrees(5);
  } else {
    moveForward(SLOW_SPEED);
    distanceTraveled += 5.0 * deltaTime;
  }
  
  lastTime = currentTime;
  
  // Debug output
  if ((int)distanceTraveled % 2 == 0) { // Print every ~2cm
    Serial.print("Distance traveled: ");
    Serial.print(distanceTraveled);
    Serial.println(" cm");
  }
}

void pushBallToCenter() {
  Serial.println("=== PUSHING BALL TO CENTER ===");
  
  stopMotors();
  delay(500);
  
  // Turn 90 degrees to face the center (toward the black square)
  Serial.println("Turning toward center...");
  turnLeftDegrees(90);
  
  delay(300);
  
  // Move forward into the center to push the ball
  Serial.println("Moving to push ball...");
  moveForward(BASE_SPEED);
  delay(2000); // Move for 2 seconds (adjust based on testing)
  
  stopMotors();
  Serial.println("Ball should be pushed toward wall!");
  
  delay(500);
  
  // Back up
  Serial.println("Backing up...");
  moveBackward(BASE_SPEED);
  delay(1000);
  
  stopMotors();
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
    moveForward(BASE_SPEED);
  } else if (leftOnWhite && !rightOnWhite) {
    turnRightDegrees(5);
  } else if (!leftOnWhite && rightOnWhite) {
    turnLeftDegrees(5);
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
