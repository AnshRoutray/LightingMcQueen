// ============================================
// PHASE 1: START -> RED PATH -> OBSTACLE COURSE
// Reupload Point: After picking up box at end of obstacle course
// ============================================

// ================== PIN DEFINITIONS ==================
#define PIN_ENA             5
#define PIN_IN1             6
#define PIN_IN2             7
#define PIN_IN3             8
#define PIN_IN4             9
#define PIN_ENB             10

#define PIN_TRIG            11
#define PIN_ECHO            12

#define PIN_IR_LEFT         A0
#define PIN_IR_RIGHT        A1

#define PIN_COLOR_S0        2
#define PIN_COLOR_S1        3
#define PIN_COLOR_S2        4
#define PIN_COLOR_S3        A2
#define PIN_COLOR_OUT       A3

#define PIN_CLAW_SERVO      A4
#define PIN_LAUNCHER_SERVO  A5

// ================== PATH CONFIGURATION ==================
const int RED_PATH_DIRECTION = -1;  // 1=left, -1=right
const int PATH_ENTRY_ANGLE = 45;
const int PATH_ENTRY_FORWARD = 300;

// ================== MOTOR SPEEDS ==================
const int SPEED_CRAWL = 80;
const int SPEED_SLOW = 120;
const int SPEED_BASE = 150;
const int SPEED_FAST = 200;
const int SPEED_TURN = 130;

const float LEFT_MOTOR_MULT = 1.0;
const float RIGHT_MOTOR_MULT = 1.0;

// ================== TIMING CONSTANTS ==================
const int MS_PER_90_DEG = 500;
const int STEP_FORWARD_MS = 150;
const int STEP_BACKWARD_MS = 150;

const unsigned long TIMEOUT_PICKUP = 5000;
const unsigned long TIMEOUT_DROP = 5000;
const unsigned long TIMEOUT_OBSTACLE = 30000;
const unsigned long TIMEOUT_LINE_LOST = 2000;
const unsigned long TOTAL_TIME_LIMIT = 300000;

const int DELAY_AFTER_STOP = 50;
const int DELAY_AFTER_TURN = 50;
const int DELAY_SERVO_MOVE = 15;
const int DELAY_COLOR_READ = 5;
const int DELAY_STABILIZE = 200;

// ================== SERVO POSITIONS ==================
const int CLAW_OPEN = 10;
const int CLAW_CLOSED = 80;
const int LAUNCHER_READY = 0;

// ================== DISTANCE THRESHOLDS ==================
const int DIST_BOX_PICKUP = 8;
const int DIST_BOX_DETECT = 25;
const int DIST_BOX_SEARCH = 40;
const int DIST_OBSTACLE = 15;
const int DIST_NO_READING = 999;

// ================== COLOR SENSOR ==================
const bool COLOR_S0_STATE = HIGH;
const bool COLOR_S1_STATE = LOW;
const int COLOR_SAMPLES = 5;
const int THRESH_BLACK_R = 50;
const int THRESH_BLACK_G = 50;
const int THRESH_BLACK_B = 50;
const int COLOR_DIFF_MIN = 20;
const int PURPLE_RB_DIFF = 30;
const unsigned long COLOR_PULSE_TIMEOUT = 10000;

const bool IR_ACTIVE_LOW = true;

// ================== NAVIGATION ==================
const int AVOID_TURN_DEG = 45;
const int AVOID_FORWARD_MS = 400;
const int SHARP_TURN_SCAN = 45;

// ================== DEBUG ==================
const bool DEBUG_MOTORS = false;
const bool DEBUG_SENSORS = false;
const bool DEBUG_COLOR = true;
const bool DEBUG_STATE = true;
const bool DEBUG_NAV = true;
const bool DEBUG_LINE = true;

// ============================================
#include <Servo.h>

Servo clawServo;
Servo launcherServo;

// ---- STATE MACHINES ----
enum RobotState {
  STATE_START,
  STATE_FOLLOW_TO_JUNCTION,
  STATE_PICKUP_BOX_1,
  STATE_NAVIGATE_TO_RED,
  STATE_DROP_BOX_RED,
  STATE_OBSTACLE_COURSE,
  STATE_PICKUP_BOX_END,
  STATE_DONE
};

enum LineFollowState {
  LINE_FOLLOWING,
  LINE_LOST_SEARCHING,
  LINE_FOUND_TARGET
};

enum PickupState {
  PICKUP_INIT,
  PICKUP_SCAN_FOR_BOX,
  PICKUP_TURN_TO_BOX,
  PICKUP_APPROACH,
  PICKUP_FINAL_APPROACH,
  PICKUP_GRAB,
  PICKUP_VERIFY,
  PICKUP_DONE
};

enum DropState {
  DROP_SEARCH_BLUE,
  DROP_CENTER_ON_BLUE,
  DROP_RELEASE,
  DROP_BACKUP,
  DROP_DONE
};

enum ObstacleState {
  OBS_START,
  OBS_FOLLOW_PATH,
  OBS_AVOID_OBSTACLE,
  OBS_SHARP_TURN,
  OBS_CHECK_COMPLETE,
  OBS_DONE
};

enum SplitNavState {
  SPLIT_FOLLOW_LINE,
  SPLIT_DETECT_COLOR,
  SPLIT_TURN_TO_PATH,
  SPLIT_ENTER_PATH,
  SPLIT_DONE
};

// ---- GLOBAL STATE ----
RobotState currentState = STATE_START;
PickupState pickupState = PICKUP_INIT;
DropState dropState = DROP_SEARCH_BLUE;
ObstacleState obsState = OBS_START;
SplitNavState splitState = SPLIT_FOLLOW_LINE;
LineFollowState lineState = LINE_FOLLOWING;

unsigned long stateStartTime = 0;
unsigned long totalStartTime = 0;
unsigned long lineLostTime = 0;

bool hasBox = false;
int obstacleCount = 0;
int avoidDirection = 1;
int boxDirection = 0;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(9600);
  
  initMotors();
  initUltrasonic();
  initIRSensors();
  initColorSensor();
  initServos();
  
  clawServo.write(CLAW_OPEN);
  launcherServo.write(LAUNCHER_READY);
  
  if (DEBUG_STATE) {
    Serial.println(F("================================="));
    Serial.println(F("  PHASE 1: OBSTACLE COURSE"));
    Serial.println(F("================================="));
    Serial.println(F("Starting in 3 seconds..."));
  }
  
  delay(3000);
  
  totalStartTime = millis();
  stateStartTime = millis();
}

// ============================================
// INITIALIZATION
// ============================================
void initMotors() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  stopMotors();
}

void initUltrasonic() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}

void initIRSensors() {
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
}

void initColorSensor() {
  pinMode(PIN_COLOR_S0, OUTPUT);
  pinMode(PIN_COLOR_S1, OUTPUT);
  pinMode(PIN_COLOR_S2, OUTPUT);
  pinMode(PIN_COLOR_S3, OUTPUT);
  pinMode(PIN_COLOR_OUT, INPUT);
  digitalWrite(PIN_COLOR_S0, COLOR_S0_STATE);
  digitalWrite(PIN_COLOR_S1, COLOR_S1_STATE);
}

void initServos() {
  clawServo.attach(PIN_CLAW_SERVO);
  launcherServo.attach(PIN_LAUNCHER_SERVO);
}

// ============================================
// MOTOR CONTROL
// ============================================
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = (int)(leftSpeed * LEFT_MOTOR_MULT);
  rightSpeed = (int)(rightSpeed * RIGHT_MOTOR_MULT);
  
  if (leftSpeed >= 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PIN_ENA, constrain(leftSpeed, 0, 255));
  
  if (rightSpeed >= 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PIN_ENB, constrain(rightSpeed, 0, 255));
}

void moveForward(int speed) { setMotors(speed, speed); }
void moveBackward(int speed) { setMotors(-speed, -speed); }
void turnLeft(int speed) { setMotors(-speed, speed); }
void turnRight(int speed) { setMotors(speed, -speed); }

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

void turnByDirection(int direction, int speed) {
  if (direction > 0) turnRight(speed);
  else turnLeft(speed);
}

void turnDegrees(int degrees, int direction) {
  int duration = (abs(degrees) * MS_PER_90_DEG) / 90;
  turnByDirection(direction, SPEED_TURN);
  delay(duration);
  stopMotors();
  delay(DELAY_AFTER_TURN);
}

void turnLeftDegrees(int degrees) { turnDegrees(degrees, -1); }
void turnRightDegrees(int degrees) { turnDegrees(degrees, 1); }

void stepForward(int duration) {
  moveForward(SPEED_SLOW);
  delay(duration);
  stopMotors();
  delay(DELAY_AFTER_STOP);
}

void stepBackward(int duration) {
  moveBackward(SPEED_SLOW);
  delay(duration);
  stopMotors();
  delay(DELAY_AFTER_STOP);
}

// ============================================
// SENSOR READING
// ============================================
long readDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duration == 0) return DIST_NO_READING;
  return duration * 0.034 / 2;
}

int readColorFrequency(char color) {
  switch (color) {
    case 'R': digitalWrite(PIN_COLOR_S2, LOW); digitalWrite(PIN_COLOR_S3, LOW); break;
    case 'G': digitalWrite(PIN_COLOR_S2, HIGH); digitalWrite(PIN_COLOR_S3, HIGH); break;
    case 'B': digitalWrite(PIN_COLOR_S2, LOW); digitalWrite(PIN_COLOR_S3, HIGH); break;
  }
  delayMicroseconds(100);
  return pulseIn(PIN_COLOR_OUT, LOW, COLOR_PULSE_TIMEOUT);
}

char detectColor() {
  int r = readColorFrequency('R');
  int g = readColorFrequency('G');
  int b = readColorFrequency('B');
  
  if (r == 0 && g == 0 && b == 0) return 'X';
  if (r < THRESH_BLACK_R && g < THRESH_BLACK_G && b < THRESH_BLACK_B) return 'K';
  if (r < g && b < g && abs(r - b) < PURPLE_RB_DIFF) return 'P';
  if (r < g - COLOR_DIFF_MIN && r < b - COLOR_DIFF_MIN) return 'R';
  if (g < r - COLOR_DIFF_MIN && g < b - COLOR_DIFF_MIN) return 'G';
  if (b < r - COLOR_DIFF_MIN && b < g - COLOR_DIFF_MIN) return 'B';
  if (r <= g && r <= b) return 'R';
  if (g <= r && g <= b) return 'G';
  return 'B';
}

char detectColorStable() {
  int rSum = 0, gSum = 0, bSum = 0;
  for (int i = 0; i < COLOR_SAMPLES; i++) {
    rSum += readColorFrequency('R');
    gSum += readColorFrequency('G');
    bSum += readColorFrequency('B');
    delay(DELAY_COLOR_READ);
  }
  
  int r = rSum / COLOR_SAMPLES;
  int g = gSum / COLOR_SAMPLES;
  int b = bSum / COLOR_SAMPLES;
  
  if (DEBUG_COLOR) {
    Serial.print(F("RGB: ")); Serial.print(r); Serial.print(F(","));
    Serial.print(g); Serial.print(F(",")); Serial.println(b);
  }
  
  if (r == 0 && g == 0 && b == 0) return 'X';
  if (r < THRESH_BLACK_R && g < THRESH_BLACK_G && b < THRESH_BLACK_B) return 'K';
  if (r < g && b < g && abs(r - b) < PURPLE_RB_DIFF) return 'P';
  if (r < g - COLOR_DIFF_MIN && r < b - COLOR_DIFF_MIN) return 'R';
  if (g < r - COLOR_DIFF_MIN && g < b - COLOR_DIFF_MIN) return 'G';
  if (b < r - COLOR_DIFF_MIN && b < g - COLOR_DIFF_MIN) return 'B';
  if (r <= g && r <= b) return 'R';
  if (g <= r && g <= b) return 'G';
  return 'B';
}

bool irLeft() {
  bool reading = digitalRead(PIN_IR_LEFT);
  return IR_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);
}

bool irRight() {
  bool reading = digitalRead(PIN_IR_RIGHT);
  return IR_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);
}

// ============================================
// CLAW CONTROL
// ============================================
void openClaw() {
  int current = clawServo.read();
  if (current > CLAW_OPEN) {
    for (int pos = current; pos > CLAW_OPEN; pos -= 2) {
      clawServo.write(pos);
      delay(DELAY_SERVO_MOVE);
    }
  }
  clawServo.write(CLAW_OPEN);
  delay(DELAY_STABILIZE);
}

void closeClaw() {
  int current = clawServo.read();
  if (current < CLAW_CLOSED) {
    for (int pos = current; pos < CLAW_CLOSED; pos += 2) {
      clawServo.write(pos);
      delay(DELAY_SERVO_MOVE);
    }
  }
  clawServo.write(CLAW_CLOSED);
  delay(DELAY_STABILIZE);
}

// ============================================
// LINE FOLLOWING
// ============================================
char followBlackLine() {
  bool left = irLeft();
  bool right = irRight();
  char color = detectColor();
  
  if (color == 'G' || color == 'R' || color == 'B' || color == 'P') {
    stopMotors();
    if (DEBUG_LINE) { Serial.print(F("Line: Color marker: ")); Serial.println(color); }
    return color;
  }
  
  if (left && right) {
    moveForward(SPEED_BASE);
    lineState = LINE_FOLLOWING;
    return 'F';
  } else if (left && !right) {
    setMotors(SPEED_SLOW, SPEED_FAST);
    lineState = LINE_FOLLOWING;
    return 'F';
  } else if (!left && right) {
    setMotors(SPEED_FAST, SPEED_SLOW);
    lineState = LINE_FOLLOWING;
    return 'F';
  } else {
    if (lineState != LINE_LOST_SEARCHING) {
      lineLostTime = millis();
      lineState = LINE_LOST_SEARCHING;
      if (DEBUG_LINE) Serial.println(F("Line: LOST"));
    }
    if (millis() - lineLostTime < TIMEOUT_LINE_LOST) {
      moveForward(SPEED_CRAWL);
      return 'F';
    } else {
      stopMotors();
      return 'L';
    }
  }
}

void searchForLine() {
  if (DEBUG_LINE) Serial.println(F("Searching for line..."));
  
  for (int i = 0; i < 3; i++) {
    turnLeftDegrees(15);
    if (irLeft() || irRight()) { if (DEBUG_LINE) Serial.println(F("Line found LEFT")); return; }
  }
  turnRightDegrees(45);
  for (int i = 0; i < 3; i++) {
    turnRightDegrees(15);
    if (irLeft() || irRight()) { if (DEBUG_LINE) Serial.println(F("Line found RIGHT")); return; }
  }
  turnLeftDegrees(45);
  stepForward(200);
}

// ============================================
// BOX SCANNING
// ============================================
int scanForBox() {
  stopMotors();
  delay(100);
  
  long distFront = readDistance();
  
  turnLeftDegrees(45);
  delay(100);
  long distLeft = readDistance();
  turnRightDegrees(45);
  
  turnRightDegrees(45);
  delay(100);
  long distRight = readDistance();
  turnLeftDegrees(45);
  
  if (DEBUG_NAV) {
    Serial.print(F("Box scan - L:")); Serial.print(distLeft);
    Serial.print(F(" F:")); Serial.print(distFront);
    Serial.print(F(" R:")); Serial.println(distRight);
  }
  
  if (distLeft < DIST_BOX_SEARCH && distLeft < distRight && distLeft < distFront) return -1;
  else if (distRight < DIST_BOX_SEARCH && distRight < distLeft && distRight < distFront) return 1;
  else if (distFront < DIST_BOX_SEARCH) return 0;
  return 0;
}

// ============================================
// BOX PICKUP
// ============================================
bool pickupBox() {
  static unsigned long pickupTimer = 0;
  static int scanAttempts = 0;
  long dist;
  
  switch (pickupState) {
    case PICKUP_INIT:
      if (DEBUG_STATE) Serial.println(F("Pickup: Starting"));
      openClaw();
      scanAttempts = 0;
      boxDirection = 0;
      pickupState = PICKUP_SCAN_FOR_BOX;
      pickupTimer = millis();
      break;
      
    case PICKUP_SCAN_FOR_BOX:
      boxDirection = scanForBox();
      if (boxDirection != 0 || scanAttempts > 2) {
        if (boxDirection != 0) {
          if (DEBUG_STATE) { Serial.print(F("Pickup: Box found ")); Serial.println(boxDirection > 0 ? "RIGHT" : "LEFT"); }
          pickupState = PICKUP_TURN_TO_BOX;
        } else {
          dist = readDistance();
          if (dist < DIST_BOX_DETECT) {
            if (DEBUG_STATE) Serial.println(F("Pickup: Box in front"));
            pickupState = PICKUP_APPROACH;
          } else {
            stepForward(200);
            scanAttempts++;
          }
        }
      }
      if (millis() - pickupTimer > TIMEOUT_PICKUP) {
        if (DEBUG_STATE) Serial.println(F("Pickup: Timeout"));
        pickupState = PICKUP_DONE;
      }
      break;
      
    case PICKUP_TURN_TO_BOX:
      turnDegrees(60, boxDirection);
      pickupState = PICKUP_APPROACH;
      break;
      
    case PICKUP_APPROACH:
      dist = readDistance();
      if (dist <= DIST_BOX_DETECT && dist > DIST_BOX_PICKUP) {
        moveForward(SPEED_SLOW);
      } else if (dist <= DIST_BOX_PICKUP) {
        stopMotors();
        pickupState = PICKUP_FINAL_APPROACH;
        pickupTimer = millis();
      } else {
        moveForward(SPEED_CRAWL);
        if (millis() - pickupTimer > 3000) {
          pickupState = PICKUP_SCAN_FOR_BOX;
          scanAttempts++;
        }
      }
      break;
      
    case PICKUP_FINAL_APPROACH:
      stepForward(100);
      pickupState = PICKUP_GRAB;
      break;
      
    case PICKUP_GRAB:
      stopMotors();
      closeClaw();
      pickupTimer = millis();
      pickupState = PICKUP_VERIFY;
      break;
      
    case PICKUP_VERIFY:
      if (millis() - pickupTimer > 300) {
        stepBackward(100);
        if (boxDirection != 0) turnDegrees(60, -boxDirection);
        hasBox = true;
        pickupState = PICKUP_DONE;
      }
      break;
      
    case PICKUP_DONE:
      if (DEBUG_STATE) Serial.println(F("Pickup: Complete"));
      pickupState = PICKUP_INIT;
      return true;
  }
  return false;
}

// ============================================
// NAVIGATE TO SPLIT
// ============================================
bool navigateToSplit(char targetColor, int pathDirection) {
  static unsigned long splitTimer = 0;
  char detected;
  
  switch (splitState) {
    case SPLIT_FOLLOW_LINE:
      detected = followBlackLine();
      if (detected == targetColor || detected == 'B') {
        stopMotors();
        if (DEBUG_STATE) { Serial.print(F("Split: Found ")); Serial.println(detected); }
        if (detected == 'B') splitState = SPLIT_DONE;
        else splitState = SPLIT_DETECT_COLOR;
      }
      if (detected == 'L') { stopMotors(); searchForLine(); }
      break;
      
    case SPLIT_DETECT_COLOR:
      detected = detectColorStable();
      if (detected == targetColor) splitState = SPLIT_TURN_TO_PATH;
      else splitState = SPLIT_FOLLOW_LINE;
      break;
      
    case SPLIT_TURN_TO_PATH:
      if (DEBUG_STATE) { Serial.print(F("Split: Turning ")); Serial.println(pathDirection > 0 ? "RIGHT" : "LEFT"); }
      turnDegrees(PATH_ENTRY_ANGLE, pathDirection);
      splitTimer = millis();
      splitState = SPLIT_ENTER_PATH;
      break;
      
    case SPLIT_ENTER_PATH:
      moveForward(SPEED_BASE);
      if (millis() - splitTimer > PATH_ENTRY_FORWARD) {
        stopMotors();
        splitState = SPLIT_DONE;
      }
      break;
      
    case SPLIT_DONE:
      splitState = SPLIT_FOLLOW_LINE;
      return true;
  }
  return false;
}

// ============================================
// DROP BOX AT BLUE
// ============================================
bool dropBoxAtBlue() {
  static unsigned long dropTimer = 0;
  char color;
  
  switch (dropState) {
    case DROP_SEARCH_BLUE:
      color = detectColor();
      if (color == 'B') {
        stopMotors();
        if (DEBUG_STATE) Serial.println(F("Drop: Blue found"));
        dropState = DROP_CENTER_ON_BLUE;
        dropTimer = millis();
      } else {
        moveForward(SPEED_SLOW);
        if (millis() - stateStartTime > TIMEOUT_DROP) {
          if (DEBUG_STATE) Serial.println(F("Drop: Timeout, dropping anyway"));
          stopMotors();
          dropState = DROP_RELEASE;
        }
      }
      break;
      
    case DROP_CENTER_ON_BLUE:
      moveForward(SPEED_CRAWL);
      if (millis() - dropTimer > 150) {
        stopMotors();
        dropState = DROP_RELEASE;
      }
      break;
      
    case DROP_RELEASE:
      if (DEBUG_STATE) Serial.println(F("Drop: Releasing"));
      openClaw();
      hasBox = false;
      dropTimer = millis();
      dropState = DROP_BACKUP;
      break;
      
    case DROP_BACKUP:
      moveBackward(SPEED_BASE);
      if (millis() - dropTimer > 400) {
        stopMotors();
        dropState = DROP_DONE;
      }
      break;
      
    case DROP_DONE:
      if (DEBUG_STATE) Serial.println(F("Drop: Complete"));
      dropState = DROP_SEARCH_BLUE;
      return true;
  }
  return false;
}

// ============================================
// OBSTACLE COURSE
// ============================================
bool navigateObstacleCourse() {
  static unsigned long obsTimer = 0;
  long dist;
  char color;
  bool left, right;
  
  switch (obsState) {
    case OBS_START:
      if (DEBUG_STATE) Serial.println(F("Obstacle: Starting"));
      obsTimer = millis();
      obstacleCount = 0;
      avoidDirection = 1;
      obsState = OBS_FOLLOW_PATH;
      break;
      
    case OBS_FOLLOW_PATH:
      dist = readDistance();
      if (dist > 0 && dist < DIST_OBSTACLE) {
        stopMotors();
        if (DEBUG_STATE) { Serial.print(F("Obstacle: Detected at ")); Serial.println(dist); }
        obsState = OBS_AVOID_OBSTACLE;
        break;
      }
      
      left = irLeft();
      right = irRight();
      
      if (!left && !right) {
        stopMotors();
        obsState = OBS_SHARP_TURN;
        break;
      }
      
      if (left && right) moveForward(SPEED_FAST);
      else if (left && !right) setMotors(SPEED_SLOW, SPEED_FAST);
      else setMotors(SPEED_FAST, SPEED_SLOW);
      
      color = detectColor();
      if (color == 'K' && millis() - obsTimer > 5000) obsState = OBS_CHECK_COMPLETE;
      if (millis() - obsTimer > TIMEOUT_OBSTACLE) obsState = OBS_DONE;
      break;
      
    case OBS_AVOID_OBSTACLE: // good
      obstacleCount++;
      stepBackward(150);
      turnDegrees(AVOID_TURN_DEG, avoidDirection);
      stepForward(AVOID_FORWARD_MS);
      turnDegrees(AVOID_TURN_DEG, -avoidDirection);
      avoidDirection *= -1;
      obsState = OBS_FOLLOW_PATH;
      break;
      
    case OBS_SHARP_TURN:
      if (DEBUG_STATE) Serial.println(F("Obstacle: Sharp turn"));
      turnLeftDegrees(SHARP_TURN_SCAN);
      if (irLeft() || irRight()) { obsState = OBS_FOLLOW_PATH; break; }
      turnRightDegrees(SHARP_TURN_SCAN * 2);
      if (irLeft() || irRight()) { obsState = OBS_FOLLOW_PATH; break; }
      turnLeftDegrees(SHARP_TURN_SCAN);
      stepForward(200);
      obsState = OBS_FOLLOW_PATH;
      break;
      
    case OBS_CHECK_COMPLETE:
      stopMotors();
      color = detectColorStable();
      if (color == 'K') {
        if (DEBUG_STATE) { Serial.println(F("Obstacle: Complete!")); Serial.print(F("Avoided: ")); Serial.println(obstacleCount); }
        obsState = OBS_DONE;
      } else obsState = OBS_FOLLOW_PATH;
      break;
      
    case OBS_DONE:
      obsState = OBS_START;
      return true;
  }
  return false;
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  if (millis() - totalStartTime > TOTAL_TIME_LIMIT) {
    if (DEBUG_STATE) Serial.println(F("!!! TIME LIMIT !!!"));
    currentState = STATE_DONE;
  }
  
  switch (currentState) {
    case STATE_START:
      if (DEBUG_STATE) Serial.println(F("=== PHASE 1: START ==="));
      stateStartTime = millis();
      currentState = STATE_FOLLOW_TO_JUNCTION;
      break;
      
    case STATE_FOLLOW_TO_JUNCTION:
      {
        char result = followBlackLine();
        if (result == 'B') { // wrong
          stopMotors();
          if (DEBUG_STATE) Serial.println(F("=== At junction ==="));
          currentState = STATE_PICKUP_BOX_1;
          stateStartTime = millis();
        }
      }
      break;
      
    case STATE_PICKUP_BOX_1:
      if (pickupBox()) {
        if (DEBUG_STATE) Serial.println(F("=== Box 1 acquired ==="));
        currentState = STATE_NAVIGATE_TO_RED;
        stateStartTime = millis();
      }
      break;
      
    case STATE_NAVIGATE_TO_RED:
      if (navigateToSplit('R', RED_PATH_DIRECTION)) { // red path is wrong
        if (DEBUG_STATE) Serial.println(F("=== At red path ==="));
        currentState = STATE_DROP_BOX_RED;
        stateStartTime = millis();
      }
      break;
      
    case STATE_DROP_BOX_RED:
      if (dropBoxAtBlue()) {
        if (DEBUG_STATE) Serial.println(F("=== Red unlocked ==="));
        currentState = STATE_OBSTACLE_COURSE;
        stateStartTime = millis();
      }
      break;
      
    case STATE_OBSTACLE_COURSE:
      if (navigateObstacleCourse()) {
        if (DEBUG_STATE) Serial.println(F("=== Obstacle complete ==="));
        currentState = STATE_PICKUP_BOX_END;
        stateStartTime = millis();
      }
      break;
      
    case STATE_PICKUP_BOX_END:
      if (pickupBox()) {
        if (DEBUG_STATE) Serial.println(F("=== PHASE 1 COMPLETE ==="));
        if (DEBUG_STATE) Serial.println(F("=== REUPLOAD POINT 1 ==="));
        currentState = STATE_DONE;
      }
      break;
      
    case STATE_DONE:
      stopMotors();
      // PHASE 1 COMPLETE - REUPLOAD HERE
      break;
  }
}
