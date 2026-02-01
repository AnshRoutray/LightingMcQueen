// ============================================
// PHASE 2: RETURN -> GREEN PATH -> CLIMB RAMP
// Starts: After picking up box at end of obstacle course
// Reupload Point: After reaching top of ramp
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
const int GREEN_PATH_DIRECTION = 1;  // 1=left, -1=right
const int PATH_ENTRY_ANGLE = 45;
const int PATH_ENTRY_FORWARD = 300;
const bool USE_CURVED_RAMP = true;  // true=curved (4pts), false=straight (2pts)

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

const unsigned long TIMEOUT_DROP = 5000;
const unsigned long TIMEOUT_CLIMB = 6000;
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

// ================== RAMP PARAMETERS ==================
const int RAMP_CURVE_DIFF = 30;
const int RAMP_ASCEND_TIME = 2500;
const int RAMP_TOP_READINGS = 3;

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
  STATE_RETURN_TO_MAIN,
  STATE_NAVIGATE_TO_GREEN,
  STATE_DROP_BOX_GREEN,
  STATE_CLIMB_RAMP,
  STATE_DONE
};

enum LineFollowState {
  LINE_FOLLOWING,
  LINE_LOST_SEARCHING,
  LINE_FOUND_TARGET
};

enum DropState {
  DROP_SEARCH_BLUE,
  DROP_CENTER_ON_BLUE,
  DROP_RELEASE,
  DROP_BACKUP,
  DROP_DONE
};

enum ClimbState {
  CLIMB_START,
  CLIMB_ASCENDING,
  CLIMB_DETECT_TOP,
  CLIMB_DONE
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
DropState dropState = DROP_SEARCH_BLUE;
ClimbState climbState = CLIMB_START;
SplitNavState splitState = SPLIT_FOLLOW_LINE;
LineFollowState lineState = LINE_FOLLOWING;

unsigned long stateStartTime = 0;
unsigned long totalStartTime = 0;
unsigned long lineLostTime = 0;

bool hasBox = true;  // Starting with box from phase 1

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
  
  clawServo.write(CLAW_CLOSED);  // Holding box from phase 1
  launcherServo.write(LAUNCHER_READY);
  
  if (DEBUG_STATE) {
    Serial.println(F("================================="));
    Serial.println(F("  PHASE 2: GREEN PATH + RAMP"));
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
// RAMP CLIMBING
// ============================================
bool climbRamp(bool curved) {
  static unsigned long climbTimer = 0;
  static int topReadings = 0;
  char color;
  
  switch (climbState) {
    case CLIMB_START:
      if (DEBUG_STATE) {
        Serial.print(F("Climb: Starting "));
        Serial.println(curved ? "CURVED" : "STRAIGHT");
      }
      climbTimer = millis();
      topReadings = 0;
      climbState = CLIMB_ASCENDING;
      break;
      
    case CLIMB_ASCENDING:
      if (curved) {
        setMotors(SPEED_FAST, SPEED_FAST - RAMP_CURVE_DIFF);
      } else {
        moveForward(SPEED_FAST);
      }
      
      if (millis() - climbTimer > RAMP_ASCEND_TIME) {
        climbState = CLIMB_DETECT_TOP;
      }
      break;
      
    case CLIMB_DETECT_TOP:
      moveForward(SPEED_SLOW);
      color = detectColor();
      
      if (color == 'B' || color == 'R' || color == 'G') {
        topReadings++;
        if (topReadings >= RAMP_TOP_READINGS) {
          stopMotors();
          if (DEBUG_STATE) Serial.println(F("Climb: Top reached!"));
          climbState = CLIMB_DONE;
        }
      } else {
        topReadings = 0;
      }
      
      if (millis() - climbTimer > TIMEOUT_CLIMB) {
        stopMotors();
        if (DEBUG_STATE) Serial.println(F("Climb: Timeout"));
        climbState = CLIMB_DONE;
      }
      break;
      
    case CLIMB_DONE:
      climbState = CLIMB_START;
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
      if (DEBUG_STATE) Serial.println(F("=== PHASE 2: START ==="));
      stateStartTime = millis();
      // Turn around to face back toward main line
      turnRightDegrees(180);
      currentState = STATE_RETURN_TO_MAIN;
      break;
      
    case STATE_RETURN_TO_MAIN:
      {
        char result = followBlackLine();
        // Look for main black line or green marker
        if (result == 'G' || result == 'K') {
          stopMotors();
          if (DEBUG_STATE) Serial.println(F("=== Back at main line ==="));
          currentState = STATE_NAVIGATE_TO_GREEN;
          stateStartTime = millis();
        }
      }
      break;
      
    case STATE_NAVIGATE_TO_GREEN:
      if (navigateToSplit('G', GREEN_PATH_DIRECTION)) {
        if (DEBUG_STATE) Serial.println(F("=== At green path ==="));
        currentState = STATE_DROP_BOX_GREEN;
        stateStartTime = millis();
      }
      break;
      
    case STATE_DROP_BOX_GREEN:
      if (dropBoxAtBlue()) {
        if (DEBUG_STATE) Serial.println(F("=== Green unlocked ==="));
        currentState = STATE_CLIMB_RAMP;
        stateStartTime = millis();
      }
      break;
      
    case STATE_CLIMB_RAMP:
      if (climbRamp(USE_CURVED_RAMP)) {
        if (DEBUG_STATE) Serial.println(F("=== PHASE 2 COMPLETE ==="));
        if (DEBUG_STATE) Serial.println(F("=== REUPLOAD POINT 2 ==="));
        currentState = STATE_DONE;
      }
      break;
      
    case STATE_DONE:
      stopMotors();
      // PHASE 2 COMPLETE - REUPLOAD HERE
      break;
  }
}
