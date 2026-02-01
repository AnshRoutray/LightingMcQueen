// ============================================
// PHASE 3: TARGET NAVIGATION + BALL LAUNCH
// Starts: At top of ramp
// Reupload Point: After shooting ball
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

// ================== MOTOR SPEEDS ==================
const int SPEED_CRAWL = 80;
const int SPEED_SLOW = 120;
const int SPEED_BASE = 150;
const int SPEED_TURN = 130;
const int ARC_DIFFERENTIAL = 40;

const float LEFT_MOTOR_MULT = 1.0;
const float RIGHT_MOTOR_MULT = 1.0;

// ================== TIMING CONSTANTS ==================
const int MS_PER_90_DEG = 500;
const int STEP_FORWARD_MS = 150;
const int STEP_BACKWARD_MS = 150;
const int STEP_INWARD_MS = 200;

const unsigned long TIMEOUT_NAV_ARC = 5000;
const unsigned long TIMEOUT_NAV_ENTRY = 5000;
const unsigned long TOTAL_TIME_LIMIT = 300000;

const int DELAY_AFTER_STOP = 50;
const int DELAY_AFTER_TURN = 50;
const int DELAY_SERVO_MOVE = 15;
const int DELAY_COLOR_READ = 5;
const int DELAY_STABILIZE = 200;

// ================== SERVO POSITIONS ==================
const int CLAW_OPEN = 10;
const int LAUNCHER_READY = 0;
const int LAUNCHER_FIRE = 150;

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

// ================== NAVIGATION ==================
const int NAV_SCAN_STEP_DEG = 30;
const int NAV_FULL_ROTATION = 360;

// ================== DEBUG ==================
const bool DEBUG_MOTORS = false;
const bool DEBUG_SENSORS = false;
const bool DEBUG_COLOR = true;
const bool DEBUG_STATE = true;
const bool DEBUG_NAV = true;

// ============================================
#include <Servo.h>

Servo clawServo;
Servo launcherServo;

// ---- STATE MACHINES ----
enum RobotState {
  STATE_START,
  STATE_NAVIGATE_TARGET,
  STATE_SHOOT_BALL,
  STATE_DONE
};

enum TargetNavState {
  NAV_INIT,
  NAV_FIND_WALL,
  NAV_IDENTIFY_ZONE,
  NAV_ARC_AROUND,
  NAV_STEP_INWARD,
  NAV_CORRECTION,
  NAV_PREPARE_ENTRY,
  NAV_ALIGN_TO_WALL,
  NAV_ENTER_BLACK,
  NAV_COMPLETE
};

enum LaunchState {
  LAUNCH_PREPARE,
  LAUNCH_FIRE,
  LAUNCH_RESET,
  LAUNCH_DONE
};

// ---- GLOBAL STATE ----
RobotState currentState = STATE_START;
TargetNavState navState = NAV_INIT;
LaunchState launchState = LAUNCH_PREPARE;

unsigned long stateStartTime = 0;
unsigned long totalStartTime = 0;

int wallDirection = -1;
bool wallFound = false;
char currentZone = 'X';
unsigned long arcStartTime = 0;
int arcDirection = 1;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(9600);
  
  initMotors();
  initUltrasonic();
  initColorSensor();
  initServos();
  
  clawServo.write(CLAW_OPEN);
  launcherServo.write(LAUNCHER_READY);
  
  if (DEBUG_STATE) {
    Serial.println(F("================================="));
    Serial.println(F("  PHASE 3: TARGET + LAUNCH"));
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

void moveArcClockwise(int speed) {
  setMotors(speed, speed - ARC_DIFFERENTIAL);
}

void moveArcCounterClockwise(int speed) {
  setMotors(speed - ARC_DIFFERENTIAL, speed);
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

// ============================================
// TARGET NAV HELPERS
// ============================================
bool isMoreInward(char detected, char current) {
  if (current == 'B' && detected == 'R') return true;
  if (current == 'R' && detected == 'G') return true;
  if (current == 'G' && detected == 'K') return true;
  return false;
}

bool isMoreOutward(char detected, char current) {
  if (current == 'R' && detected == 'B') return true;
  if (current == 'G' && detected == 'R') return true;
  if (current == 'K' && detected == 'G') return true;
  return false;
}

bool isBlackZone(char color) {
  return color == 'K';
}

void scanForWall() {
  int minDistance = DIST_NO_READING;
  int minAngle = 0;
  
  if (DEBUG_NAV) Serial.println(F("Nav: Scanning for wall..."));
  
  for (int angle = 0; angle < NAV_FULL_ROTATION; angle += NAV_SCAN_STEP_DEG) {
    long dist = readDistance();
    if (dist > 0 && dist < minDistance) {
      minDistance = dist;
      minAngle = angle;
    }
    turnRightDegrees(NAV_SCAN_STEP_DEG);
  }
  
  wallDirection = minAngle;
  wallFound = true;
  
  if (DEBUG_NAV) {
    Serial.print(F("Nav: Wall at ")); Serial.print(wallDirection);
    Serial.print(F("° dist:")); Serial.println(minDistance);
  }
}

void faceWall() {
  if (DEBUG_NAV) Serial.println(F("Nav: Facing wall..."));
  
  int minDist = DIST_NO_READING;
  int currentAngle = 0;
  int bestAngle = 0;
  
  for (int i = 0; i < (NAV_FULL_ROTATION / NAV_SCAN_STEP_DEG); i++) {
    long dist = readDistance();
    if (dist > 0 && dist < minDist) {
      minDist = dist;
      bestAngle = currentAngle;
    }
    turnRightDegrees(NAV_SCAN_STEP_DEG);
    currentAngle += NAV_SCAN_STEP_DEG;
  }
  
  if (bestAngle > 0) turnRightDegrees(bestAngle);
  
  if (DEBUG_NAV) {
    Serial.print(F("Nav: Now facing wall at ")); Serial.println(minDist);
  }
}

// ============================================
// TARGET NAVIGATION
// ============================================
bool navigateTargetByColor() {
  static unsigned long navTimer = 0;
  char detected;
  
  switch (navState) {
    case NAV_INIT:
      if (DEBUG_NAV) Serial.println(F("=== TARGET NAV START ==="));
      stopMotors();
      delay(300);
      wallFound = false;
      arcDirection = 1;
      navState = NAV_FIND_WALL;
      break;
      
    case NAV_FIND_WALL:
      if (!wallFound) scanForWall();
      navState = NAV_IDENTIFY_ZONE;
      break;
      
    case NAV_IDENTIFY_ZONE:
      detected = detectColorStable();
      currentZone = detected;
      
      if (DEBUG_NAV) { Serial.print(F("Nav: Zone = ")); Serial.println(currentZone); }
      
      if (currentZone == 'K') {
        if (DEBUG_NAV) Serial.println(F("Nav: WARNING on black!"));
        navState = NAV_CORRECTION;
      } else if (currentZone == 'G') {
        navState = NAV_PREPARE_ENTRY;
      } else if (currentZone == 'B' || currentZone == 'R') {
        arcStartTime = millis();
        navState = NAV_ARC_AROUND;
      } else {
        stepForward(200);
      }
      break;
      
    case NAV_ARC_AROUND:
      detected = detectColorStable();
      
      if (isBlackZone(detected)) {
        if (DEBUG_NAV) Serial.println(F("Nav: DANGER black detected!"));
        stopMotors();
        navState = NAV_CORRECTION;
        break;
      }
      
      if (isMoreInward(detected, currentZone)) {
        stopMotors();
        if (DEBUG_NAV) {
          Serial.print(F("Nav: Transition ")); Serial.print(currentZone);
          Serial.print(F(" -> ")); Serial.println(detected);
        }
        currentZone = detected;
        navState = NAV_STEP_INWARD;
        break;
      }
      
      if (isMoreOutward(detected, currentZone)) {
        stopMotors();
        if (DEBUG_NAV) Serial.println(F("Nav: Drifted out, correcting"));
        stepForward(100);
        detected = detectColorStable();
        currentZone = detected;
        break;
      }
      
      if (arcDirection > 0) moveArcClockwise(SPEED_BASE);
      else moveArcCounterClockwise(SPEED_BASE);
      
      if (millis() - arcStartTime > TIMEOUT_NAV_ARC) {
        stopMotors();
        arcDirection *= -1;
        arcStartTime = millis();
        if (DEBUG_NAV) Serial.println(F("Nav: Arc timeout, reversing"));
      }
      break;
      
    case NAV_STEP_INWARD:
      if (DEBUG_NAV) { Serial.print(F("Nav: Stepping to ")); Serial.println(currentZone); }
      stepForward(STEP_INWARD_MS);
      detected = detectColorStable();
      
      if (isBlackZone(detected)) {
        if (DEBUG_NAV) Serial.println(F("Nav: Overshot to black!"));
        navState = NAV_CORRECTION;
        break;
      }
      
      currentZone = detected;
      if (currentZone == 'G') navState = NAV_PREPARE_ENTRY;
      else { arcStartTime = millis(); navState = NAV_ARC_AROUND; }
      break;
      
    case NAV_CORRECTION:
      if (DEBUG_NAV) Serial.println(F("Nav: CORRECTING"));
      stepBackward(STEP_BACKWARD_MS);
      detected = detectColorStable();
      
      if (!isBlackZone(detected)) {
        currentZone = detected;
        if (DEBUG_NAV) { Serial.print(F("Nav: Corrected to ")); Serial.println(currentZone); }
        if (currentZone == 'G') navState = NAV_PREPARE_ENTRY;
        else { arcStartTime = millis(); navState = NAV_ARC_AROUND; }
      }
      break;
      
    case NAV_PREPARE_ENTRY:
      if (DEBUG_NAV) Serial.println(F("Nav: On GREEN, preparing entry"));
      stopMotors();
      delay(200);
      navState = NAV_ALIGN_TO_WALL;
      navTimer = millis();
      break;
      
    case NAV_ALIGN_TO_WALL:
      if (DEBUG_NAV) Serial.println(F("Nav: Aligning to wall"));
      faceWall();
      delay(200);
      detected = detectColorStable();
      if (isBlackZone(detected)) { navState = NAV_CORRECTION; break; }
      navState = NAV_ENTER_BLACK;
      navTimer = millis();
      break;
      
    case NAV_ENTER_BLACK:
      detected = detectColorStable();
      if (isBlackZone(detected)) {
        stopMotors();
        stepForward(150);
        if (DEBUG_NAV) Serial.println(F("=== NAV: CENTERED! ==="));
        navState = NAV_COMPLETE;
        break;
      }
      moveForward(SPEED_CRAWL);
      delay(80);
      stopMotors();
      delay(30);
      if (millis() - navTimer > TIMEOUT_NAV_ENTRY) {
        if (DEBUG_NAV) Serial.println(F("Nav: Entry timeout"));
        navState = NAV_COMPLETE;
      }
      break;
      
    case NAV_COMPLETE:
      stopMotors();
      navState = NAV_INIT;
      return true;
  }
  return false;
}

// ============================================
// BALL LAUNCHER
// ============================================
bool launchBall() {
  static unsigned long launchTimer = 0;
  
  switch (launchState) {
    case LAUNCH_PREPARE:
      stopMotors();
      if (DEBUG_STATE) Serial.println(F("Launch: Aiming"));
      delay(200);
      launchTimer = millis();
      launchState = LAUNCH_FIRE;
      break;
      
    case LAUNCH_FIRE:
      if (DEBUG_STATE) Serial.println(F("Launch: FIRE!"));
      launcherServo.write(LAUNCHER_FIRE);
      if (millis() - launchTimer > 500) {
        launchState = LAUNCH_RESET;
        launchTimer = millis();
      }
      break;
      
    case LAUNCH_RESET:
      launcherServo.write(LAUNCHER_READY);
      if (millis() - launchTimer > 300) launchState = LAUNCH_DONE;
      break;
      
    case LAUNCH_DONE:
      if (DEBUG_STATE) Serial.println(F("Launch: Complete!"));
      launchState = LAUNCH_PREPARE;
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
      if (DEBUG_STATE) Serial.println(F("=== PHASE 3: START ==="));
      stateStartTime = millis();
      currentState = STATE_NAVIGATE_TARGET;
      navState = NAV_INIT;
      break;
      
    case STATE_NAVIGATE_TARGET:
      if (navigateTargetByColor()) {
        if (DEBUG_STATE) Serial.println(F("=== At center ==="));
        currentState = STATE_SHOOT_BALL;
        stateStartTime = millis();
      }
      break;
      
    case STATE_SHOOT_BALL:
      if (launchBall()) {
        if (DEBUG_STATE) Serial.println(F("=== PHASE 3 COMPLETE ==="));
        if (DEBUG_STATE) Serial.println(F("=== REUPLOAD POINT 3 ==="));
        currentState = STATE_DONE;
      }
      break;
      
    case STATE_DONE:
      stopMotors();
      // PHASE 3 COMPLETE - REUPLOAD HERE
      break;
  }
}
