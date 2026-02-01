// ============================================
// PHASE 4: DESCEND RAMP + RETURN HOME
// Starts: After shooting ball at target center
// End: Back at start position
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

const float LEFT_MOTOR_MULT = 1.0;
const float RIGHT_MOTOR_MULT = 1.0;

// ================== TIMING CONSTANTS ==================
const int MS_PER_90_DEG = 500;
const int STEP_FORWARD_MS = 150;

const unsigned long TIMEOUT_RETURN = 15000;
const unsigned long TOTAL_TIME_LIMIT = 300000;

const int DELAY_AFTER_STOP = 50;
const int DELAY_AFTER_TURN = 50;
const int DELAY_SERVO_MOVE = 15;
const int DELAY_COLOR_READ = 5;
const int DELAY_STABILIZE = 200;

// ================== SERVO POSITIONS ==================
const int CLAW_OPEN = 10;
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
const int RAMP_DESCEND_TIME = 2000;

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
  STATE_DESCEND_RAMP,
  STATE_RETURN_HOME,
  STATE_DONE
};

enum DescendState {
  DESCEND_TURN_AROUND,
  DESCEND_GO_DOWN,
  DESCEND_DETECT_BOTTOM,
  DESCEND_DONE
};

enum HomeState {
  HOME_FIND_LINE,
  HOME_FOLLOW,
  HOME_DETECT_START,
  HOME_DONE
};

// ---- GLOBAL STATE ----
RobotState currentState = STATE_START;
DescendState descendState = DESCEND_TURN_AROUND;
HomeState homeState = HOME_FIND_LINE;

unsigned long stateStartTime = 0;
unsigned long totalStartTime = 0;

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
    Serial.println(F("  PHASE 4: DESCEND + HOME"));
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

bool irLeft() {
  bool reading = digitalRead(PIN_IR_LEFT);
  return IR_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);
}

bool irRight() {
  bool reading = digitalRead(PIN_IR_RIGHT);
  return IR_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);
}

// ============================================
// RAMP DESCENT
// ============================================
bool descendRamp() {
  static unsigned long descendTimer = 0;
  char color;
  
  switch (descendState) {
    case DESCEND_TURN_AROUND:
      if (DEBUG_STATE) Serial.println(F("Descend: Turning around"));
      turnRightDegrees(180);
      descendTimer = millis();
      descendState = DESCEND_GO_DOWN;
      break;
      
    case DESCEND_GO_DOWN:
      moveForward(SPEED_SLOW);
      if (millis() - descendTimer > RAMP_DESCEND_TIME) {
        descendState = DESCEND_DETECT_BOTTOM;
      }
      break;
      
    case DESCEND_DETECT_BOTTOM:
      moveForward(SPEED_SLOW);
      color = detectColor();
      
      if (color == 'G' || color == 'K') {
        stopMotors();
        if (DEBUG_STATE) Serial.println(F("Descend: Bottom reached"));
        descendState = DESCEND_DONE;
      }
      
      if (millis() - descendTimer > 5000) {
        stopMotors();
        descendState = DESCEND_DONE;
      }
      break;
      
    case DESCEND_DONE:
      descendState = DESCEND_TURN_AROUND;
      return true;
  }
  return false;
}

// ============================================
// RETURN HOME
// ============================================
bool returnHome() {
  static unsigned long homeTimer = 0;
  char color;
  bool left, right;
  long dist;
  
  switch (homeState) {
    case HOME_FIND_LINE:
      if (DEBUG_STATE) Serial.println(F("Home: Finding line"));
      color = detectColor();
      if (color == 'K') {
        homeState = HOME_FOLLOW;
        homeTimer = millis();
      } else {
        turnRight(SPEED_SLOW);
        delay(100);
        stopMotors();
      }
      break;
      
    case HOME_FOLLOW:
      left = irLeft();
      right = irRight();
      
      if (left && right) moveForward(SPEED_BASE);
      else if (left && !right) setMotors(SPEED_SLOW, SPEED_BASE);
      else if (!left && right) setMotors(SPEED_BASE, SPEED_SLOW);
      else moveForward(SPEED_SLOW);
      
      if (millis() - homeTimer > 8000) {
        homeState = HOME_DETECT_START;
      }
      break;
      
    case HOME_DETECT_START:
      stopMotors();
      dist = readDistance();
      if (dist < 20) {
        if (DEBUG_STATE) Serial.println(F("Home: START reached!"));
        homeState = HOME_DONE;
      } else {
        stepForward(300);
        if (millis() - homeTimer > TIMEOUT_RETURN) {
          homeState = HOME_DONE;
        }
      }
      break;
      
    case HOME_DONE:
      stopMotors();
      homeState = HOME_FIND_LINE;
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
      if (DEBUG_STATE) Serial.println(F("=== PHASE 4: START ==="));
      stateStartTime = millis();
      currentState = STATE_DESCEND_RAMP;
      break;
      
    case STATE_DESCEND_RAMP:
      if (descendRamp()) {
        if (DEBUG_STATE) Serial.println(F("=== Back at bottom ==="));
        currentState = STATE_RETURN_HOME;
        stateStartTime = millis();
      }
      break;
      
    case STATE_RETURN_HOME:
      if (returnHome()) {
        if (DEBUG_STATE) Serial.println(F("================================="));
        if (DEBUG_STATE) Serial.println(F("    COURSE COMPLETE!"));
        if (DEBUG_STATE) Serial.println(F("================================="));
        currentState = STATE_DONE;
      }
      break;
      
    case STATE_DONE:
      stopMotors();
      // COURSE COMPLETE
      break;
  }
}
