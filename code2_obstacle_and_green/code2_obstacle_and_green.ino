/*
 * UTRA HACKS - CODE 2: Red Obstacle Course + Return + Green Path to Ramp
 * 
 * WORKFLOW:
 * 1. Continue on RED path with box
 * 2. Follow line, detect obstacle with US -> go around RIGHT (square maneuver)
 * 3. Return to line, continue
 * 4. Repeat obstacle avoidance for second obstacle
 * 5. Hit BLUE -> turn LEFT, drop box, turn RIGHT back
 * 6. Continue on RED back to junction
 * 7. Find and align to BLACK line at junction
 * 8. Find GREEN path, follow it
 * 9. Hit BLUE -> pickup second box
 * 10. At fork, take LEFT/straight (straight green to ramp)
 * 11. Stop at reupload point
 * 
 * STATE MACHINE:
 * INIT -> LINE_FOLLOW_RED -> OBSTACLE_DETECT -> OBSTACLE_AVOID_1 -> ... ->
 * OBSTACLE_AVOID_6 -> FIND_LINE -> LINE_FOLLOW_RED_2 -> (repeat obstacle) ->
 * BOX_DROPOFF_TURN -> BOX_DROPOFF_LOWER -> BOX_DROPOFF_RETURN ->
 * RETURN_TO_JUNCTION -> FIND_BLACK -> FIND_GREEN -> LINE_FOLLOW_GREEN ->
 * BOX2_PICKUP_TURN -> BOX2_PICKUP_APPROACH -> BOX2_PICKUP_GRAB -> BOX2_PICKUP_RETURN ->
 * LINE_FOLLOW_GREEN_WITH_BOX -> AT_FORK_STAY_LEFT -> LINE_FOLLOW_RAMP -> STOP_REUPLOAD
 */

#include <Servo.h>

// ===================== PIN DEFINITIONS =====================
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;
const int ENB = 10;
const int IN3 = 12;
const int IN4 = 11;

const int IR_LEFT = 2;
const int IR_RIGHT = 3;

const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int COLOR_OUT = 4;

const int TRIG = 5;
const int ECHO = 6;

const int SERVO_PIN = 13;

// ===================== TUNABLE PARAMETERS =====================
const int SPEED_NORMAL = 150;
const int SPEED_SLOW = 100;
const int SPEED_TURN = 160;
const int SPEED_OBSTACLE = 140;

// Timing (ms) - TUNE ON SITE
const int TURN_90_TIME = 350;
const int TURN_45_TIME = 175;
const int FORWARD_TO_BOX = 400;
const int BACKWARD_FROM_BOX = 350;
const int HORN_DELAY = 600;

// Obstacle avoidance timing - forms a square path around obstacle
const int OBS_TURN_TIME = 350;       // 90 deg turn
const int OBS_FORWARD_1 = 400;       // Forward past obstacle width
const int OBS_FORWARD_2 = 500;       // Forward past obstacle length
const int OBS_FORWARD_3 = 400;       // Forward back to line area

// Ultrasonic obstacle threshold (cm)
const int OBSTACLE_DIST = 15;

// Horn angles
const int HORN_DOWN = 0;
const int HORN_UP = 40;

// Color calibration
struct ColorCal {
  int rMin, rMax;
  int gMin, gMax;
  int bMin, bMax;
};

ColorCal CAL_BLACK = {80, 200, 80, 200, 80, 200};
ColorCal CAL_RED = {10, 50, 60, 150, 60, 150};
ColorCal CAL_GREEN = {60, 150, 10, 50, 60, 150};
ColorCal CAL_BLUE = {60, 150, 60, 150, 10, 50};
ColorCal CAL_WHITE = {10, 40, 10, 40, 10, 40};

// ===================== STATE MACHINE =====================
enum State {
  INIT,
  LINE_FOLLOW_RED,
  OBSTACLE_DETECTED,
  OBS_TURN_RIGHT,
  OBS_FWD_1,
  OBS_TURN_LEFT,
  OBS_FWD_2,
  OBS_TURN_LEFT_2,
  OBS_FWD_3,
  OBS_TURN_RIGHT_2,
  OBS_FIND_LINE,
  LINE_FOLLOW_RED_2,
  OBS2_DETECTED,
  OBS2_TURN_RIGHT,
  OBS2_FWD_1,
  OBS2_TURN_LEFT,
  OBS2_FWD_2,
  OBS2_TURN_LEFT_2,
  OBS2_FWD_3,
  OBS2_TURN_RIGHT_2,
  OBS2_FIND_LINE,
  LINE_FOLLOW_RED_3,
  BOX_DROPOFF_TURN,
  BOX_DROPOFF_APPROACH,
  BOX_DROPOFF_LOWER,
  BOX_DROPOFF_BACKUP,
  BOX_DROPOFF_RETURN,
  RETURN_TO_JUNCTION,
  AT_JUNCTION,
  FIND_BLACK,
  FIND_GREEN,
  LINE_FOLLOW_GREEN,
  BOX2_PICKUP_TURN,
  BOX2_PICKUP_APPROACH,
  BOX2_PICKUP_GRAB,
  BOX2_PICKUP_RETURN,
  LINE_FOLLOW_GREEN_BOX,
  AT_FORK,
  LINE_FOLLOW_RAMP,
  STOP_REUPLOAD,
  ERROR_STATE
};

State currentState = INIT;

// ===================== GLOBAL VARIABLES =====================
Servo hornServo;
int colorR, colorG, colorB;
unsigned long stateStartTime = 0;
int obstacleCount = 0;
bool hasBox = true;  // Starting with box from CODE 1

// ===================== COLOR SENSOR FUNCTIONS =====================
int readColorChannel(int s2Val, int s3Val) {
  digitalWrite(S2, s2Val);
  digitalWrite(S3, s3Val);
  delay(10);
  return pulseIn(COLOR_OUT, LOW, 10000);
}

void readColor() {
  colorR = readColorChannel(LOW, LOW);
  colorG = readColorChannel(HIGH, HIGH);
  colorB = readColorChannel(LOW, HIGH);
}

enum DetectedColor { C_UNKNOWN, C_BLACK, C_RED, C_GREEN, C_BLUE, C_WHITE };

bool inRange(int val, int minV, int maxV) {
  return (val >= minV && val <= maxV);
}

DetectedColor getColor() {
  readColor();
  if (colorR == 0 && colorG == 0 && colorB == 0) return C_UNKNOWN;
  
  if (inRange(colorR, CAL_RED.rMin, CAL_RED.rMax) &&
      inRange(colorG, CAL_RED.gMin, CAL_RED.gMax) &&
      inRange(colorB, CAL_RED.bMin, CAL_RED.bMax)) return C_RED;
      
  if (inRange(colorR, CAL_GREEN.rMin, CAL_GREEN.rMax) &&
      inRange(colorG, CAL_GREEN.gMin, CAL_GREEN.gMax) &&
      inRange(colorB, CAL_GREEN.bMin, CAL_GREEN.bMax)) return C_GREEN;
      
  if (inRange(colorR, CAL_BLUE.rMin, CAL_BLUE.rMax) &&
      inRange(colorG, CAL_BLUE.gMin, CAL_BLUE.gMax) &&
      inRange(colorB, CAL_BLUE.bMin, CAL_BLUE.bMax)) return C_BLUE;
      
  if (inRange(colorR, CAL_WHITE.rMin, CAL_WHITE.rMax) &&
      inRange(colorG, CAL_WHITE.gMin, CAL_WHITE.gMax) &&
      inRange(colorB, CAL_WHITE.bMin, CAL_WHITE.bMax)) return C_WHITE;
      
  if (inRange(colorR, CAL_BLACK.rMin, CAL_BLACK.rMax) &&
      inRange(colorG, CAL_BLACK.gMin, CAL_BLACK.gMax) &&
      inRange(colorB, CAL_BLACK.bMin, CAL_BLACK.bMax)) return C_BLACK;
  
  return C_UNKNOWN;
}

// ===================== MOTOR FUNCTIONS =====================
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setLeftMotor(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setRightMotor(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void moveForward(int speed) {
  setLeftMotor(speed);
  setRightMotor(speed);
}

void moveBackward(int speed) {
  setLeftMotor(-speed);
  setRightMotor(-speed);
}

void pivotRight(int speed) {
  setLeftMotor(speed);
  setRightMotor(-speed);
}

void pivotLeft(int speed) {
  setLeftMotor(-speed);
  setRightMotor(speed);
}

void curveLeft(int speed) {
  setLeftMotor(speed / 3);
  setRightMotor(speed);
}

void curveRight(int speed) {
  setLeftMotor(speed);
  setRightMotor(speed / 3);
}

// ===================== IR SENSOR FUNCTIONS =====================
bool leftOnLine() {
  return digitalRead(IR_LEFT) == LOW;
}

bool rightOnLine() {
  return digitalRead(IR_RIGHT) == LOW;
}

// ===================== ULTRASONIC FUNCTIONS =====================
long getDistanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH, 30000);  // Timeout 30ms
  if (duration == 0) return 999;  // No echo
  return duration * 0.034 / 2;
}

bool obstacleAhead() {
  long dist = getDistanceCM();
  return (dist > 0 && dist < OBSTACLE_DIST);
}

// ===================== LINE FOLLOWING =====================
bool lineFollow(int speed) {
  bool left = leftOnLine();
  bool right = rightOnLine();
  
  if (left && right) {
    moveForward(speed);
  } else if (left && !right) {
    curveLeft(speed);
  } else if (!left && right) {
    curveRight(speed);
  } else {
    moveForward(speed / 2);
    return false;
  }
  return true;
}

// ===================== HORN FUNCTIONS =====================
void hornDown() {
  hornServo.write(HORN_DOWN);
}

void hornUp() {
  hornServo.write(HORN_UP);
}

// ===================== STATE FUNCTIONS =====================
void changeState(State newState) {
  Serial.print("STATE: ");
  Serial.print(currentState);
  Serial.print(" -> ");
  Serial.println(newState);
  currentState = newState;
  stateStartTime = millis();
}

unsigned long stateElapsed() {
  return millis() - stateStartTime;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial.println("CODE 2: Obstacle Course + Green Path");
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  hornServo.attach(SERVO_PIN);
  hornUp();  // Start with horn UP (holding box from CODE 1)
  
  stopMotors();
  delay(1000);
  
  changeState(LINE_FOLLOW_RED);
}

// ===================== MAIN LOOP =====================
void loop() {
  DetectedColor currentColor = getColor();
  
  switch (currentState) {
    
    // ===== FIRST OBSTACLE SECTION =====
    case LINE_FOLLOW_RED:
      if (obstacleAhead()) {
        stopMotors();
        changeState(OBSTACLE_DETECTED);
      } else if (currentColor == C_BLUE && obstacleCount >= 2) {
        // Blue after 2 obstacles = dropoff zone
        stopMotors();
        changeState(BOX_DROPOFF_TURN);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case OBSTACLE_DETECTED:
      obstacleCount++;
      Serial.print("Obstacle #");
      Serial.println(obstacleCount);
      changeState(OBS_TURN_RIGHT);
      break;
      
    // Obstacle avoidance: RIGHT, FWD, LEFT, FWD, LEFT, FWD, RIGHT (square around RIGHT side)
    case OBS_TURN_RIGHT:
      pivotRight(SPEED_TURN);
      if (stateElapsed() > OBS_TURN_TIME) {
        stopMotors();
        changeState(OBS_FWD_1);
      }
      break;
      
    case OBS_FWD_1:
      moveForward(SPEED_OBSTACLE);
      if (stateElapsed() > OBS_FORWARD_1) {
        stopMotors();
        changeState(OBS_TURN_LEFT);
      }
      break;
      
    case OBS_TURN_LEFT:
      pivotLeft(SPEED_TURN);
      if (stateElapsed() > OBS_TURN_TIME) {
        stopMotors();
        changeState(OBS_FWD_2);
      }
      break;
      
    case OBS_FWD_2:
      moveForward(SPEED_OBSTACLE);
      if (stateElapsed() > OBS_FORWARD_2) {
        stopMotors();
        changeState(OBS_TURN_LEFT_2);
      }
      break;
      
    case OBS_TURN_LEFT_2:
      pivotLeft(SPEED_TURN);
      if (stateElapsed() > OBS_TURN_TIME) {
        stopMotors();
        changeState(OBS_FWD_3);
      }
      break;
      
    case OBS_FWD_3:
      moveForward(SPEED_OBSTACLE);
      if (stateElapsed() > OBS_FORWARD_3) {
        stopMotors();
        changeState(OBS_TURN_RIGHT_2);
      }
      break;
      
    case OBS_TURN_RIGHT_2:
      pivotRight(SPEED_TURN);
      if (stateElapsed() > OBS_TURN_TIME) {
        stopMotors();
        changeState(OBS_FIND_LINE);
      }
      break;
      
    case OBS_FIND_LINE:
      // Slowly move forward until we find the red line
      moveForward(SPEED_SLOW);
      if (leftOnLine() || rightOnLine()) {
        stopMotors();
        if (obstacleCount < 2) {
          changeState(LINE_FOLLOW_RED);  // Go for obstacle 2
        } else {
          changeState(LINE_FOLLOW_RED);  // Continue to dropoff
        }
      }
      if (stateElapsed() > 2000) {
        // Timeout - try turning to find line
        pivotLeft(SPEED_SLOW);
      }
      break;
      
    // ===== BOX DROPOFF SECTION =====
    case BOX_DROPOFF_TURN:
      // Turn LEFT to face dropoff zone
      pivotLeft(SPEED_TURN);
      if (stateElapsed() > TURN_90_TIME) {
        stopMotors();
        changeState(BOX_DROPOFF_APPROACH);
      }
      break;
      
    case BOX_DROPOFF_APPROACH:
      moveForward(SPEED_SLOW);
      if (stateElapsed() > FORWARD_TO_BOX) {
        stopMotors();
        changeState(BOX_DROPOFF_LOWER);
      }
      break;
      
    case BOX_DROPOFF_LOWER:
      hornDown();
      if (stateElapsed() > HORN_DELAY) {
        hasBox = false;
        changeState(BOX_DROPOFF_BACKUP);
      }
      break;
      
    case BOX_DROPOFF_BACKUP:
      moveBackward(SPEED_SLOW);
      if (stateElapsed() > BACKWARD_FROM_BOX) {
        stopMotors();
        changeState(BOX_DROPOFF_RETURN);
      }
      break;
      
    case BOX_DROPOFF_RETURN:
      // Turn RIGHT to return to path
      pivotRight(SPEED_TURN);
      if (stateElapsed() > TURN_90_TIME) {
        stopMotors();
        changeState(RETURN_TO_JUNCTION);
      }
      break;
      
    // ===== RETURN TO JUNCTION SECTION =====
    case RETURN_TO_JUNCTION:
      // Follow red path back to junction
      // Junction = when we see black or green
      currentColor = getColor();
      if (currentColor == C_BLACK || currentColor == C_GREEN) {
        stopMotors();
        changeState(AT_JUNCTION);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case AT_JUNCTION:
      // Brief pause
      if (stateElapsed() > 200) {
        changeState(FIND_BLACK);
      }
      break;
      
    case FIND_BLACK:
      // Turn to find and align on black line for orientation
      currentColor = getColor();
      if (currentColor == C_BLACK && (leftOnLine() || rightOnLine())) {
        stopMotors();
        delay(100);
        changeState(FIND_GREEN);
      } else {
        pivotRight(SPEED_SLOW);
        if (stateElapsed() > 3000) {
          // Try other direction
          pivotLeft(SPEED_SLOW);
        }
      }
      break;
      
    case FIND_GREEN:
      // From black, turn to find green path
      currentColor = getColor();
      if (currentColor == C_GREEN && (leftOnLine() || rightOnLine())) {
        stopMotors();
        delay(100);
        changeState(LINE_FOLLOW_GREEN);
      } else {
        pivotRight(SPEED_SLOW);
        if (stateElapsed() > 4000) {
          Serial.println("ERROR: Cannot find green");
          changeState(ERROR_STATE);
        }
      }
      break;
      
    // ===== GREEN PATH + BOX 2 PICKUP =====
    case LINE_FOLLOW_GREEN:
      currentColor = getColor();
      if (currentColor == C_BLUE) {
        stopMotors();
        changeState(BOX2_PICKUP_TURN);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case BOX2_PICKUP_TURN:
      // Turn RIGHT to face box (green path box is on right side based on map)
      pivotRight(SPEED_TURN);
      if (stateElapsed() > TURN_90_TIME) {
        stopMotors();
        changeState(BOX2_PICKUP_APPROACH);
      }
      break;
      
    case BOX2_PICKUP_APPROACH:
      moveForward(SPEED_SLOW);
      if (stateElapsed() > FORWARD_TO_BOX) {
        stopMotors();
        changeState(BOX2_PICKUP_GRAB);
      }
      break;
      
    case BOX2_PICKUP_GRAB:
      hornUp();
      if (stateElapsed() > HORN_DELAY) {
        hasBox = true;
        changeState(BOX2_PICKUP_RETURN);
      }
      break;
      
    case BOX2_PICKUP_RETURN:
      if (stateElapsed() < BACKWARD_FROM_BOX) {
        moveBackward(SPEED_SLOW);
      } else if (stateElapsed() < BACKWARD_FROM_BOX + TURN_90_TIME) {
        pivotLeft(SPEED_TURN);
      } else {
        stopMotors();
        changeState(LINE_FOLLOW_GREEN_BOX);
      }
      break;
      
    case LINE_FOLLOW_GREEN_BOX:
      // Follow green until fork (we'll see the path split)
      // At fork, stay LEFT (straight green path to ramp)
      // Fork detection: both IRs see line AND color is still green
      if (leftOnLine() && rightOnLine() && stateElapsed() > 500) {
        // Possible fork - check if this is wide section
        stopMotors();
        delay(100);
        changeState(AT_FORK);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case AT_FORK:
      // Stay LEFT/STRAIGHT at fork
      // Go forward while staying on left side
      moveForward(SPEED_SLOW);
      if (stateElapsed() > 300) {
        // Keep left by curving slightly left if needed
        if (rightOnLine() && !leftOnLine()) {
          // We drifted right, curve back left
          curveLeft(SPEED_SLOW);
        }
        if (stateElapsed() > 600) {
          changeState(LINE_FOLLOW_RAMP);
        }
      }
      break;
      
    case LINE_FOLLOW_RAMP:
      // Follow green up to reupload point
      currentColor = getColor();
      // Reupload point is gray - stop when not green for a while
      if (currentColor != C_GREEN && currentColor != C_BLUE && 
          currentColor != C_UNKNOWN && stateElapsed() > 1000) {
        stopMotors();
        changeState(STOP_REUPLOAD);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case STOP_REUPLOAD:
      stopMotors();
      Serial.println("=== REUPLOAD POINT 2 REACHED ===");
      Serial.println("Robot is at top of ramp. Upload CODE 3 now.");
      while(1) {
        delay(1000);
        Serial.println("Waiting for CODE 3...");
      }
      break;
      
    case ERROR_STATE:
      stopMotors();
      Serial.println("ERROR STATE");
      delay(1000);
      break;
      
    default:
      stopMotors();
      break;
  }
  
  delay(10);
}
