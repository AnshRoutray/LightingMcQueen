/*
 * UTRA HACKS - CODE 3: Target Navigation + Return to Start
 * 
 * WORKFLOW:
 * 1. Robot placed randomly on target (raised platform with colored rings)
 * 2. SUMO ALGORITHM: Go forward, detect edge with IR -> back up, random turn, repeat
 * 3. When BLACK detected (center) -> stop, spin to find wall with US
 * 4. Wall is opposite the ramp - orient toward wall
 * 5. Go to center of black area (US-guided)
 * 6. Do 180 turn (now facing ramp/down)
 * 7. Go down ramp (follow green line)
 * 8. At fork, continue straight on green
 * 9. Hit BLUE -> turn RIGHT, drop box, turn LEFT back
 * 10. Continue on green to junction
 * 11. Find BLACK path, follow to start
 * 12. STOP at start
 * 
 * STATE MACHINE:
 * INIT -> SUMO_FORWARD -> SUMO_EDGE_BACKUP -> SUMO_RANDOM_TURN ->
 * BLACK_FOUND -> SPIN_FIND_WALL -> ALIGN_TO_WALL -> GO_TO_CENTER ->
 * TURN_180 -> GO_DOWN_RAMP -> LINE_FOLLOW_GREEN_DOWN -> AT_FORK_STRAIGHT ->
 * LINE_FOLLOW_GREEN_2 -> BOX_DROPOFF_TURN -> BOX_DROPOFF_LOWER -> 
 * BOX_DROPOFF_RETURN -> LINE_FOLLOW_GREEN_3 -> AT_JUNCTION -> FIND_BLACK ->
 * LINE_FOLLOW_BLACK_HOME -> STOP_FINISH
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
const int SPEED_SUMO = 120;

// Timing (ms)
const int TURN_90_TIME = 350;
const int TURN_180_TIME = 700;
const int FORWARD_TO_BOX = 400;
const int BACKWARD_FROM_BOX = 350;
const int HORN_DELAY = 600;
const int SUMO_BACKUP_TIME = 300;
const int SUMO_TURN_MIN = 200;
const int SUMO_TURN_MAX = 600;

// Distances (cm)
const int WALL_DETECT_DIST = 100;    // Max distance to "see" wall
const int WALL_TARGET_DIST = 30;     // Stop this far from wall
const int CENTER_FORWARD_TIME = 400; // Time to drive to center after wall align

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
  SUMO_FORWARD,
  SUMO_EDGE_BACKUP,
  SUMO_RANDOM_TURN,
  BLACK_FOUND,
  SPIN_FIND_WALL,
  ALIGN_TO_WALL,
  GO_TO_CENTER,
  TURN_180,
  GO_DOWN_RAMP,
  LINE_FOLLOW_GREEN_DOWN,
  AT_FORK_STRAIGHT,
  LINE_FOLLOW_GREEN_2,
  BOX_DROPOFF_TURN,
  BOX_DROPOFF_APPROACH,
  BOX_DROPOFF_LOWER,
  BOX_DROPOFF_BACKUP,
  BOX_DROPOFF_RETURN,
  LINE_FOLLOW_GREEN_3,
  AT_JUNCTION,
  FIND_BLACK,
  LINE_FOLLOW_BLACK_HOME,
  STOP_FINISH,
  ERROR_STATE
};

State currentState = INIT;

// ===================== GLOBAL VARIABLES =====================
Servo hornServo;
int colorR, colorG, colorB;
unsigned long stateStartTime = 0;
int randomTurnTime = 0;
bool hasBox = true;  // Has box from CODE 2
long wallDistance = 999;

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
// In SUMO mode, we use IR to detect EDGE (white/off platform)
// LOW = on dark surface (platform colors)
// HIGH = off platform or white edge
bool leftOnLine() {
  return digitalRead(IR_LEFT) == LOW;
}

bool rightOnLine() {
  return digitalRead(IR_RIGHT) == LOW;
}

bool leftSeeEdge() {
  return digitalRead(IR_LEFT) == HIGH;
}

bool rightSeeEdge() {
  return digitalRead(IR_RIGHT) == HIGH;
}

// ===================== ULTRASONIC FUNCTIONS =====================
long getDistanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
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
  Serial.println("CODE 3: Target Navigation + Return Home");
  
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
  hornUp();  // Has box from CODE 2
  
  randomSeed(analogRead(A4));  // Seed random from floating pin
  
  stopMotors();
  delay(1000);
  
  changeState(SUMO_FORWARD);
}

// ===================== MAIN LOOP =====================
void loop() {
  DetectedColor currentColor = getColor();
  
  switch (currentState) {
    
    // ===== SUMO ALGORITHM TO FIND BLACK CENTER =====
    case SUMO_FORWARD:
      // Check for black (center target)
      if (currentColor == C_BLACK) {
        stopMotors();
        changeState(BLACK_FOUND);
        break;
      }
      
      // Check for edge (about to fall off)
      if (leftSeeEdge() || rightSeeEdge()) {
        stopMotors();
        changeState(SUMO_EDGE_BACKUP);
        break;
      }
      
      // Otherwise keep going forward
      moveForward(SPEED_SUMO);
      break;
      
    case SUMO_EDGE_BACKUP:
      moveBackward(SPEED_SUMO);
      if (stateElapsed() > SUMO_BACKUP_TIME) {
        stopMotors();
        // Generate random turn time
        randomTurnTime = random(SUMO_TURN_MIN, SUMO_TURN_MAX);
        changeState(SUMO_RANDOM_TURN);
      }
      break;
      
    case SUMO_RANDOM_TURN:
      // Random direction turn
      if (random(2) == 0) {
        pivotLeft(SPEED_TURN);
      } else {
        pivotRight(SPEED_TURN);
      }
      if (stateElapsed() > randomTurnTime) {
        stopMotors();
        changeState(SUMO_FORWARD);
      }
      break;
      
    // ===== FOUND BLACK CENTER =====
    case BLACK_FOUND:
      Serial.println("BLACK CENTER FOUND!");
      if (stateElapsed() > 300) {
        changeState(SPIN_FIND_WALL);
      }
      break;
      
    case SPIN_FIND_WALL:
      // Spin slowly until US sees wall (something closer than max)
      wallDistance = getDistanceCM();
      Serial.print("Wall dist: ");
      Serial.println(wallDistance);
      
      if (wallDistance < WALL_DETECT_DIST && wallDistance > 5) {
        // Found wall
        stopMotors();
        changeState(ALIGN_TO_WALL);
      } else {
        pivotRight(SPEED_SLOW);
        // Timeout after full rotation
        if (stateElapsed() > 3000) {
          Serial.println("WARN: Wall not found, proceeding anyway");
          stopMotors();
          changeState(GO_TO_CENTER);
        }
      }
      break;
      
    case ALIGN_TO_WALL:
      // Fine-tune alignment by finding minimum distance (facing wall directly)
      // Simple approach: we already see wall, assume roughly aligned
      if (stateElapsed() > 200) {
        changeState(GO_TO_CENTER);
      }
      break;
      
    case GO_TO_CENTER:
      // Move forward a bit to center on black
      // Then we'll be facing the wall, need 180 to face ramp
      moveForward(SPEED_SLOW);
      if (stateElapsed() > CENTER_FORWARD_TIME) {
        stopMotors();
        changeState(TURN_180);
      }
      break;
      
    case TURN_180:
      // Turn 180 to face the ramp (away from wall)
      pivotRight(SPEED_TURN);
      if (stateElapsed() > TURN_180_TIME) {
        stopMotors();
        changeState(GO_DOWN_RAMP);
      }
      break;
      
    // ===== GO DOWN RAMP =====
    case GO_DOWN_RAMP:
      // Go straight forward to get onto ramp
      moveForward(SPEED_SLOW);
      // Once we see green (ramp path), switch to line following
      if (currentColor == C_GREEN) {
        changeState(LINE_FOLLOW_GREEN_DOWN);
      }
      // Timeout
      if (stateElapsed() > 3000) {
        // Force switch, hope we're on path
        changeState(LINE_FOLLOW_GREEN_DOWN);
      }
      break;
      
    case LINE_FOLLOW_GREEN_DOWN:
      // Follow green down the ramp
      // Watch for fork (both IRs see line = wide section)
      if (leftOnLine() && rightOnLine() && stateElapsed() > 1000) {
        stopMotors();
        changeState(AT_FORK_STRAIGHT);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case AT_FORK_STRAIGHT:
      // At fork, go STRAIGHT (stay on green toward dropoff)
      // The curved ramp goes right, we want straight
      moveForward(SPEED_SLOW);
      if (stateElapsed() > 500) {
        changeState(LINE_FOLLOW_GREEN_2);
      }
      break;
      
    case LINE_FOLLOW_GREEN_2:
      // Continue on green until blue (dropoff zone)
      if (currentColor == C_BLUE) {
        stopMotors();
        changeState(BOX_DROPOFF_TURN);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    // ===== BOX DROPOFF =====
    case BOX_DROPOFF_TURN:
      // Turn RIGHT to face dropoff zone (based on map, coming down from ramp)
      pivotRight(SPEED_TURN);
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
      // Turn LEFT to return to path
      pivotLeft(SPEED_TURN);
      if (stateElapsed() > TURN_90_TIME) {
        stopMotors();
        changeState(LINE_FOLLOW_GREEN_3);
      }
      break;
      
    // ===== RETURN TO START =====
    case LINE_FOLLOW_GREEN_3:
      // Follow green to junction
      if (currentColor == C_BLACK) {
        stopMotors();
        changeState(AT_JUNCTION);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case AT_JUNCTION:
      if (stateElapsed() > 200) {
        changeState(FIND_BLACK);
      }
      break;
      
    case FIND_BLACK:
      // Find and align to black path (home)
      currentColor = getColor();
      if (currentColor == C_BLACK && (leftOnLine() || rightOnLine())) {
        stopMotors();
        changeState(LINE_FOLLOW_BLACK_HOME);
      } else {
        pivotRight(SPEED_SLOW);
        if (stateElapsed() > 2000) {
          pivotLeft(SPEED_SLOW);
        }
        if (stateElapsed() > 4000) {
          Serial.println("ERROR: Cannot find black home path");
          changeState(ERROR_STATE);
        }
      }
      break;
      
    case LINE_FOLLOW_BLACK_HOME:
      // Follow black back to start
      // Detect end: both IR off line and color not black for a while
      lineFollow(SPEED_NORMAL);
      
      // Simple end detection: if we've been on black long enough
      // and suddenly lose line completely, we're at the end
      if (!leftOnLine() && !rightOnLine() && stateElapsed() > 3000) {
        // Might be at end, check color
        if (currentColor != C_BLACK && currentColor != C_UNKNOWN) {
          stopMotors();
          changeState(STOP_FINISH);
        }
      }
      break;
      
    case STOP_FINISH:
      stopMotors();
      hornDown();  // Lower horn for clean finish
      Serial.println("=============================");
      Serial.println("=== COURSE COMPLETE! ===");
      Serial.println("=============================");
      while(1) {
        delay(1000);
        Serial.println("FINISHED!");
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
