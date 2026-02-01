/*
 * UTRA HACKS - CODE 1: Start to First Reupload Point
 * 
 * WORKFLOW:
 * 1. Start on BLACK line at BEGIN
 * 2. Line follow BLACK until intersection (detect RED or GREEN alongside)
 * 3. Turn to find and align with RED path
 * 4. Line follow RED path
 * 5. When BLUE detected -> turn RIGHT, pickup box (horn down->up), turn LEFT back
 * 6. Continue line following RED
 * 7. Stop at reupload checkpoint (PURPLE/gray detected or after set distance)
 * 
 * STATE MACHINE:
 * INIT -> LINE_FOLLOW_BLACK -> AT_INTERSECTION -> ALIGN_TO_RED -> 
 * LINE_FOLLOW_RED -> BOX_PICKUP_TURN -> BOX_PICKUP_APPROACH -> 
 * BOX_PICKUP_GRAB -> BOX_PICKUP_RETURN -> LINE_FOLLOW_RED_WITH_BOX -> STOP_REUPLOAD
 */

#include <Servo.h>

// ===================== PIN DEFINITIONS =====================
// L298N Motor Driver
const int ENA = 9;    // Left motor PWM
const int IN1 = 8;    // Left motor dir 1
const int IN2 = 7;    // Left motor dir 2
const int ENB = 10;   // Right motor PWM
const int IN3 = 12;   // Right motor dir 1
const int IN4 = 11;   // Right motor dir 2

// IR Sensors (LOW = line detected)
const int IR_LEFT = 2;
const int IR_RIGHT = 3;

// TCS3200 Color Sensor
const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int COLOR_OUT = 4;

// Ultrasonic Sensor (not used in CODE 1, but defined)
const int TRIG = 5;
const int ECHO = 6;

// Horn Servo
const int SERVO_PIN = 13;

// ===================== TUNABLE PARAMETERS =====================
// Motor speeds (0-255)
const int SPEED_NORMAL = 150;
const int SPEED_SLOW = 100;
const int SPEED_TURN = 160;

// Timing (milliseconds) - ADJUST THESE ON SITE
const int TURN_90_TIME = 350;         // Time for ~90 degree pivot turn
const int FORWARD_TO_BOX = 400;       // Forward distance to reach box
const int BACKWARD_FROM_BOX = 350;    // Backward after grabbing
const int HORN_GRAB_DELAY = 600;      // Time for horn to engage
const int POST_INTERSECTION_FWD = 200; // Small forward after intersection detect

// Horn servo angles
const int HORN_DOWN = 0;
const int HORN_UP = 40;

// Color sensor calibration - CALIBRATE THESE ON SITE
// These are pulse width values from TCS3200 (lower = more of that color)
struct ColorCal {
  int rMin, rMax;
  int gMin, gMax;
  int bMin, bMax;
};

// You MUST calibrate these by reading actual values on the course
ColorCal CAL_BLACK = {80, 200, 80, 200, 80, 200};     // All channels high (dark)
ColorCal CAL_RED = {10, 50, 60, 150, 60, 150};        // R low, G/B high
ColorCal CAL_GREEN = {60, 150, 10, 50, 60, 150};      // G low, R/B high
ColorCal CAL_BLUE = {60, 150, 60, 150, 10, 50};       // B low, R/G high
ColorCal CAL_WHITE = {10, 40, 10, 40, 10, 40};        // All channels low (bright)

// ===================== STATE MACHINE =====================
enum State {
  INIT,
  LINE_FOLLOW_BLACK,
  AT_INTERSECTION,
  ALIGN_TO_RED,
  LINE_FOLLOW_RED,
  BOX_PICKUP_TURN,
  BOX_PICKUP_APPROACH,
  BOX_PICKUP_GRAB,
  BOX_PICKUP_RETURN,
  LINE_FOLLOW_RED_WITH_BOX,
  STOP_REUPLOAD,
  ERROR_STATE
};

State currentState = INIT;
String stateNames[] = {
  "INIT", "LINE_FOLLOW_BLACK", "AT_INTERSECTION", "ALIGN_TO_RED",
  "LINE_FOLLOW_RED", "BOX_PICKUP_TURN", "BOX_PICKUP_APPROACH",
  "BOX_PICKUP_GRAB", "BOX_PICKUP_RETURN", "LINE_FOLLOW_RED_WITH_BOX",
  "STOP_REUPLOAD", "ERROR_STATE"
};

// ===================== GLOBAL VARIABLES =====================
Servo hornServo;
int colorR, colorG, colorB;
unsigned long stateStartTime = 0;
bool boxPickedUp = false;

// ===================== COLOR SENSOR FUNCTIONS =====================
int readColorChannel(int s2Val, int s3Val) {
  digitalWrite(S2, s2Val);
  digitalWrite(S3, s3Val);
  delay(10);
  return pulseIn(COLOR_OUT, LOW, 10000);
}

void readColor() {
  // TCS3200 filter selection:
  // S2=L, S3=L -> Red
  // S2=H, S3=H -> Green  
  // S2=L, S3=H -> Blue
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

const char* colorToString(DetectedColor c) {
  switch(c) {
    case C_BLACK: return "BLACK";
    case C_RED: return "RED";
    case C_GREEN: return "GREEN";
    case C_BLUE: return "BLUE";
    case C_WHITE: return "WHITE";
    default: return "UNKNOWN";
  }
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

// ===================== STATE TRANSITION =====================
void changeState(State newState) {
  Serial.print("STATE: ");
  Serial.print(stateNames[currentState]);
  Serial.print(" -> ");
  Serial.println(stateNames[newState]);
  currentState = newState;
  stateStartTime = millis();
}

unsigned long stateElapsed() {
  return millis() - stateStartTime;
}

// ===================== DEBUG =====================
void printDebug() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Color R:");
    Serial.print(colorR);
    Serial.print(" G:");
    Serial.print(colorG);
    Serial.print(" B:");
    Serial.print(colorB);
    Serial.print(" = ");
    Serial.print(colorToString(getColor()));
    Serial.print(" | IR L:");
    Serial.print(leftOnLine());
    Serial.print(" R:");
    Serial.println(rightOnLine());
    lastPrint = millis();
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial.println("CODE 1: Start to First Reupload Point");
  
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // IR sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  // Color sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  
  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  // Ultrasonic
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  // Servo
  hornServo.attach(SERVO_PIN);
  hornDown();
  
  stopMotors();
  delay(1000);
  
  changeState(LINE_FOLLOW_BLACK);
}

// ===================== MAIN LOOP =====================
void loop() {
  DetectedColor currentColor = getColor();
  
  switch (currentState) {
    
    case LINE_FOLLOW_BLACK:
      lineFollow(SPEED_NORMAL);
      if (currentColor == C_RED || currentColor == C_GREEN) {
        stopMotors();
        changeState(AT_INTERSECTION);
      }
      break;
      
    case AT_INTERSECTION:
      if (stateElapsed() > 200) {
        moveForward(SPEED_SLOW);
        delay(POST_INTERSECTION_FWD);
        stopMotors();
        changeState(ALIGN_TO_RED);
      }
      break;
      
    case ALIGN_TO_RED:
      currentColor = getColor();
      if (currentColor == C_RED && (leftOnLine() || rightOnLine())) {
        stopMotors();
        delay(100);
        changeState(LINE_FOLLOW_RED);
      } else {
        pivotRight(SPEED_SLOW);
        if (stateElapsed() > 3000) {
          stopMotors();
          Serial.println("ERROR: Could not find RED path");
          changeState(ERROR_STATE);
        }
      }
      break;
      
    case LINE_FOLLOW_RED:
      if (currentColor == C_BLUE) {
        stopMotors();
        changeState(BOX_PICKUP_TURN);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case BOX_PICKUP_TURN:
      pivotRight(SPEED_TURN);
      if (stateElapsed() > TURN_90_TIME) {
        stopMotors();
        delay(100);
        changeState(BOX_PICKUP_APPROACH);
      }
      break;
      
    case BOX_PICKUP_APPROACH:
      moveForward(SPEED_SLOW);
      if (stateElapsed() > FORWARD_TO_BOX) {
        stopMotors();
        changeState(BOX_PICKUP_GRAB);
      }
      break;
      
    case BOX_PICKUP_GRAB:
      hornUp();
      if (stateElapsed() > HORN_GRAB_DELAY) {
        boxPickedUp = true;
        changeState(BOX_PICKUP_RETURN);
      }
      break;
      
    case BOX_PICKUP_RETURN:
      if (stateElapsed() < BACKWARD_FROM_BOX) {
        moveBackward(SPEED_SLOW);
      } else if (stateElapsed() < BACKWARD_FROM_BOX + TURN_90_TIME) {
        pivotLeft(SPEED_TURN);
      } else {
        stopMotors();
        delay(100);
        changeState(LINE_FOLLOW_RED_WITH_BOX);
      }
      break;
      
    case LINE_FOLLOW_RED_WITH_BOX:
      // Continue until we hit gray reupload point
      // Gray should read as neither red/green/blue distinctly
      if (currentColor != C_RED && currentColor != C_BLUE && 
          currentColor != C_BLACK && currentColor != C_UNKNOWN) {
        stopMotors();
        changeState(STOP_REUPLOAD);
      } else {
        lineFollow(SPEED_NORMAL);
      }
      break;
      
    case STOP_REUPLOAD:
      stopMotors();
      Serial.println("=== REUPLOAD POINT REACHED ===");
      Serial.println("Upload CODE 2 now.");
      while(1) {
        delay(1000);
        Serial.println("Waiting for CODE 2...");
      }
      break;
      
    case ERROR_STATE:
      stopMotors();
      Serial.println("ERROR STATE - Check robot");
      delay(1000);
      break;
      
    default:
      stopMotors();
      break;
  }
  
  printDebug();
  delay(10);
}

// ===================== CALIBRATION HELPER =====================
// Uncomment in setup() to use: calibrateColors();
void calibrateColors() {
  Serial.println("=== COLOR CALIBRATION MODE ===");
  Serial.println("Place robot on each color and record values");
  while(1) {
    readColor();
    Serial.print("R: ");
    Serial.print(colorR);
    Serial.print(" G: ");
    Serial.print(colorG);
    Serial.print(" B: ");
    Serial.println(colorB);
    delay(500);
  }
}
