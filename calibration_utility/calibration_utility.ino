/*
 * UTRA HACKS - Color Sensor Calibration Utility
 * 
 * USAGE:
 * 1. Upload this sketch
 * 2. Open Serial Monitor at 115200 baud
 * 3. Place robot on each color surface (BLACK, RED, GREEN, BLUE, WHITE)
 * 4. Record the R, G, B values for each color
 * 5. Update the CAL_xxx values in CODE 1, 2, 3
 * 
 * IMPORTANT: The TCS3200 outputs PULSE WIDTH, not color intensity.
 * LOWER values = MORE of that color reflected
 * HIGHER values = LESS of that color reflected
 * 
 * So BLACK (absorbs all light) will have HIGH R, G, B values
 * WHITE (reflects all light) will have LOW R, G, B values
 * RED surface will have LOW R, HIGH G, HIGH B
 */

// TCS3200 Color Sensor
const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int COLOR_OUT = 4;

// IR Sensors
const int IR_LEFT = 2;
const int IR_RIGHT = 3;

int colorR, colorG, colorB;

int readColorChannel(int s2Val, int s3Val) {
  digitalWrite(S2, s2Val);
  digitalWrite(S3, s3Val);
  delay(10);
  return pulseIn(COLOR_OUT, LOW, 10000);
}

void readColor() {
  colorR = readColorChannel(LOW, LOW);   // Red filter
  colorG = readColorChannel(HIGH, HIGH); // Green filter
  colorB = readColorChannel(LOW, HIGH);  // Blue filter
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== TCS3200 Color Calibration ===");
  Serial.println("Place robot on each surface and record values");
  Serial.println("");
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  
  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
}

void loop() {
  readColor();
  
  Serial.print("R: ");
  Serial.print(colorR);
  Serial.print("\tG: ");
  Serial.print(colorG);
  Serial.print("\tB: ");
  Serial.print(colorB);
  
  // Guess the color based on typical ranges
  Serial.print("\t| Guess: ");
  
  if (colorR < 50 && colorG > 60 && colorB > 60) {
    Serial.print("RED");
  } else if (colorR > 60 && colorG < 50 && colorB > 60) {
    Serial.print("GREEN");
  } else if (colorR > 60 && colorG > 60 && colorB < 50) {
    Serial.print("BLUE");
  } else if (colorR > 80 && colorG > 80 && colorB > 80) {
    Serial.print("BLACK/DARK");
  } else if (colorR < 40 && colorG < 40 && colorB < 40) {
    Serial.print("WHITE/BRIGHT");
  } else {
    Serial.print("UNKNOWN");
  }
  
  // Also show IR sensor states
  Serial.print("\t| IR L:");
  Serial.print(digitalRead(IR_LEFT) == LOW ? "LINE" : "----");
  Serial.print(" R:");
  Serial.println(digitalRead(IR_RIGHT) == LOW ? "LINE" : "----");
  
  delay(300);
}

/*
 * CALIBRATION INSTRUCTIONS:
 * 
 * 1. Place robot on BLACK surface, note values:
 *    Expected: R~100+, G~100+, B~100+ (all high = dark)
 *    Your values: R:____ G:____ B:____
 *    
 * 2. Place robot on RED surface, note values:
 *    Expected: R~10-50, G~60-150, B~60-150
 *    Your values: R:____ G:____ B:____
 *    
 * 3. Place robot on GREEN surface, note values:
 *    Expected: R~60-150, G~10-50, B~60-150
 *    Your values: R:____ G:____ B:____
 *    
 * 4. Place robot on BLUE surface, note values:
 *    Expected: R~60-150, G~60-150, B~10-50
 *    Your values: R:____ G:____ B:____
 *    
 * 5. Place robot on WHITE surface, note values:
 *    Expected: R~10-40, G~10-40, B~10-40 (all low = bright)
 *    Your values: R:____ G:____ B:____
 * 
 * Then update CAL_xxx in all three code files:
 * 
 * ColorCal CAL_BLACK = {rMin, rMax, gMin, gMax, bMin, bMax};
 * ColorCal CAL_RED = {rMin, rMax, gMin, gMax, bMin, bMax};
 * etc.
 * 
 * Add +/-20 margin to each value for safety.
 */
