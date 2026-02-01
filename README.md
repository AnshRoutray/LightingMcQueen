# UTRA HACKS - Robot Code Package

## Quick Start
1. Wire everything according to pin diagram below
2. Upload `calibration_utility.ino` first
3. Calibrate colors on actual course surfaces
4. Update CAL_xxx values in all three code files
5. Upload CODE 1, run to first reupload point
6. Upload CODE 2, run to second reupload point  
7. Upload CODE 3, complete course

---

## Wiring Diagram

### L298N Motor Driver
```
Arduino     L298N
---------   -------
Pin 9   ->  ENA (Left motor PWM)
Pin 8   ->  IN1
Pin 7   ->  IN2
Pin 10  ->  ENB (Right motor PWM)
Pin 12  ->  IN3
Pin 11  ->  IN4

L298N OUT1/OUT2 -> Left DC Motor
L298N OUT3/OUT4 -> Right DC Motor
L298N 12V       -> 9V Battery (through adapter)
L298N GND       -> Arduino GND + Battery GND
L298N 5V        -> Can power Arduino (or use separate power)
```

### TCS3200 Color Sensor
```
Arduino     TCS3200
---------   -------
A0      ->  S0
A1      ->  S1
A2      ->  S2
A3      ->  S3
Pin 4   ->  OUT
5V      ->  VCC
GND     ->  GND
OE      ->  GND (always enabled)
```

### IR Sensors
```
Arduino     IR Sensors
---------   ----------
Pin 2   ->  Left IR OUT
Pin 3   ->  Right IR OUT
5V      ->  Both IR VCC
GND     ->  Both IR GND
```

### HC-SR04 Ultrasonic
```
Arduino     HC-SR04
---------   -------
Pin 5   ->  TRIG
Pin 6   ->  ECHO
5V      ->  VCC
GND     ->  GND
```

### Horn Servo
```
Arduino     Servo
---------   -----
Pin 13  ->  Signal (orange/yellow wire)
5V      ->  VCC (red wire) - USE SEPARATE 5V IF SERVO IS LARGE
GND     ->  GND (brown/black wire)
```

---

## State Machine Flows

### CODE 1: Start -> First Reupload Point
```
[Start on BLACK]
     |
     v
LINE_FOLLOW_BLACK -- follow black line
     |
     | (see RED or GREEN = intersection)
     v
AT_INTERSECTION
     |
     v
ALIGN_TO_RED -- pivot until color sensor sees RED + IR sees line
     |
     v
LINE_FOLLOW_RED -- follow red path
     |
     | (see BLUE = box pickup zone)
     v
BOX_PICKUP_TURN -- pivot RIGHT 90deg
     |
     v
BOX_PICKUP_APPROACH -- drive forward to box
     |
     v
BOX_PICKUP_GRAB -- raise horn (0 -> 40 deg)
     |
     v
BOX_PICKUP_RETURN -- backup, pivot LEFT 90deg
     |
     v
LINE_FOLLOW_RED_WITH_BOX -- continue on red
     |
     | (see gray/purple = reupload point)
     v
STOP_REUPLOAD -- DONE, upload CODE 2
```

### CODE 2: First Reupload -> Second Reupload
```
[Continuing on RED with box]
     |
     v
LINE_FOLLOW_RED
     |
     | (US detects obstacle < 15cm)
     v
OBSTACLE_DETECTED
     |
     v
[Obstacle Avoidance Sequence - goes RIGHT]
OBS_TURN_RIGHT (90deg) -> OBS_FWD_1 -> OBS_TURN_LEFT (90deg) 
-> OBS_FWD_2 -> OBS_TURN_LEFT (90deg) -> OBS_FWD_3 
-> OBS_TURN_RIGHT (90deg) -> OBS_FIND_LINE
     |
     | (repeat for obstacle 2)
     v
LINE_FOLLOW_RED (after 2 obstacles)
     |
     | (see BLUE = dropoff zone)
     v
BOX_DROPOFF_TURN -- pivot LEFT 90deg
     |
     v
BOX_DROPOFF -- approach, lower horn, backup
     |
     v
BOX_DROPOFF_RETURN -- pivot RIGHT back to path
     |
     v
RETURN_TO_JUNCTION -- follow red back
     |
     | (see BLACK or GREEN = junction)
     v
FIND_BLACK -> FIND_GREEN -- reorient
     |
     v
LINE_FOLLOW_GREEN
     |
     | (see BLUE = second box)
     v
BOX2_PICKUP -- same sequence as CODE 1
     |
     v
LINE_FOLLOW_GREEN_BOX
     |
     | (wide line = fork)
     v
AT_FORK -- stay LEFT/straight
     |
     v
LINE_FOLLOW_RAMP -- up the ramp
     |
     | (reupload point)
     v
STOP_REUPLOAD -- DONE, upload CODE 3
```

### CODE 3: Target Navigation -> Home
```
[Placed randomly on target platform]
     |
     v
SUMO_FORWARD -- drive forward
     |
     | (IR sees edge = about to fall off)
     v
SUMO_EDGE_BACKUP -- reverse
     |
     v
SUMO_RANDOM_TURN -- random direction/duration
     |
     | (loop back to SUMO_FORWARD)
     | (until color sensor sees BLACK = center)
     v
BLACK_FOUND
     |
     v
SPIN_FIND_WALL -- rotate until US sees wall
     |
     v
GO_TO_CENTER -- drive forward a bit
     |
     v
TURN_180 -- face the ramp
     |
     v
GO_DOWN_RAMP -- drive until GREEN detected
     |
     v
LINE_FOLLOW_GREEN_DOWN
     |
     | (fork detected)
     v
AT_FORK_STRAIGHT -- continue straight
     |
     v
LINE_FOLLOW_GREEN_2
     |
     | (BLUE = dropoff)
     v
BOX_DROPOFF -- turn RIGHT, lower, backup, turn LEFT
     |
     v
LINE_FOLLOW_GREEN_3 -- to junction
     |
     | (see BLACK)
     v
FIND_BLACK -- align to black path
     |
     v
LINE_FOLLOW_BLACK_HOME
     |
     | (line lost = at start)
     v
STOP_FINISH -- COURSE COMPLETE!
```

---

## Tuning Guide

### Values You WILL Need to Tune On-Site

**Motor Timing (most critical):**
```cpp
TURN_90_TIME = 350;    // Start here, adjust until robot turns exactly 90 degrees
TURN_180_TIME = 700;   // Should be roughly 2x TURN_90_TIME
FORWARD_TO_BOX = 400;  // Distance from line to box center
BACKWARD_FROM_BOX = 350;
```

**Motor Speed:**
```cpp
SPEED_NORMAL = 150;    // Main driving speed
SPEED_SLOW = 100;      // Precise movements
SPEED_TURN = 160;      // Pivot turn speed
```

**Obstacle Avoidance (CODE 2):**
```cpp
OBS_TURN_TIME = 350;   // 90 degree turn
OBS_FORWARD_1 = 400;   // Past obstacle width
OBS_FORWARD_2 = 500;   // Past obstacle length
OBS_FORWARD_3 = 400;   // Back to line
OBSTACLE_DIST = 15;    // Detection threshold in cm
```

**Color Calibration (CRITICAL):**
Run `calibration_utility.ino` on actual course and record values.
```cpp
ColorCal CAL_BLACK = {rMin, rMax, gMin, gMax, bMin, bMax};
ColorCal CAL_RED = {rMin, rMax, gMin, gMax, bMin, bMax};
ColorCal CAL_GREEN = {rMin, rMax, gMin, gMax, bMin, bMax};
ColorCal CAL_BLUE = {rMin, rMax, gMin, gMax, bMin, bMax};
```

---

## Troubleshooting

**Robot doesn't detect line:**
- Check IR sensor connections
- Verify IR_LEFT and IR_RIGHT pins
- Some sensors are inverted (HIGH = line) - change `LOW` to `HIGH` in leftOnLine()/rightOnLine()

**Robot spins endlessly looking for color:**
- Run calibration utility
- Check color sensor is close to ground (1-2cm)
- Update CAL_xxx values with actual readings

**Motor directions wrong:**
- Swap IN1/IN2 for left motor, IN3/IN4 for right motor in code
- Or swap physical wire connections

**Turns are too short/long:**
- Adjust TURN_90_TIME (increase = more turn)
- Battery voltage affects this - retune as battery drains

**Obstacle avoidance misses the path:**
- Tune OBS_FORWARD_1, OBS_FORWARD_2, OBS_FORWARD_3
- Make OBS_FORWARD_2 longer to ensure you pass the obstacle

**Horn doesn't grab box:**
- Adjust HORN_DOWN and HORN_UP angles
- Check servo mechanical mounting
- Increase HORN_GRAB_DELAY

---

## Competition Tips

1. **Bring laptop with Arduino IDE ready** - you'll be reuploading multiple times
2. **Test each code segment individually** before the run
3. **Calibrate colors on the actual course** - lighting matters
4. **Fresh batteries** - timing depends on voltage
5. **Have backup wires** - they break easily
6. **Know your state machine** - if something goes wrong, you can explain what state it was in

Good luck!
