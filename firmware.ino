#include <Wire.h>
#include <EEPROM.h>

// ─── LED UI CONFIGURATION ───────────────────────────────────
const byte LED_RED  = 4;
const byte LED_BLUE = 11;

// ─── MOTOR DRIVER (TB6612FNG) ───────────────────────────────
const byte PWMA = 5;
const byte AIN1 = 7;
const byte AIN2 = 8;

const byte PWMB = 6;
const byte BIN1 = 9;
const byte BIN2 = 10;

// ─── QUADRATURE ENCODERS ────────────────────────────────────
const byte ENC_L_A = 2; // INT0
const byte ENC_R_A = 3; // INT1

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

void leftEncoderISR()  { leftTicks++; }
void rightEncoderISR() { rightTicks++; }

void resetTicks() {
  noInterrupts();
  leftTicks  = 0;
  rightTicks = 0;
  interrupts();
}

// ─── KINEMATICS & PHYSICAL SPECIFICATIONS ───────────────────
#define TICKS_PER_CELL 645L

float ARC_FRACTION = 0.24f; 
float TRACK_WIDTH_MM = 77.0f; 
long baseEarlyTicks = 270L; 

const long START_STEP_TICKS = 130L; 

const byte CELL_CRUISE_PWM = 180; 
const byte FAST_CRUISE_PWM = 255; 
byte FAST_CRUISE_PWM_VAR   = FAST_CRUISE_PWM; 
const byte CELL_MIN_PWM    = 55;

const byte FAST_MIN_PWM       = 110; 
const byte FAST_CORNER_PWM    = 170; 
const long FAST_ACCEL_TICKS   = 90L;
const long FAST_DECEL_TICKS   = 160L;

float TURN_RADIUS_MM = 71.0f; 

float SPEED_RATIO_OUTER; 
float SPEED_RATIO_INNER; 
long FAST_TURN_EARLY_TICKS; 
long ARC_EXIT_TICKS; 

void updateKinematics() {
  SPEED_RATIO_OUTER = 1.0f + (TRACK_WIDTH_MM / (2.0f * TURN_RADIUS_MM)); 
  SPEED_RATIO_INNER = 1.0f - (TRACK_WIDTH_MM / (2.0f * TURN_RADIUS_MM)); 
  FAST_TURN_EARLY_TICKS = (long)(baseEarlyTicks * (0.25f / ARC_FRACTION)); 
  ARC_EXIT_TICKS = (long)((ARC_FRACTION / 0.25f) * (TURN_RADIUS_MM / 180.0f) * TICKS_PER_CELL + 0.5f);
}

// ─── SENSOR PIN DEFINITIONS ─────────────────────────────────
const byte PIN_IR_FR = A0; 
const byte PIN_IR_AL = A1; 
const byte PIN_IR_AR = A2; 
const byte PIN_IR_FL = A3; 

// ─── CALIBRATION & THRESHOLDS ───────────────────────────────
const int AL_NOMINAL = 168; 
const int AR_NOMINAL = 151; 

const int AL_WALL_THRESHOLD = 125;
const int AR_WALL_THRESHOLD = 125;
const int FL_WALL_THRESHOLD = 120;
const int FR_WALL_THRESHOLD = 120;

const int FRONT_COLLISION_THRESHOLD = 400; // Hand gestures only
const int MOVE_COLLISION_THRESHOLD  = 1023; // Disabled for search diagnostics; never triggers a movement abort

const float IR_KP      = 0.22f;
const float ENCODER_KP = 0.03f; 

const unsigned long STALL_TIMEOUT_MS = 300;
const unsigned long BASE_MOVE_TIMEOUT_MS = 1500;
const unsigned long MOVE_TIMEOUT_PER_CELL_MS = 1200;

// ─── MAZE BIT-MASK DEFINITIONS ──────────────────────────────
#define WALL_N  0x01
#define WALL_E  0x02
#define WALL_S  0x04
#define WALL_W  0x08
#define VISITED 0x80

const byte DIR_MASK[4] = { WALL_N, WALL_E, WALL_S, WALL_W };
const int8_t DX[4] = { 0,  1,  0, -1 };
const int8_t DY[4] = { 1,  0, -1,  0 };

enum Heading { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum RunState { IDLE, SEARCH_GOAL, RETURN_START };

// ─── MAZE SIZE CONFIGURATION ─────────────────────────────────
const byte MAZE_SIZE = 16;
const byte MAZE_MAX  = MAZE_SIZE - 1;

uint8_t maze[MAZE_SIZE][MAZE_SIZE];
uint8_t dist[MAZE_SIZE][MAZE_SIZE];

byte currentX = 0;
byte currentY = 0;
Heading currentHeading = NORTH;
RunState currentState = IDLE;

const byte GOAL_MIN_X = 7;
const byte GOAL_MAX_X = 8;
const byte GOAL_MIN_Y = 7;
const byte GOAL_MAX_Y = 8;

Heading pathBuffer[256];
uint16_t pathLength = 0;
bool pathIsReady = false;

// ─── EEPROM MEMORY MAP ──────────────────────────────────────
const byte EEPROM_MAGIC_0 = 0x4D; // 'M'
const byte EEPROM_MAGIC_1 = 0x52; // 'R'
const byte EEPROM_VERSION = 1;
const int EEPROM_ADDR_MAGIC0 = 0;
const int EEPROM_ADDR_MAGIC1 = 1;
const int EEPROM_ADDR_VERSION = 2;
const int EEPROM_ADDR_LEN_L = 3;
const int EEPROM_ADDR_LEN_H = 4;
const int EEPROM_ADDR_PATH = 5;
const int EEPROM_ADDR_CHECKSUM_L = EEPROM_ADDR_PATH + 255;
const int EEPROM_ADDR_CHECKSUM_H = EEPROM_ADDR_CHECKSUM_L + 1;

uint8_t queue[256];
uint16_t qHead = 0;
uint16_t qTail = 0;

inline void qPush(byte x, byte y) { queue[qHead++] = (x << 4) | (y & 0x0F); }
inline void qPop(byte &x, byte &y) { uint8_t val = queue[qTail++]; x = val >> 4; y = val & 0x0F; }
inline bool qIsEmpty() { return qHead == qTail; }

// ─── ICM-20602 GYROSCOPE ────────────────────────────────────
const byte ICM_ADDR        = 0x68;
const byte ICM_PWR_MGMT_1  = 0x6B;
const byte ICM_GYRO_CONFIG = 0x1B;
const byte ICM_GYRO_ZOUT_H = 0x47;

const float GYRO_SCALE    = 32.8f;
const float GYRO_DEADBAND = 0.5f;

float angleZ = 0.0f;
float gyroZ_offset = 0.0f;
unsigned long lastGyroTime = 0;

const float STRAIGHT_KP     = 2.5f;
const float STRAIGHT_KD     = 0.08f;
const float TURN_KP         = 1.8f;
const float TURN_KD         = 0.12f;
const byte  TURN_MAX_PWM    = 145; 
const byte  TURN_MIN_PWM    = 60;  
const float ANGLE_TOLERANCE = 2.0f; 

// ─── FUNCTION PROTOTYPES ────────────────────────────────────
void initICM20602();
void calibrateGyro();
int16_t readGyroZRaw();
float updateGyro();
void brakeMotors();
bool turnToAngle(float targetAngle);
bool turnRelative(int degrees);
bool turnArc(int degrees, byte cruisePWM = CELL_CRUISE_PWM);
bool advanceCells(uint8_t cells, byte cruisePWM = CELL_CRUISE_PWM);
bool advanceTicks(long targetTicks, byte cruisePWM = FAST_CRUISE_PWM);
bool advanceTicksFast(long targetTicks, byte cruisePWM, byte startPWM, byte endPWM, bool brakeAtEnd);
int readCleanADC(byte pin);

void initMaze();
void setWall(byte x, byte y, byte direction);
void updateCurrentCellWalls();
void floodFillTarget(byte minX, byte maxX, byte minY, byte maxY);
Heading getNextMoveDirection();
bool orientTo(Heading targetDir);
bool moveOneCellForward();
void runFloodFillAutonomous();
bool prepareFastRunPath();
void runArcFastRun();
void runPivotFastRun();
void fastReturnToStart();

enum ManeuverType {
  MANEUVER_NONE,
  MANEUVER_STRAIGHT,
  MANEUVER_RIGHT_90,
  MANEUVER_LEFT_90,
  MANEUVER_RIGHT_LEFT_S,
  MANEUVER_LEFT_RIGHT_S,
  MANEUVER_RIGHT_180,
  MANEUVER_LEFT_180,
  MANEUVER_DIRECT_UTURN
};

struct MotionProfile {
  byte entryPWM;
  byte turnPWM;
  byte exitPWM;
  long entryCutTicks;
  long exitCarryTicks;
  float arcFraction;
  float radiusMM;
};

// ─── FAST-RUN MANEUVER PROFILES ───────────────────────────────
// IMPORTANT: pathBuffer/segments ALWAYS decide when a corner exists.
// A motion profile only decides HOW that already-existing corner is driven.
// The user's reported tuned standalone 90-degree turnPWM=50 and radius=65 mm
// are applied below. Verify the other PROFILE_90 values against your latest
// locally-tuned sketch before competition use.
const MotionProfile PROFILE_90 = {
  170, 195, 170,
  360L, 150L,  //375, 105
  0.24f, 47.0f
};

const MotionProfile PROFILE_S = {
  170, 180, 175,
  350L, 170L,
  0.22f, 51.0f //0.22
};

const MotionProfile PROFILE_180 = {
  160, 165, 160,
  360L, 160L,
  0.25f, 60.0f
};

// Compound profiles are intentionally OFF for the first reliability test.
// Set true only after every standalone 90-degree corner works across the maze.
const bool ENABLE_COMPOUND_PROFILES = true;

// A second corner may influence the current profile ONLY when the segment
// between the two corners is this short. This prevents a 5-cell jog from being
// misclassified as an S-turn / same-direction 180 sequence.
const uint8_t MAX_COMPOUND_GAP_CELLS = 1;

// Explicit prototype prevents Arduino's auto-prototype generator from
// placing headingDiff() before the Heading enum declaration.
int headingDiff(Heading from, Heading to);

ManeuverType detectManeuver(uint16_t idx, Heading heading);
bool executeProfiledTurn(ManeuverType m, Heading &heading, long &carriedTicks, byte &carriedPWM);
bool executeSingleArcProfile(int degrees, const MotionProfile &profile);
bool saveFastPathToEEPROM();
bool loadFastPathFromEEPROM();
void clearSavedFastPath();
void startNewSearchRun();

void handleIdleGestures();
void wiggleChassis();
void preRunCountdown();
void updateLEDState();
void handleSerialCommands();

// ─── SETUP ──────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);

  updateKinematics(); 

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  brakeMotors();

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);
  resetTicks();

  analogReference(DEFAULT);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true); 

  initICM20602();

  Serial.println(F("Power-on: Waiting 3 seconds before Gyro calibration..."));
  delay(3000);

  Serial.println(F("KEEP ROBOT STILL: Calibrating Gyro..."));
  calibrateGyro();
  lastGyroTime = micros();

  Serial.println(F("Gyro calibrated! Wait 1 second..."));
  delay(1000);

  Serial.println(F("Centering in (0,0)..."));
  advanceTicks(START_STEP_TICKS, CELL_CRUISE_PWM);

  initMaze();

  if (loadFastPathFromEEPROM()) {
    Serial.println(F("SAVED PATH FOUND in EEPROM."));
  } else {
    Serial.println(F("NO SAVED PATH."));
  }

  currentState = IDLE;
  updateLEDState();
}

// ─── MAIN LOOP ──────────────────────────────────────────────
void loop() {
  updateGyro();
  handleSerialCommands();

  if (currentState == IDLE) {
    handleIdleGestures();
  }
}

// ─── LED STATE MANAGER ──────────────────────────────────────
void updateLEDState() {
  if (pathIsReady && pathLength > 0) {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, HIGH);
  } else {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, LOW);
  }
}

// ─── HAND GESTURE UI ────────────────────────────────────────
void handleIdleGestures() {
  static unsigned long coverStartTime = 0;
  static byte currentGesture = 0; // 0=none, 1=BOTH, 2=FL, 3=FR
  
  int fl = readCleanADC(PIN_IR_FL);
  int fr = readCleanADC(PIN_IR_FR);
  
  bool flCovered = (fl > FRONT_COLLISION_THRESHOLD);
  bool frCovered = (fr > FRONT_COLLISION_THRESHOLD);
  
  byte detected = 0;
  if (flCovered && frCovered) detected = 1;
  else if (flCovered) detected = 2;
  else if (frCovered) detected = 3;
  
  if (detected != currentGesture) {
    coverStartTime = millis();
    currentGesture = detected;
  } else if (currentGesture != 0) {
    // If hand is held continuously for 1.5 seconds, execute command
    if (millis() - coverStartTime >= 1500) {
      byte actionGesture = currentGesture;
      currentGesture = 0; 
      
      if (actionGesture == 1) { // 1. BOTH SENSORS
        Serial.println(F("GESTURE: BOTH -> Wiping EEPROM..."));
        clearSavedFastPath();
        wiggleChassis();
        updateLEDState();
      } 
      else if (actionGesture == 2) { // 2. FL SENSOR ONLY
        if (pathIsReady && pathLength > 0) {
          Serial.println(F("GESTURE: FL -> Pivot Fast Run"));
          preRunCountdown();
          runPivotFastRun();
        } else {
          Serial.println(F("GESTURE: FL -> Search Run"));
          preRunCountdown();
          startNewSearchRun();
        }
        currentState = IDLE;
        updateLEDState();
      }
      else if (actionGesture == 3) { // 3. FR SENSOR ONLY
        if (pathIsReady && pathLength > 0) {
          Serial.println(F("GESTURE: FR -> Arc Fast Run"));
          preRunCountdown();
          runArcFastRun();
        } else {
          Serial.println(F("GESTURE: FR -> Search Run"));
          preRunCountdown();
          startNewSearchRun();
        }
        currentState = IDLE;
        updateLEDState();
      }
    }
  }
}

// Gives tactile feedback without leaving the cell
void wiggleChassis() {
  // Pivot left slightly
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 120); analogWrite(PWMB, 120);
  delay(80);
  // Pivot right slightly
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 120); analogWrite(PWMB, 120);
  delay(160);
  // Pivot back to center
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 120); analogWrite(PWMB, 120);
  delay(80);
  brakeMotors();
}

// Allow the user to remove their hand and stabilize the robot before launch
void preRunCountdown() {
  Serial.println(F("REMOVE HAND. Starting in 3 seconds..."));
  // Rapid LED flashing for 2.5 seconds
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED_RED, HIGH); digitalWrite(LED_BLUE, HIGH);
    delay(250);
    digitalWrite(LED_RED, LOW); digitalWrite(LED_BLUE, LOW);
    delay(250);
  }
  
  Serial.println(F("Recalibrating gyro..."));
  calibrateGyro();
  angleZ = 0.0f;
  currentX = 0; 
  currentY = 0; 
  currentHeading = NORTH;
  lastGyroTime = micros();
  digitalWrite(LED_RED, LOW); digitalWrite(LED_BLUE, LOW);
}

// ─── PIVOT FAST RUN (Stop-and-turn style) ───────────────────
void runPivotFastRun() {
  if (!pathIsReady || pathLength == 0) return;
  Serial.println(F("\n=== LAUNCHING PIVOT FAST RUN ==="));
  
  currentX = 0; 
  currentY = 0; 
  currentHeading = NORTH;
  uint16_t idx = 0;
  
  while (idx < pathLength) {
    uint8_t straightCount = 0;
    while (idx < pathLength && pathBuffer[idx] == currentHeading) {
      straightCount++;
      idx++;
    }
    
    if (straightCount > 0) {
      if (!advanceCells(straightCount, FAST_CRUISE_PWM_VAR)) {
        brakeMotors();
        return; 
      }
      
      // Update Logical Position
      for (uint8_t c = 0; c < straightCount; c++) {
        if (currentHeading == NORTH) currentY++;
        else if (currentHeading == EAST) currentX++;
        else if (currentHeading == SOUTH) currentY--;
        else if (currentHeading == WEST) currentX--;
      }
    }
    
    if (idx < pathLength) {
      Heading nextHeading = pathBuffer[idx];
      if (!orientTo(nextHeading)) {
        brakeMotors();
        return; 
      }
      brakeMotors();
      delay(150); // Small stabilization delay before sprinting again
    }
  }
  
  brakeMotors();
  Serial.println(F("Pivot Fast Run Complete!"));
  fastReturnToStart();
}

// ─── FAST RETURN TO START USING SAVED PATH IN REVERSE ───────
void fastReturnToStart() {
  if (!pathIsReady || pathLength == 0) return;

  brakeMotors();
  delay(150);
  Serial.println(F("\n=== FAST RETURN TO START ==="));

  int idx = (int)pathLength - 1;

  while (idx >= 0) {
    Heading returnHeading = (Heading)(((int)pathBuffer[idx] + 2) % 4);

    if (returnHeading != currentHeading) {
      if (!orientTo(returnHeading)) {
        brakeMotors();
        Serial.println(F("FAST RETURN STOPPED: TURN FAILED."));
        return;
      }
      brakeMotors();
      delay(150);
    }

    uint8_t straightCount = 0;
    while (idx >= 0 &&
           (Heading)(((int)pathBuffer[idx] + 2) % 4) == currentHeading) {
      straightCount++;
      idx--;
    }

    if (straightCount > 0) {
      if (!advanceCells(straightCount, FAST_CRUISE_PWM_VAR)) {
        brakeMotors();
        Serial.println(F("FAST RETURN STOPPED: STRAIGHT SEGMENT FAILED."));
        return;
      }

      for (uint8_t c = 0; c < straightCount; c++) {
        if (currentHeading == NORTH && currentY < MAZE_MAX) currentY++;
        else if (currentHeading == EAST && currentX < MAZE_MAX) currentX++;
        else if (currentHeading == SOUTH && currentY > 0) currentY--;
        else if (currentHeading == WEST && currentX > 0) currentX--;
      }
    }
  }

  brakeMotors();

  if (currentHeading != NORTH) {
    if (!orientTo(NORTH)) {
      brakeMotors();
      Serial.println(F("FAST RETURN COMPLETE, BUT NORTH ALIGNMENT FAILED."));
      return;
    }
  }

  brakeMotors();
  Serial.println(F(">>> FAST RETURN COMPLETED AT START (0,0)! <<<"));
}

// ─── SERIAL COMMAND HANDLER ─────────────────────────────────
void handleSerialCommands() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  if (cmd == '\r' || cmd == '\n' || cmd < 32) return;

  switch (cmd) {
    case 'a':
    case 'A':
      delay(8); 
      if (Serial.peek() == 'f' || Serial.peek() == 'F') {
        Serial.read(); 
        float newArcFraction = Serial.parseFloat();
        if (newArcFraction >= 0.10f && newArcFraction <= 0.40f) {
          ARC_FRACTION = newArcFraction;
          updateKinematics();
          Serial.print(F("Arc Fraction updated to: ")); Serial.println(ARC_FRACTION, 3);
        }
      } else if (Serial.peek() == 'r' || Serial.peek() == 'R') {
        Serial.read(); 
        float newRadius = Serial.parseFloat();
        float minimumSafeRadius = (TRACK_WIDTH_MM * 0.5f) + 1.0f;
        if (newRadius >= minimumSafeRadius && newRadius <= 250.0f) {
          TURN_RADIUS_MM = newRadius;
          updateKinematics();
          Serial.print(F("Arc Radius updated to: ")); Serial.print(TURN_RADIUS_MM, 1); Serial.println(F(" mm"));
        }
      } else {
        startNewSearchRun();
      }
      break;

    case 't':
    case 'T':
      delay(5);
      if (Serial.peek() == 'w' || Serial.peek() == 'W') {
        Serial.read(); 
        TRACK_WIDTH_MM = Serial.parseFloat();
        updateKinematics();
        Serial.print(F("Track Width updated: ")); Serial.println(TRACK_WIDTH_MM);
      }
      break;

    case 'p':
    case 'P':
      Serial.println(F("\n--- CURRENT PARAMETERS ---"));
      Serial.print(F("FC  Fast Cruise PWM    = ")); Serial.println(FAST_CRUISE_PWM_VAR);
      Serial.print(F("AF  Arc Fraction       = ")); Serial.println(ARC_FRACTION, 3);
      Serial.print(F("AR  Arc Radius         = ")); Serial.print(TURN_RADIUS_MM, 1); Serial.println(F(" mm"));
      Serial.print(F("TW  Track Width        = ")); Serial.print(TRACK_WIDTH_MM, 1); Serial.println(F(" mm"));
      Serial.println(F("------------------------------\n"));
      break;
      
    case 's':
    case 'S': {
      int al = readCleanADC(PIN_IR_AL);
      int ar = readCleanADC(PIN_IR_AR);
      int fl = readCleanADC(PIN_IR_FL);
      int fr = readCleanADC(PIN_IR_FR);
      Serial.print(F("AL: ")); Serial.print(al); Serial.print(F(" | AR: ")); Serial.print(ar);
      Serial.print(F(" | FL: ")); Serial.print(fl); Serial.print(F(" | FR: ")); Serial.println(fr);
      break;
    }
  }
}

// ─── EEPROM FAST-PATH PERSISTENCE ───────────────────────────
static uint16_t calculatePathChecksum(uint16_t len) {
  uint16_t sum = 0x5A5A;
  sum = (uint16_t)(sum + len);
  for (uint16_t i = 0; i < len; i++) {
    sum = (uint16_t)(sum + ((uint16_t)pathBuffer[i] + 1U) * (i + 1U));
  }
  return sum;
}

bool saveFastPathToEEPROM() {
  if (!pathIsReady || pathLength == 0 || pathLength > 255) return false;
  uint16_t checksum = calculatePathChecksum(pathLength);
  EEPROM.update(EEPROM_ADDR_MAGIC0, 0x00);
  EEPROM.update(EEPROM_ADDR_MAGIC1, 0x00);
  EEPROM.update(EEPROM_ADDR_VERSION, EEPROM_VERSION);
  EEPROM.update(EEPROM_ADDR_LEN_L, (byte)(pathLength & 0xFF));
  EEPROM.update(EEPROM_ADDR_LEN_H, (byte)((pathLength >> 8) & 0xFF));

  for (uint16_t i = 0; i < pathLength; i++) {
    EEPROM.update(EEPROM_ADDR_PATH + i, (byte)pathBuffer[i]);
  }

  EEPROM.update(EEPROM_ADDR_CHECKSUM_L, (byte)(checksum & 0xFF));
  EEPROM.update(EEPROM_ADDR_CHECKSUM_H, (byte)((checksum >> 8) & 0xFF));
  EEPROM.update(EEPROM_ADDR_MAGIC0, EEPROM_MAGIC_0);
  EEPROM.update(EEPROM_ADDR_MAGIC1, EEPROM_MAGIC_1);
  return true;
}

bool loadFastPathFromEEPROM() {
  if (EEPROM.read(EEPROM_ADDR_MAGIC0) != EEPROM_MAGIC_0) return false;
  if (EEPROM.read(EEPROM_ADDR_MAGIC1) != EEPROM_MAGIC_1) return false;
  if (EEPROM.read(EEPROM_ADDR_VERSION) != EEPROM_VERSION) return false;

  uint16_t len = (uint16_t)EEPROM.read(EEPROM_ADDR_LEN_L) |
                 ((uint16_t)EEPROM.read(EEPROM_ADDR_LEN_H) << 8);
  if (len == 0 || len > 255) return false;

  for (uint16_t i = 0; i < len; i++) {
    byte d = EEPROM.read(EEPROM_ADDR_PATH + i);
    if (d > 3) return false;
    pathBuffer[i] = (Heading)d;
  }

  pathLength = len;
  uint16_t storedChecksum = (uint16_t)EEPROM.read(EEPROM_ADDR_CHECKSUM_L) |
                            ((uint16_t)EEPROM.read(EEPROM_ADDR_CHECKSUM_H) << 8);
  uint16_t actualChecksum = calculatePathChecksum(pathLength);

  if (storedChecksum != actualChecksum) {
    pathLength = 0;
    pathIsReady = false;
    return false;
  }

  pathIsReady = true;
  return true;
}

void clearSavedFastPath() {
  EEPROM.update(EEPROM_ADDR_MAGIC0, 0x00);
  EEPROM.update(EEPROM_ADDR_MAGIC1, 0x00);
  pathLength = 0;
  pathIsReady = false;
}

void startNewSearchRun() {
  brakeMotors();
  Serial.println(F("\nNEW SEARCH REQUESTED."));
  clearSavedFastPath();
  
  angleZ = 0.0f;
  currentHeading = NORTH;
  lastGyroTime = micros();

  initMaze();
  runFloodFillAutonomous();
}

// ─── AUTONOMOUS SEARCH CONTROLLER ───────────────────────────
void runFloodFillAutonomous() {
  Serial.println(F("STARTING ROUND-TRIP EXPLORATION..."));
  currentState = SEARCH_GOAL;

  while (true) {
    updateCurrentCellWalls();

    // PHASE 1 COMPLETE: Goal reached -> Route back to start
    if (currentState == SEARCH_GOAL &&
        currentX >= GOAL_MIN_X && currentX <= GOAL_MAX_X &&
        currentY >= GOAL_MIN_Y && currentY <= GOAL_MAX_Y) {
      
      brakeMotors();
      Serial.println(F("\n>>> GOAL REACHED! RE-ROUTING BACK TO START... <<<"));
      delay(400);

      currentState = RETURN_START;
      floodFillTarget(0, 0, 0, 0);

      Heading exitDir = getNextMoveDirection();
      if (exitDir != currentHeading) {
        orientTo(exitDir);
        brakeMotors();
        delay(400);
      }
      continue;
    }

    // PHASE 2 COMPLETE: Returned to (0,0) -> Auto-Align, Pivot 180
    if (currentState == RETURN_START && currentX == 0 && currentY == 0) {
      brakeMotors();
      Serial.println(F("\n>>> RETURN COMPLETED! AUTO-ALIGNING TO NORTH... <<<"));

      turnRelative(180);
      brakeMotors();
      delay(500);

      if (prepareFastRunPath()) {
        saveFastPathToEEPROM();
      }
      break; // Exit to IDLE
    }

    if (currentState == SEARCH_GOAL) {
      floodFillTarget(GOAL_MIN_X, GOAL_MAX_X, GOAL_MIN_Y, GOAL_MAX_Y);
    } else if (currentState == RETURN_START) {
      floodFillTarget(0, 0, 0, 0);
    }

    Heading nextDir = getNextMoveDirection();

    if ((int)nextDir < 0 || (int)nextDir > 3) {
      brakeMotors();
      Serial.println(F("NO VALID FLOOD-FILL MOVE - SEARCH STOPPED SAFELY."));
      break;
    }

    if (nextDir != currentHeading) {
      if (!orientTo(nextDir)) {
        brakeMotors();
        Serial.println(F("TURN FAILED - autonomous run stopped."));
        break;
      }

      // Re-check the front after turning. This prevents a stale map decision
      // from driving directly into a wall that the turn exposed.
      int postTurnFL = readCleanADC(PIN_IR_FL);
      int postTurnFR = readCleanADC(PIN_IR_FR);
      Serial.print(F("POST-TURN FRONT FL="));
      Serial.print(postTurnFL);
      Serial.print(F(" FR="));
      Serial.println(postTurnFR);

      if (postTurnFL > FL_WALL_THRESHOLD || postTurnFR > FR_WALL_THRESHOLD) {
        Serial.println(F("POST-TURN WALL DETECTED - RECORDING WALL, REPLANNING."));
        updateCurrentCellWalls();
        continue;
      }
    }

    if (!moveOneCellForward()) {
      brakeMotors();
      Serial.println(F("CELL MOVE FAILED - SEARCH STOPPED SAFELY."));
      break;
    }

    if (Serial.available()) {
      char abortCmd = Serial.read();
      if (abortCmd == 'x' || abortCmd == 'X') {
        brakeMotors();
        Serial.println(F("Autonomous Run Aborted by User."));
        break;
      }
    }
  }
}

// ─── ADVANCE CELLS ──────────────────────────────────────────
bool advanceCells(uint8_t cells, byte cruisePWM) {
  long targetTicks = (long)cells * TICKS_PER_CELL;
  return advanceTicks(targetTicks, cruisePWM);
}

bool advanceTicks(long targetTicks, byte cruisePWM) {
  if (targetTicks <= 0) return true;

  bool completed = true;
  resetTicks();
  float targetHeading = angleZ;
  lastGyroTime = micros();

  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  const long accelTicks = 120L;
  const long decelTicks = 140L + (long)(cruisePWM * 0.4f);

  unsigned long moveStartTime = millis();
  unsigned long lastProgressTime = moveStartTime;
  long lastProgressTicks = 0;
  unsigned long moveTimeout = BASE_MOVE_TIMEOUT_MS + ((targetTicks / TICKS_PER_CELL) + 1) * MOVE_TIMEOUT_PER_CELL_MS;

  while (true) {
    if (Serial.available() && (Serial.read() == 'x' || Serial.read() == 'X')) { completed = false; break; }

    long currentL, currentR;
    noInterrupts();
    currentL = leftTicks; currentR = rightTicks;
    interrupts();

    long avgTicks = (currentL + currentR) / 2;
    long remainingTicks = targetTicks - avgTicks;

    if (remainingTicks <= 0) break;
    if (millis() - moveStartTime > moveTimeout) { completed = false; break; }

    if (avgTicks >= lastProgressTicks + 2) {
      lastProgressTicks = avgTicks;
      lastProgressTime = millis();
    } else if (millis() - lastProgressTime > STALL_TIMEOUT_MS) {
      completed = false; break;
    }

    // NOTE:
    // Do NOT use the gesture threshold here. Earlier testing showed that the
    // 400 threshold falsely aborted movement inside the maze. Wall protection
    // is handled by the pre-move/post-turn checks instead.
    if (MOVE_COLLISION_THRESHOLD < 1023) {
      int flVal = readCleanADC(PIN_IR_FL);
      int frVal = readCleanADC(PIN_IR_FR);
      if (flVal > MOVE_COLLISION_THRESHOLD || frVal > MOVE_COLLISION_THRESHOLD) {
        Serial.println(F("MOVE ABORT: MOVEMENT COLLISION THRESHOLD."));
        completed = false;
        break;
      }
    }

    int accelPWM = (avgTicks < accelTicks) ? map(avgTicks, 0, accelTicks, CELL_MIN_PWM, cruisePWM) : cruisePWM;
    int decelPWM = (remainingTicks < decelTicks) ? map(remainingTicks, 0, decelTicks, CELL_MIN_PWM, cruisePWM) : cruisePWM;
    int baseSpeed = constrain(min(accelPWM, decelPWM), CELL_MIN_PWM, cruisePWM);

    int alVal = readCleanADC(PIN_IR_AL);
    int arVal = readCleanADC(PIN_IR_AR);

    bool hasLeftWall  = (alVal > AL_WALL_THRESHOLD);
    bool hasRightWall = (arVal > AR_WALL_THRESHOLD);
    float irError = 0.0f;

    if (hasLeftWall && hasRightWall) {
      irError = (float)(alVal - AL_NOMINAL) - (float)(arVal - AR_NOMINAL);
    } else if (hasLeftWall) {
      irError = (float)(alVal - AL_NOMINAL) * 1.5f;
    } else if (hasRightWall) {
      irError = -(float)(arVal - AR_NOMINAL) * 1.5f;
    }

    float dps = updateGyro();
    float steerCorrection = ((STRAIGHT_KP * (targetHeading - angleZ)) - (STRAIGHT_KD * dps)) - (IR_KP * irError) + (ENCODER_KP * (float)(currentL - currentR));

    analogWrite(PWMA, constrain((int)(baseSpeed - steerCorrection), 0, 255));
    analogWrite(PWMB, constrain((int)(baseSpeed + steerCorrection), 0, 255));
  }

  brakeMotors();
  return completed;
}

// ─── FAST-RUN STRAIGHT WITH VELOCITY CARRY-THROUGH ──────────
bool advanceTicksFast(long targetTicks, byte cruisePWM, byte startPWM, byte endPWM, bool brakeAtEnd) {
  if (targetTicks <= 0) { if (brakeAtEnd) brakeMotors(); return true; }

  bool completed = true;
  resetTicks();
  float targetHeading = angleZ;
  lastGyroTime = micros();

  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  startPWM = (byte)constrain(startPWM, FAST_MIN_PWM, 255);
  endPWM   = (byte)constrain(endPWM,   FAST_MIN_PWM, 255);
  cruisePWM = (byte)constrain(cruisePWM, max(startPWM, endPWM), 255);

  unsigned long moveStartTime = millis();
  unsigned long lastProgressTime = moveStartTime;
  long lastProgressTicks = 0;
  unsigned long moveTimeout = BASE_MOVE_TIMEOUT_MS + ((targetTicks / TICKS_PER_CELL) + 1) * MOVE_TIMEOUT_PER_CELL_MS;

  while (true) {
    if (Serial.available() && (Serial.read() == 'x' || Serial.read() == 'X')) { completed = false; break; }

    long currentL, currentR;
    noInterrupts();
    currentL = leftTicks; currentR = rightTicks;
    interrupts();

    long avgTicks = (currentL + currentR) / 2;
    long remainingTicks = targetTicks - avgTicks;

    if (remainingTicks <= 0) break;
    if (millis() - moveStartTime > moveTimeout) { completed = false; break; }

    if (avgTicks >= lastProgressTicks + 2) {
      lastProgressTicks = avgTicks;
      lastProgressTime = millis();
    } else if (millis() - lastProgressTime > STALL_TIMEOUT_MS) {
      completed = false; break;
    }

    // IMPORTANT:
    // Do not use FRONT_COLLISION_THRESHOLD here. That threshold is intentionally
    // reserved for hand gestures. The old version used 400 here, which could
    // make the ARC fast run abort as soon as the front IR saw the maze wall
    // near a corner. That matches the old "same-cell" failure symptom.
    //
    // Arc fast-run safety is handled by the pre-turn/post-turn geometry and
    // encoder/gyro motion control instead.
    int accelPWM = cruisePWM;
    if (avgTicks < FAST_ACCEL_TICKS) accelPWM = map(avgTicks, 0, FAST_ACCEL_TICKS, startPWM, cruisePWM);

    int decelPWM = cruisePWM;
    if (remainingTicks < FAST_DECEL_TICKS) decelPWM = map(remainingTicks, 0, FAST_DECEL_TICKS, endPWM, cruisePWM);

    int baseSpeed = constrain(min(accelPWM, decelPWM), min(startPWM, endPWM), cruisePWM);

    int alVal = readCleanADC(PIN_IR_AL);
    int arVal = readCleanADC(PIN_IR_AR);
    bool hasLeftWall  = (alVal > AL_WALL_THRESHOLD);
    bool hasRightWall = (arVal > AR_WALL_THRESHOLD);
    float irError = 0.0f;

    if (hasLeftWall && hasRightWall) irError = (float)(alVal - AL_NOMINAL) - (float)(arVal - AR_NOMINAL);
    else if (hasLeftWall) irError = (float)(alVal - AL_NOMINAL) * 1.5f;
    else if (hasRightWall) irError = -(float)(arVal - AR_NOMINAL) * 1.5f;

    float dps = updateGyro();
    float steerCorrection = ((STRAIGHT_KP * (targetHeading - angleZ)) - (STRAIGHT_KD * dps)) - (IR_KP * irError) + (ENCODER_KP * (float)(currentL - currentR));

    analogWrite(PWMA, constrain((int)(baseSpeed - steerCorrection), 0, 255));
    analogWrite(PWMB, constrain((int)(baseSpeed + steerCorrection), 0, 255));
  }

  if (!completed || brakeAtEnd) brakeMotors();
  else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMA, endPWM); analogWrite(PWMB, endPWM);
  }

  return completed;
}

// ─── CONTINUOUS ARC TURN ROUTINE ────────────────────────────
bool turnArc(int degrees, byte cruisePWM) {
  float initialAngle = angleZ;
  bool isLeftTurn = (degrees > 0);

  Serial.print(F("ARC START: degrees="));
  Serial.print(degrees);
  Serial.print(F(" angle="));
  Serial.print(initialAngle, 2);
  Serial.print(F(" cell=("));
  Serial.print(currentX);
  Serial.print(F(","));
  Serial.print(currentY);
  Serial.println(F(")"));
  bool reachedTarget = false;

  int targetLeftPWM  = isLeftTurn ? constrain((int)(cruisePWM * SPEED_RATIO_INNER), CELL_MIN_PWM, 255) : constrain((int)(cruisePWM * SPEED_RATIO_OUTER), CELL_MIN_PWM, 255);
  int targetRightPWM = isLeftTurn ? constrain((int)(cruisePWM * SPEED_RATIO_OUTER), CELL_MIN_PWM, 255) : constrain((int)(cruisePWM * SPEED_RATIO_INNER), CELL_MIN_PWM, 255);

  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  unsigned long turnStartTime = millis();
  lastGyroTime = micros();

  while (millis() - turnStartTime < 2000) {
    if (Serial.available() && (Serial.read() == 'x' || Serial.read() == 'X')) { brakeMotors(); return false; }

    float dps = updateGyro();
    float angleError = (360.0f * ARC_FRACTION) - abs(angleZ - initialAngle);

    if (angleError <= ANGLE_TOLERANCE) { reachedTarget = true; break; }

    float taper = constrain(angleError / 45.0f, 0.60f, 1.0f);
    int leftPWM  = (int)(targetLeftPWM * taper);
    int rightPWM = (int)(targetRightPWM * taper);

    if (angleError < 15.0f && abs(dps) > 140.0f) { leftPWM = (int)(leftPWM * 0.85f); rightPWM = (int)(rightPWM * 0.85f); }

    analogWrite(PWMA, constrain(leftPWM, CELL_MIN_PWM, 255));
    analogWrite(PWMB, constrain(rightPWM, CELL_MIN_PWM, 255));
  }

  if (!reachedTarget) {
    brakeMotors();
    Serial.print(F("ARC FAILED: timeout angle="));
    Serial.println(angleZ, 2);
    return false;
  }

  Serial.print(F("ARC COMPLETE: angle="));
  Serial.println(angleZ, 2);

  if (degrees == 90) currentHeading = (Heading)((currentHeading + 3) % 4);
  else if (degrees == -90) currentHeading = (Heading)((currentHeading + 1) % 4);

  return true;
}

// ─── LOOKAHEAD FAST-RUN HELPERS ─────────────────────────────
int headingDiff(Heading from, Heading to) {
  int d = (int)to - (int)from;
  while (d > 2) d -= 4;
  while (d < -2) d += 4;
  return d;
}

ManeuverType detectManeuver(uint16_t idx, Heading heading) {
  if (idx >= pathLength) return MANEUVER_NONE;

  Heading d0 = pathBuffer[idx];
  if (d0 == heading) return MANEUVER_STRAIGHT;

  int first = headingDiff(heading, d0);

  if (first == 1 || first == -1) {
    // Count the ACTUAL number of cells in the intermediate segment.
    uint16_t j = idx;
    uint8_t middleCells = 0;

    while (j < pathLength && pathBuffer[j] == d0) {
      if (middleCells < 255) middleCells++;
      j++;
    }

    // Compound classification is optional and distance limited.
    // It can NEVER remove a corner; runArcFastRun() still executes every
    // heading change directly from pathBuffer.
    if (ENABLE_COMPOUND_PROFILES &&
        middleCells <= MAX_COMPOUND_GAP_CELLS &&
        j < pathLength) {

      Heading d1 = pathBuffer[j];
      int second = headingDiff(d0, d1);

      if (first == 1  && second == -1) return MANEUVER_RIGHT_LEFT_S;
      if (first == -1 && second == 1)  return MANEUVER_LEFT_RIGHT_S;
      if (first == 1  && second == 1)  return MANEUVER_RIGHT_180;
      if (first == -1 && second == -1) return MANEUVER_LEFT_180;
    }

    return (first == 1) ? MANEUVER_RIGHT_90 : MANEUVER_LEFT_90;
  }

  if (first == 2 || first == -2) return MANEUVER_DIRECT_UTURN;

  return MANEUVER_NONE;
}

bool executeSingleArcProfile(int degrees, const MotionProfile &profile) {
  // Temporarily apply only the geometric values used by turnArc().
  float oldFraction = ARC_FRACTION;
  float oldRadius = TURN_RADIUS_MM;

  ARC_FRACTION = profile.arcFraction;
  TURN_RADIUS_MM = profile.radiusMM;
  updateKinematics();

  bool ok = turnArc(degrees, profile.turnPWM);

  ARC_FRACTION = oldFraction;
  TURN_RADIUS_MM = oldRadius;
  updateKinematics();
  return ok;
}

bool executeProfiledTurn(ManeuverType m, Heading &heading, long &carriedTicks, byte &carriedPWM) {
  bool ok = false;

  if (m == MANEUVER_RIGHT_90) {
    ok = executeSingleArcProfile(-90, PROFILE_90);
    if (ok) {
      carriedTicks = PROFILE_90.exitCarryTicks;
      carriedPWM = PROFILE_90.exitPWM;
    }
    return ok;
  }

  if (m == MANEUVER_LEFT_90) {
    ok = executeSingleArcProfile(90, PROFILE_90);
    if (ok) {
      carriedTicks = PROFILE_90.exitCarryTicks;
      carriedPWM = PROFILE_90.exitPWM;
    }
    return ok;
  }

  // S-turn and 180 profiles are executed as two connected arcs.
  // The path parser consumes the short middle segment naturally on the next loop,
  // but the velocity is deliberately carried through so the robot does not stop.
  if (m == MANEUVER_RIGHT_LEFT_S) {
    ok = executeSingleArcProfile(-90, PROFILE_S);
    if (ok) {
      carriedTicks = PROFILE_S.exitCarryTicks;
      carriedPWM = PROFILE_S.exitPWM;
    }
    return ok;
  }

  if (m == MANEUVER_LEFT_RIGHT_S) {
    ok = executeSingleArcProfile(90, PROFILE_S);
    if (ok) {
      carriedTicks = PROFILE_S.exitCarryTicks;
      carriedPWM = PROFILE_S.exitPWM;
    }
    return ok;
  }

  if (m == MANEUVER_RIGHT_180) {
    // This is the FIRST 90-degree corner of an upcoming U-shaped pair.
    // Do not pivot 180 here: the path still contains the intermediate cell.
    ok = executeSingleArcProfile(-90, PROFILE_180);
    if (ok) {
      carriedTicks = PROFILE_180.exitCarryTicks;
      carriedPWM = PROFILE_180.exitPWM;
    }
    return ok;
  }

  if (m == MANEUVER_LEFT_180) {
    ok = executeSingleArcProfile(90, PROFILE_180);
    if (ok) {
      carriedTicks = PROFILE_180.exitCarryTicks;
      carriedPWM = PROFILE_180.exitPWM;
    }
    return ok;
  }

  if (m == MANEUVER_DIRECT_UTURN) {
    brakeMotors();
    delay(30);
    ok = turnRelative(180);
    if (ok) {
      carriedTicks = 0;
      carriedPWM = FAST_MIN_PWM;
    }
    return ok;
  }

  return false;
}

// ─── SEGMENT-BASED ARC FAST RUN ──────────────────────────────
// IMPORTANT DESIGN RULE:
// pathBuffer decides WHEN a turn exists and WHICH DIRECTION it goes.
// Profiles only control HOW that physical turn is driven.
// This prevents lookahead/profile classification from skipping a corner.
void runArcFastRun() {
  if (!pathIsReady || pathLength == 0) return;

  Serial.println(F("\n=== LAUNCHING SEGMENT-BASED ARC FAST RUN! ==="));

  currentX = 0;
  currentY = 0;
  currentHeading = NORTH;

  uint16_t idx = 0;

  // Distance/speed already covered by the previous arc into the new segment.
  long carriedTicks = 0;
  byte carriedPWM = FAST_MIN_PWM;

  while (idx < pathLength) {
    // ----------------------------------------------------------
    // 1. Build ONE segment directly from pathBuffer.
    //    Example: E,E,E,N,N,W  ->  E x3, N x2, W x1
    // ----------------------------------------------------------
    Heading segmentHeading = pathBuffer[idx];
    uint16_t segmentStart = idx;
    uint8_t segmentCells = 0;

    while (idx < pathLength && pathBuffer[idx] == segmentHeading) {
      segmentCells++;
      idx++;
    }

    Serial.print(F("\nSEGMENT startIdx="));
    Serial.print(segmentStart);
    Serial.print(F(" heading="));
    Serial.print((int)segmentHeading);
    Serial.print(F(" cells="));
    Serial.println(segmentCells);

    // ----------------------------------------------------------
    // 2. Turn INTO this segment if needed.
    //    This turn is mandatory because pathBuffer changed heading.
    //    No lookahead classifier is allowed to remove/skip it.
    // ----------------------------------------------------------
    if (segmentHeading != currentHeading) {
      int diff = headingDiff(currentHeading, segmentHeading);
      bool turnOK = false;

      Serial.print(F("TURN REQUIRED: H="));
      Serial.print((int)currentHeading);
      Serial.print(F(" -> "));
      Serial.print((int)segmentHeading);
      Serial.print(F(" diff="));
      Serial.println(diff);

      ManeuverType profileType = detectManeuver(segmentStart, currentHeading);
      const MotionProfile* activeProfile = &PROFILE_90;

      if (ENABLE_COMPOUND_PROFILES) {
        if (profileType == MANEUVER_RIGHT_LEFT_S ||
            profileType == MANEUVER_LEFT_RIGHT_S) {
          activeProfile = &PROFILE_S;
        }
        else if (profileType == MANEUVER_RIGHT_180 ||
                 profileType == MANEUVER_LEFT_180) {
          activeProfile = &PROFILE_180;
        }
      }

      Serial.print(F(" profileType="));
      Serial.print((int)profileType);
      Serial.print(F(" compound="));
      Serial.println(ENABLE_COMPOUND_PROFILES ? 1 : 0);

      if (diff == 1 || diff == -3) {
        // Mandatory RIGHT 90 degrees from pathBuffer.
        turnOK = executeSingleArcProfile(-90, *activeProfile);
        if (turnOK) {
          carriedTicks = activeProfile->exitCarryTicks;
          carriedPWM = activeProfile->exitPWM;
        }
      }
      else if (diff == -1 || diff == 3) {
        // Mandatory LEFT 90 degrees from pathBuffer.
        turnOK = executeSingleArcProfile(90, *activeProfile);
        if (turnOK) {
          carriedTicks = activeProfile->exitCarryTicks;
          carriedPWM = activeProfile->exitPWM;
        }
      }
      else if (diff == 2 || diff == -2) {
        // True immediate U-turn in the saved path.
        brakeMotors();
        delay(40);
        turnOK = turnRelative(180);
        if (turnOK) {
          carriedTicks = 0;
          carriedPWM = FAST_MIN_PWM;
        }
      }
      else {
        turnOK = false;
      }

      if (!turnOK) {
        brakeMotors();
        Serial.println(F("SEGMENT FAST RUN STOPPED: TURN FAILED."));
        return;
      }

      // turnArc()/turnRelative() already updates currentHeading.
      Serial.print(F("TURN COMPLETE. NEW H="));
      Serial.println((int)currentHeading);
    }

    // ----------------------------------------------------------
    // 3. Determine whether another segment follows.
    //    If yes, its heading change DEFINES the next turn.
    // ----------------------------------------------------------
    bool hasNextSegment = (idx < pathLength);
    Heading nextHeading = hasNextSegment ? pathBuffer[idx] : currentHeading;
    int nextDiff = hasNextSegment ? headingDiff(segmentHeading, nextHeading) : 0;

    bool nextIs90 = hasNextSegment &&
                    (nextDiff == 1 || nextDiff == -1 || nextDiff == 3 || nextDiff == -3);
    bool nextIsUTurn = hasNextSegment && (nextDiff == 2 || nextDiff == -2);

    // ----------------------------------------------------------
    // 4. Compute physical straight distance for THIS segment.
    //
    //    logical cell distance
    //      - distance already covered while exiting previous arc
    //      - distance reserved for entering the NEXT arc
    // ----------------------------------------------------------
    long rawTicks = (long)segmentCells * TICKS_PER_CELL;
    long driveTicks = rawTicks;

    long carryUsed = carriedTicks;
    if (carryUsed > 0) {
      if (driveTicks > carryUsed) driveTicks -= carryUsed;
      else driveTicks = 20L;
    }
    carriedTicks = 0;

    long entryCut = 0;
    byte endPWM = FAST_MIN_PWM;

    if (nextIs90) {
      const MotionProfile* nextProfile = &PROFILE_90;

      // idx is the first pathBuffer cell of the next segment. The same
      // distance-limited classifier chooses only its geometry/speed profile;
      // the existence/direction of the next turn still comes from pathBuffer.
      if (ENABLE_COMPOUND_PROFILES) {
        ManeuverType nextType = detectManeuver(idx, segmentHeading);
        if (nextType == MANEUVER_RIGHT_LEFT_S ||
            nextType == MANEUVER_LEFT_RIGHT_S) {
          nextProfile = &PROFILE_S;
        }
        else if (nextType == MANEUVER_RIGHT_180 ||
                 nextType == MANEUVER_LEFT_180) {
          nextProfile = &PROFILE_180;
        }
      }

      entryCut = nextProfile->entryCutTicks;
      endPWM = nextProfile->entryPWM;
    }
    else if (nextIsUTurn) {
      // Stop before a true U-turn; do not use arc-entry cut.
      entryCut = 0;
      endPWM = FAST_MIN_PWM;
    }

    if (entryCut > 0) {
      if (driveTicks > entryCut + 20L) driveTicks -= entryCut;
      else driveTicks = 20L;
    }

    bool brakeAtEnd = !hasNextSegment || nextIsUTurn;

    Serial.print(F("RAW ticks="));
    Serial.print(rawTicks);
    Serial.print(F(" carryUsed="));
    Serial.print(carryUsed);
    Serial.print(F(" nextH="));
    Serial.print((int)nextHeading);
    Serial.print(F(" entryCut="));
    Serial.print(entryCut);
    Serial.print(F(" DRIVE ticks="));
    Serial.println(driveTicks);

    // ----------------------------------------------------------
    // 5. Drive this segment.
    // ----------------------------------------------------------
    if (driveTicks > 0) {
      if (!advanceTicksFast(driveTicks,
                            FAST_CRUISE_PWM_VAR,
                            carriedPWM,
                            endPWM,
                            brakeAtEnd)) {
        brakeMotors();
        Serial.println(F("SEGMENT FAST RUN STOPPED: STRAIGHT SEGMENT FAILED."));
        return;
      }
    }

    carriedPWM = endPWM;

    // ----------------------------------------------------------
    // 6. Update logical maze position by FULL cells.
    //    The physical shortening is only arc geometry; it must not
    //    change the pathBuffer/cell accounting.
    // ----------------------------------------------------------
    for (uint8_t c = 0; c < segmentCells; c++) {
      if (segmentHeading == NORTH && currentY < MAZE_MAX) currentY++;
      else if (segmentHeading == EAST && currentX < MAZE_MAX) currentX++;
      else if (segmentHeading == SOUTH && currentY > 0) currentY--;
      else if (segmentHeading == WEST && currentX > 0) currentX--;
    }

    Serial.print(F("LOGICAL CELL NOW ("));
    Serial.print(currentX);
    Serial.print(F(","));
    Serial.print(currentY);
    Serial.println(F(")"));

    // Do NOT manually consume the next turn here.
    // idx already points at the first cell of the next segment.
    // On the next loop that segment is built, then its mandatory turn
    // is executed before its straight distance is driven.
  }

  brakeMotors();
  Serial.println(F("\n>>> SEGMENT-BASED FAST RUN COMPLETED SUCCESSFULLY! <<<"));

  // Keep the return-to-start feature from the previous version.
  fastReturnToStart();
}

// ─── FLOODFILL PATH PREPARATION ──────────────────────────────
bool prepareFastRunPath() {
  currentX = 0; currentY = 0; currentHeading = NORTH;

  for (byte x = 0; x < MAZE_SIZE; x++) {
    for (byte y = 0; y < MAZE_SIZE; y++) { dist[x][y] = 255; }
  }

  qHead = 0; qTail = 0;
  for (byte tx = GOAL_MIN_X; tx <= GOAL_MAX_X; tx++) {
    for (byte ty = GOAL_MIN_Y; ty <= GOAL_MAX_Y; ty++) {
      dist[tx][ty] = 0; qPush(tx, ty);
    }
  }

  while (!qIsEmpty()) {
    byte x, y; qPop(x, y);
    uint8_t currentDist = dist[x][y];

    for (byte d = 0; d < 4; d++) {
      if (!(maze[x][y] & DIR_MASK[d])) {
        byte nx = x + DX[d]; byte ny = y + DY[d];
        if (nx < MAZE_SIZE && ny < MAZE_SIZE) {
          if ((maze[nx][ny] & VISITED) || (nx >= GOAL_MIN_X && nx <= GOAL_MAX_X && ny >= GOAL_MIN_Y && ny <= GOAL_MAX_Y)) {
            if (dist[nx][ny] > currentDist + 1) { dist[nx][ny] = currentDist + 1; qPush(nx, ny); }
          }
        }
      }
    }
  }

  byte simX = 0; byte simY = 0; Heading simHeading = NORTH; pathLength = 0;

  while (!(simX >= GOAL_MIN_X && simX <= GOAL_MAX_X && simY >= GOAL_MIN_Y && simY <= GOAL_MAX_Y)) {
    uint8_t minDist = 255; Heading bestDir = simHeading;

    byte checkOrder[4] = { simHeading, (byte)((simHeading + 1) % 4), (byte)((simHeading + 3) % 4), (byte)((simHeading + 2) % 4) };

    for (byte i = 0; i < 4; i++) {
      byte d = checkOrder[i];
      if (!(maze[simX][simY] & DIR_MASK[d])) {
        byte nx = simX + DX[d]; byte ny = simY + DY[d];
        if (nx < MAZE_SIZE && ny < MAZE_SIZE) {
          if (dist[nx][ny] < minDist) { minDist = dist[nx][ny]; bestDir = (Heading)d; }
        }
      }
    }

    if (minDist == 255 || pathLength >= 255) { pathIsReady = false; return false; }

    pathBuffer[pathLength++] = bestDir;
    simX += DX[bestDir]; simY += DY[bestDir]; simHeading = bestDir;
  }

  pathIsReady = true; return true;
}

// ─── 3-SAMPLE FAST MEDIAN ADC FILTER ────────────────────────
int readCleanADC(byte pin) {
  analogRead(pin); delayMicroseconds(40);
  int a = analogRead(pin); int b = analogRead(pin); int c = analogRead(pin);
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

// ─── MAZE LOGIC ─────────────────────────────────────────────
void initMaze() {
  memset(maze, 0, sizeof(maze));
  for (byte i = 0; i < MAZE_SIZE; i++) { maze[i][MAZE_MAX] |= WALL_N; maze[MAZE_MAX][i] |= WALL_E; maze[i][0] |= WALL_S; maze[0][i] |= WALL_W; }
  maze[0][0] |= (WALL_W | WALL_S | VISITED);
  currentX = 0; currentY = 0; currentHeading = NORTH; currentState = SEARCH_GOAL; pathIsReady = false;
  updateCurrentCellWalls(); floodFillTarget(GOAL_MIN_X, GOAL_MAX_X, GOAL_MIN_Y, GOAL_MAX_Y);
}

void setWall(byte x, byte y, byte direction) {
  maze[x][y] |= DIR_MASK[direction];
  switch (direction) {
    case NORTH: if (y < MAZE_MAX) maze[x][y + 1] |= WALL_S; break;
    case EAST:  if (x < MAZE_MAX) maze[x + 1][y] |= WALL_W; break;
    case SOUTH: if (y > 0) maze[x][y - 1] |= WALL_N; break;
    case WEST:  if (x > 0) maze[x - 1][y] |= WALL_E; break;
  }
}

// A wall is only ever added to maze[][] by setWall() -- nothing ever clears
// a bit once it's set. That makes a single noisy/borderline ADC reading
// permanently dangerous: one false positive (vibration settling after a
// turn, glare, a marginal threshold crossing) can seal off a route the
// robot needs later, with no way to recover except a full EEPROM wipe.
//
// To guard against that, a candidate wall must be confirmed by a second,
// independent read taken WALL_CONFIRM_DELAY_MS later before it is trusted.
// This only costs time on borderline cells (most reads are clearly open or
// clearly walled and settle immediately); it never runs while the motors
// are moving, since updateCurrentCellWalls() is always called with the
// robot stationary.
const unsigned long WALL_CONFIRM_DELAY_MS = 15;

bool confirmedWall(int firstVal, byte pin, int threshold, const __FlashStringHelper* label) {
  if (firstVal <= threshold) return false;
  delay(WALL_CONFIRM_DELAY_MS);
  int secondVal = readCleanADC(pin);
  bool confirmed = (secondVal > threshold);
  if (!confirmed) {
    Serial.print(F("WALL REJECTED (unconfirmed) "));
    Serial.print(label);
    Serial.print(F(": first="));
    Serial.print(firstVal);
    Serial.print(F(" second="));
    Serial.println(secondVal);
  }
  return confirmed;
}

void updateCurrentCellWalls() {
  int alVal = readCleanADC(PIN_IR_AL);
  int arVal = readCleanADC(PIN_IR_AR);
  int flVal = readCleanADC(PIN_IR_FL);
  int frVal = readCleanADC(PIN_IR_FR);

  bool leftWall    = confirmedWall(alVal, PIN_IR_AL, AL_WALL_THRESHOLD, F("AL"));
  bool rightWall   = confirmedWall(arVal, PIN_IR_AR, AR_WALL_THRESHOLD, F("AR"));
  bool frontWallFL = confirmedWall(flVal, PIN_IR_FL, FL_WALL_THRESHOLD, F("FL"));
  bool frontWallFR = confirmedWall(frVal, PIN_IR_FR, FR_WALL_THRESHOLD, F("FR"));
  bool frontWall   = frontWallFL || frontWallFR;

  maze[currentX][currentY] |= VISITED;

  if (frontWall) setWall(currentX, currentY, currentHeading);
  if (rightWall) setWall(currentX, currentY, (currentHeading + 1) % 4);
  if (leftWall) setWall(currentX, currentY, (currentHeading + 3) % 4);

  Serial.print(F("MAP ("));
  Serial.print(currentX);
  Serial.print(F(","));
  Serial.print(currentY);
  Serial.print(F(") H="));
  Serial.print((int)currentHeading);
  Serial.print(F("  AL=")); Serial.print(alVal);
  Serial.print(F(" AR=")); Serial.print(arVal);
  Serial.print(F(" FL=")); Serial.print(flVal);
  Serial.print(F(" FR=")); Serial.print(frVal);
  Serial.print(F("  L/R/F="));
  Serial.print(leftWall ? 1 : 0);
  Serial.print(F("/"));
  Serial.print(rightWall ? 1 : 0);
  Serial.print(F("/"));
  Serial.println(frontWall ? 1 : 0);
}

void floodFillTarget(byte minX, byte maxX, byte minY, byte maxY) {
  for (byte x = 0; x < MAZE_SIZE; x++) { for (byte y = 0; y < MAZE_SIZE; y++) { dist[x][y] = 255; } }
  qHead = 0; qTail = 0;

  for (byte tx = minX; tx <= maxX; tx++) {
    for (byte ty = minY; ty <= maxY; ty++) { dist[tx][ty] = 0; qPush(tx, ty); }
  }

  while (!qIsEmpty()) {
    byte x, y; qPop(x, y);
    uint8_t currentDist = dist[x][y];

    for (byte d = 0; d < 4; d++) {
      if (!(maze[x][y] & DIR_MASK[d])) {
        int nx = (int)x + DX[d];
        int ny = (int)y + DY[d];

        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
          // A route is only considered open when BOTH cells agree that the
          // shared edge is open. This prevents a stale/asymmetric wall bit
          // from making flood-fill drive through a known wall.
          byte opposite = (d + 2) % 4;
          if (!(maze[nx][ny] & DIR_MASK[opposite])) {
            if (dist[nx][ny] > currentDist + 1) {
              dist[nx][ny] = currentDist + 1;
              qPush((byte)nx, (byte)ny);
            }
          }
        }
      }
    }
  }
}

Heading getNextMoveDirection() {
  uint8_t minDist = 255;
  Heading bestDir = currentHeading;
  bool found = false;

  byte checkOrder[4] = {
    currentHeading,
    (byte)((currentHeading + 1) % 4),
    (byte)((currentHeading + 3) % 4),
    (byte)((currentHeading + 2) % 4)
  };

  Serial.print(F("FLOOD ("));
  Serial.print(currentX);
  Serial.print(F(","));
  Serial.print(currentY);
  Serial.print(F(") H="));
  Serial.println((int)currentHeading);

  for (byte i = 0; i < 4; i++) {
    byte d = checkOrder[i];

    Serial.print(F("  D"));
    Serial.print((int)d);
    Serial.print(F(": "));

    if (maze[currentX][currentY] & DIR_MASK[d]) {
      Serial.println(F("WALL"));
      continue;
    }

    int nx = (int)currentX + DX[d];
    int ny = (int)currentY + DY[d];

    if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) {
      Serial.println(F("EDGE"));
      continue;
    }

    Serial.print(F("dist="));
    Serial.println(dist[nx][ny]);

    if (dist[nx][ny] < minDist) {
      minDist = dist[nx][ny];
      bestDir = (Heading)d;
      found = true;
    }
  }

  if (!found || minDist == 255) {
    Serial.println(F("!!! NO VALID FLOOD-FILL MOVE !!!"));
    return (Heading)255;
  }

  Serial.print(F("NEXT="));
  Serial.print((int)bestDir);
  Serial.print(F(" DIST="));
  Serial.println(minDist);

  return bestDir;
}

bool orientTo(Heading targetDir) {
  int diff = (int)targetDir - (int)currentHeading;
  if (diff == 1 || diff == -3) return turnRelative(-90);
  else if (diff == -1 || diff == 3) return turnRelative(90);
  else if (diff == 2 || diff == -2) return turnRelative(180);
  return true;
}

bool moveOneCellForward() {
  Serial.print(F("MOVE FROM ("));
  Serial.print(currentX);
  Serial.print(F(","));
  Serial.print(currentY);
  Serial.print(F(") heading="));
  Serial.println((int)currentHeading);

  if (!advanceCells(1, CELL_CRUISE_PWM)) {
    Serial.println(F("MOVE FAILED: advanceCells() aborted."));
    updateCurrentCellWalls();
    return false;
  }

  switch (currentHeading) {
    case NORTH: if (currentY < MAZE_MAX) currentY++; break;
    case EAST:  if (currentX < MAZE_MAX) currentX++; break;
    case SOUTH: if (currentY > 0) currentY--; break;
    case WEST:  if (currentX > 0) currentX--; break;
  }

  updateCurrentCellWalls();

  Serial.print(F("ARRIVED AT ("));
  Serial.print(currentX);
  Serial.print(F(","));
  Serial.print(currentY);
  Serial.println(F(")"));

  return true;
}

// ─── PIVOT TURNS ────────────────────────────────────────────
bool turnRelative(int degrees) {
  float targetAngle = angleZ + degrees;
  if (!turnToAngle(targetAngle)) return false;

  if (degrees == 90) currentHeading = (Heading)((currentHeading + 3) % 4);
  else if (degrees == -90) currentHeading = (Heading)((currentHeading + 1) % 4);
  else if (degrees == 180 || degrees == -180) currentHeading = (Heading)((currentHeading + 2) % 4);

  return true;
}

bool turnToAngle(float targetAngle) {
  unsigned long turnStartTime = millis();
  while (millis() - turnStartTime < 2500) {
    float dps = updateGyro();
    float error = targetAngle - angleZ;

    if (abs(error) <= ANGLE_TOLERANCE && abs(dps) < 20.0f) { brakeMotors(); return true; }

    float output = (TURN_KP * error) - (TURN_KD * dps);
    int turnPWM = constrain((int)abs(output), TURN_MIN_PWM, TURN_MAX_PWM);

    if (output > 0) {
      digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    } else {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWMA, turnPWM); analogWrite(PWMB, turnPWM);
  }
  brakeMotors(); return false;
}

// ─── ICM-20602 FUNCTIONS ────────────────────────────────────
void initICM20602() {
  Wire.beginTransmission(ICM_ADDR); Wire.write(ICM_PWR_MGMT_1); Wire.write(0x01); Wire.endTransmission();
  Wire.beginTransmission(ICM_ADDR); Wire.write(ICM_GYRO_CONFIG); Wire.write(0x10); Wire.endTransmission();
}

void calibrateGyro() {
  delay(500);
  long sum = 0; const int SAMPLES = 500;
  for (int i = 0; i < SAMPLES; i++) { sum += readGyroZRaw(); delay(2); }
  gyroZ_offset = (float)sum / (float)SAMPLES;
}

int16_t readGyroZRaw() {
  Wire.beginTransmission(ICM_ADDR); Wire.write(ICM_GYRO_ZOUT_H); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) return (Wire.read() << 8) | Wire.read();
  return 0;
}

float updateGyro() {
  unsigned long now = micros(); float dt = (now - lastGyroTime) * 0.000001f; lastGyroTime = now;
  int16_t rawZ = readGyroZRaw(); float dps = (rawZ - gyroZ_offset) / GYRO_SCALE;
  if (abs(dps) > GYRO_DEADBAND) angleZ += dps * dt;
  return dps;
}

void brakeMotors() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}
