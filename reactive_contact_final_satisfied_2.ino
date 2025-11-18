#include <Servo.h>

// ====================================================================
// 1. PIN DEFINITIONS & CONSTANTS
// ====================================================================
// --- Sensor Pins ---
#define TRIG_PIN 8
#define ECHO_PIN 9
#define SERVO_PIN 10
// --- Motor Driver Pins (TB6612FNG) ---
#define FL_IN1 30
#define FL_IN2 31
#define PWMA 2 
#define FR_IN1 24
#define FR_IN2 25
#define PWMB 3 
#define BL_IN1 28
#define BL_IN2 29
#define PWMC 4 
#define BR_IN1 26
#define BR_IN2 27
#define PWMD 5
// --- Encoder Pins (Mega) ---
#define ENC_FL 20 
#define ENC_FR 21 
#define ENC_BL 19 
#define ENC_BR 18 

// --- Configuration Constants ---
const int MAX_SPEED = 150; 
const int STOP_DISTANCE_CM = 25; 
const int STUCK_THRESHOLD_COUNTS = 5; 
const int MAX_STUCK_ATTEMPTS = 3; 
const int MAX_OSCILLATION_ATTEMPTS = 4; // Limit for continuous left/right rotation
const int ROBOT_WIDTH_CM = 28;  
const int ROTATION_CLEARANCE_CM = ROBOT_WIDTH_CM / 2 + 5; 
const int BACKUP_DELAY_MS = 3000; // 3 seconds backward/forward recovery move

// ====================================================================
// 2. GLOBAL VARIABLES & ENUMS
// ====================================================================

Servo myservo;
volatile long encoderCounts[4] = {0, 0, 0, 0}; 
long prevEncoderCounts[4] = {0, 0, 0, 0}; 

int forwardStuckCount = 0;
int backwardStuckCount = 0;
int rotationOscillationCount = 0;
bool currentlyMovingForward = true; 
bool isRecovering = false; // NEW: Flag to ignore stall checks during recovery delay

enum Direction { FORWARD, BACKWARD, LEFT, RIGHT, STOP };

// ====================================================================
// 3. FUNCTION PROTOTYPES 
// ====================================================================

// Motor Control
void setMotorSpeed(int in1, int in2, int pwm, int speed, Direction dir);
void moveRobot(Direction dir, int speed);
void stopRobot();
void rotateRobot(Direction turnDir, int degrees);

// Sensor/Navigation
float getDistance();
float getDistanceAtAngle(int angle);
int scanEnvironment();

// Evasion Logic
bool checkStall();
void handleStuck();
void performAdvancedEvasion();
void performPostRecoveryScan(); 
void performEmergencyStuckTrappedEvasion(); 
void performRotationBreakEvasion();

// Encoder ISRs
void isr_fl() { encoderCounts[0]++; }
void isr_fr() { encoderCounts[1]++; }
void isr_bl() { encoderCounts[2]++; }
void isr_br() { encoderCounts[3]++; }

// ====================================================================
// 4. SETUP & MAIN LOOP
// ====================================================================
void setup() {
  Serial.begin(9600);
  int outputPins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, BL_IN1, BL_IN2, BR_IN1, BR_IN2, PWMA, PWMB, PWMC, PWMD, TRIG_PIN};
  for (int pin : outputPins) pinMode(pin, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENC_FL, INPUT_PULLUP); 
  pinMode(ENC_FR, INPUT_PULLUP);
  pinMode(ENC_BL, INPUT_PULLUP);
  pinMode(ENC_BR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_FL), isr_fl, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENC_FR), isr_fr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BL), isr_bl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR), isr_br, CHANGE);
  myservo.attach(SERVO_PIN);
  myservo.write(90); 
  moveRobot(FORWARD, MAX_SPEED);
  for(int i = 0; i < 4; i++) prevEncoderCounts[i] = encoderCounts[i];
  Serial.println("System Initialized.");
}


void loop() {
  static unsigned long lastStallCheck = 0;
  
  // NEW LOGIC: Only check for stall if NOT currently in a recovery movement
  if (!isRecovering && (millis() - lastStallCheck >= 100)) { 
    if (checkStall()) {
      handleStuck();
    }
    lastStallCheck = millis();
  }
  
  // Check for rotation oscillation
  if (rotationOscillationCount > MAX_OSCILLATION_ATTEMPTS) {
      Serial.println("!!! Rotation Oscillation Detected !!!");
      rotationOscillationCount = 0;
      performRotationBreakEvasion();
  }

  float distance = getDistance();

  // Case 1: Obstacle Avoidance (Stop at 25cm)
  if (distance < STOP_DISTANCE_CM) {
    stopRobot();
    backwardStuckCount = 0; 
    int clearAngle = scanEnvironment(); 
    
    if (clearAngle != -1) {
      // FIX FOR WIDE OBSTACLE/SOFA 
      if (clearAngle == 0) { 
        rotateRobot(RIGHT, 45); 
        Serial.println("Avoidance: Rotated 90 RIGHT.");
      } 
      else if (clearAngle == 180) { 
        rotateRobot(LEFT, 45); 
        Serial.println("Avoidance: Rotated 90 LEFT.");
      }
      else {
         rotateRobot(RIGHT, 45);
         Serial.println("Avoidance: Rotated 90 RIGHT (Default).");
      }
      
      rotationOscillationCount++;
      currentlyMovingForward = true;
      moveRobot(FORWARD, MAX_SPEED);
      
    } else {
      Serial.println("!!! TRAPPED !!! All 3 sides blocked. Waiting for STUCK check...");
    }
  } else {
    // Path clear, move forward, reset oscillation counter
    rotationOscillationCount = 0; 
    currentlyMovingForward = true;
    moveRobot(FORWARD, MAX_SPEED);
  }
}

// ====================================================================
// 5. STUCK HANDLER & EVASION LOGIC
// ====================================================================

// Rotation Oscillation Break Evasion
void performRotationBreakEvasion() {
    Serial.println("Rotation Break Evasion: Backing up and rotating 45 deg.");
    stopRobot();
    
    // Commit to the backward move without stall check interruption
    moveRobot(BACKWARD, MAX_SPEED / 3); 
    isRecovering = true;
    delay(1000); 
    isRecovering = false;
    stopRobot();
    
    rotateRobot(RIGHT, 45); 
    
    performPostRecoveryScan();
    
    forwardStuckCount = 0;
    backwardStuckCount = 0;
}

// Emergency Stuck-Trapped Evasion
void performEmergencyStuckTrappedEvasion() {
  Serial.println("!!! EMERGENCY: STUCK AND TRAPPED !!!");
  stopRobot();

  // Commit to the backward move without stall check interruption
  Serial.println("Performing extended backup (3 seconds)...");
  moveRobot(BACKWARD, MAX_SPEED / 2);
  isRecovering = true;
  delay(BACKUP_DELAY_MS); 
  isRecovering = false;
  stopRobot();

  performPostRecoveryScan();
}


void performPostRecoveryScan() {
  Serial.println("Performing Post-Recovery Scan...");
  stopRobot();
  
  int bestAngle = scanEnvironment(); 

  if (bestAngle != -1) {
    Direction turnDir = RIGHT;
    int turnAngle = 90;

    bool rightSafe = (getDistanceAtAngle(0) > ROTATION_CLEARANCE_CM);
    bool leftSafe = (getDistanceAtAngle(180) > ROTATION_CLEARANCE_CM);

    if (bestAngle == 0 && rightSafe) {
        turnDir = RIGHT;
    } else if (bestAngle == 180 && leftSafe) {
        turnDir = LEFT;
    } else if (rightSafe) {
        turnDir = RIGHT;
    } else if (leftSafe) {
        turnDir = LEFT;
    } else {
        Serial.println("Post-Recovery: Side paths blocked/unsafe. Forcing 180 turn.");
        rotateRobot(RIGHT, 180);
        currentlyMovingForward = true;
        moveRobot(FORWARD, MAX_SPEED);
        return;
    }
    
    rotateRobot(turnDir, turnAngle);
    
    currentlyMovingForward = true;
    moveRobot(FORWARD, MAX_SPEED);

  } else {
    Serial.println("Post-Recovery: All scanned paths blocked (< 25cm). Performing 180 turn.");
    rotateRobot(RIGHT, 180);
    currentlyMovingForward = true;
    moveRobot(FORWARD, MAX_SPEED);
  }
}

void performAdvancedEvasion() {
  Serial.println("!!! ADVANCED EVASION TRIGGERED (Repeated Stuck > 3) !!!");
  stopRobot();
  
  performPostRecoveryScan();
  forwardStuckCount = 0; 
  backwardStuckCount = 0;
}

void handleStuck() {
  stopRobot();
  
  float dist_front = getDistanceAtAngle(90);
  int clearAngle = scanEnvironment(); 

  // Check for Emergency Stuck-Trapped Evasion
  if (checkStall() && dist_front < STOP_DISTANCE_CM && clearAngle == -1) {
      performEmergencyStuckTrappedEvasion();
      return;
  }
  
  if (currentlyMovingForward) { // Stuck Forward (Case 3)
    forwardStuckCount++;
    backwardStuckCount = 0; 
    
    if (forwardStuckCount > MAX_STUCK_ATTEMPTS) {
      performAdvancedEvasion();
      return;
    }
    
    // Standard recovery: move backward (3s delay)
    Serial.println("Standard recovery (Forward stuck): Move BACKWARD (3s), then SCAN.");
    currentlyMovingForward = false;
    moveRobot(BACKWARD, MAX_SPEED / 2);
    
    // START RECOVERY PHASE
    isRecovering = true;
    delay(BACKUP_DELAY_MS); 
    isRecovering = false;
    // END RECOVERY PHASE
    
    performPostRecoveryScan(); // Scan after 3s backup
    
  } else { // Stuck Backward (Case 4)
    backwardStuckCount++;
    forwardStuckCount = 0; 

    if (backwardStuckCount > MAX_STUCK_ATTEMPTS) {
      performAdvancedEvasion();
      return;
    }
    
    // Standard recovery: move forward nudge (3s delay)
    Serial.println("Standard recovery (Backward stuck): Move FORWARD nudge (3s), then SCAN.");
    currentlyMovingForward = true;
    moveRobot(FORWARD, MAX_SPEED / 2);
    
    // START RECOVERY PHASE
    isRecovering = true;
    delay(BACKUP_DELAY_MS); 
    isRecovering = false;
    // END RECOVERY PHASE
    
    performPostRecoveryScan(); // Scan after forward nudge
  }
}

// ====================================================================
// 6. UTILITY FUNCTIONS (Unchanged)
// ====================================================================

float getDistanceAtAngle(int angle) {
  myservo.write(angle);
  delay(300); 
  float dist = getDistance();
  myservo.write(90); 
  delay(100);
  return dist;
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (duration == 0) return 400.0; 
  return duration * 0.034 / 2;
}

int scanEnvironment() {
  int bestDistance = 0;
  int bestAngle = -1; 
  int angles[] = {180, 90, 0}; 
  
  for (int angle : angles) {
    myservo.write(angle);
    delay(350); 
    float dist = getDistance();
    if (dist > bestDistance) {
      bestDistance = (int)dist;
      bestAngle = angle;
    }
  }
  
  myservo.write(90);
  delay(300); 
  if (bestDistance <= STOP_DISTANCE_CM) return -1; 
  return bestAngle;
}

bool checkStall() {
  bool isStuck = true;
  for (int i = 0; i < 4; i++) {
    if (abs(encoderCounts[i] - prevEncoderCounts[i]) > STUCK_THRESHOLD_COUNTS) {
      isStuck = false; 
    }
    prevEncoderCounts[i] = encoderCounts[i]; 
  }
  return isStuck;
}

// --- Motor Control Functions ---

void setMotorSpeed(int in1, int in2, int pwm, int speed, Direction dir) {
  if (dir == FORWARD) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (dir == BACKWARD) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else { 
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  analogWrite(pwm, speed);
}

void moveRobot(Direction dir, int speed) {
  setMotorSpeed(FL_IN1, FL_IN2, PWMA, speed, dir);
  setMotorSpeed(FR_IN1, FR_IN2, PWMB, speed, dir);
  setMotorSpeed(BL_IN1, BL_IN2, PWMC, speed, dir);
  setMotorSpeed(BR_IN1, BR_IN2, PWMD, speed, dir);
}

void stopRobot() { moveRobot(STOP, 0); }

void rotateRobot(Direction turnDir, int degrees) {
  long requiredTime = (long)degrees * 8; 

  if (turnDir == LEFT) {
    setMotorSpeed(FL_IN1, FL_IN2, PWMA, MAX_SPEED, BACKWARD);
    setMotorSpeed(FR_IN1, FR_IN2, PWMB, MAX_SPEED, FORWARD);
    setMotorSpeed(BL_IN1, BL_IN2, PWMC, MAX_SPEED, BACKWARD);
    setMotorSpeed(BR_IN1, BR_IN2, PWMD, MAX_SPEED, FORWARD);
  } else if (turnDir == RIGHT) {
    setMotorSpeed(FL_IN1, FL_IN2, PWMA, MAX_SPEED, FORWARD);
    setMotorSpeed(FR_IN1, FR_IN2, PWMB, MAX_SPEED, BACKWARD);
    setMotorSpeed(BL_IN1, BL_IN2, PWMC, MAX_SPEED, FORWARD);
    setMotorSpeed(BR_IN1, BR_IN2, PWMD, MAX_SPEED, BACKWARD);
  }
  
  long startTime = millis();
  while(millis() - startTime < requiredTime);
  stopRobot();
}