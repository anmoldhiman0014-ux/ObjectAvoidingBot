/*
 * ============================================================
 *   OBSTACLE AVOIDANCE ROBOT — MAZE SOLVER EDITION
 * ============================================================
 * Components:
 *   - Arduino Uno (Microcontroller)
 *   - HC-SR04 Ultrasonic Sensor (mounted on SG-90 Servo)
 *   - L298N Motor Driver
 *   - 2x DC Motors (on Robotics Chassis)
 *   - SG-90 Servo Motor  [constrained to 30°–150° = 120° arc]
 *   - 9V LiPo Battery
 * ============================================================
 * PIN CONFIGURATION:
 *
 *  HC-SR04 Ultrasonic Sensor:
 *    TRIG  --> Pin 9
 *    ECHO  --> Pin 10
 *    VCC   --> 5V
 *    GND   --> GND
 *
 *  SG-90 Servo Motor:
 *    Signal --> Pin 6
 *    VCC    --> 5V
 *    GND    --> GND
 *
 *  L298N Motor Driver:
 *    IN1   --> Pin 2   (Left Motor Direction)
 *    IN2   --> Pin 3   (Left Motor Direction)
 *    IN3   --> Pin 4   (Right Motor Direction)
 *    IN4   --> Pin 5   (Right Motor Direction)
 *    ENA   --> Pin 11  (Left Motor Speed - PWM)
 *    ENB   --> Pin 12  (Right Motor Speed - PWM)
 *    VCC   --> 9V Battery (+)
 *    GND   --> 9V Battery (-) + Arduino GND (Common Ground)
 *    5V    --> Arduino 5V (optional, use L298N onboard reg)
 *
 * ============================================================
 * SERVO ARC  (120° total, 60° each side):
 *   Far-Left   = 150°  (center + 60°)
 *   Half-Left  = 120°  (center + 30°)
 *   Center     =  90°
 *   Half-Right =  60°  (center - 30°)
 *   Far-Right  =  30°  (center - 60°)
 *
 * DISTANCE THRESHOLDS:
 *   WARN_DISTANCE  25 cm — slow down, prepare to react
 *   STOP_DISTANCE  10 cm — EMERGENCY: stop → reverse → full scan
 *
 * MAZE-SOLVER STRATEGY:
 *   1. Drive forward while front > WARN_DISTANCE (full speed)
 *   2. 10–25 cm  → reduce speed, 3-point quick scan, turn if needed
 *   3. ≤ 10 cm   → emergency stop, reverse to create clearance,
 *                   5-point sweep, steer toward maximum open space
 *   4. All 5 directions blocked → long reverse + 180° U-turn
 *   5. lastTurnBias remembers last direction for corridor tiebreaks
 * ============================================================
 */

#include <Servo.h>

// ─── PIN DEFINITIONS ─────────────────────────────────────────
#define TRIG_PIN    9
#define ECHO_PIN    10
#define SERVO_PIN   6
#define IN1         2
#define IN2         3
#define IN3         4
#define IN4         5
#define ENA         11
#define ENB         12

// ─── DISTANCE THRESHOLDS (cm) ────────────────────────────────
#define WARN_DISTANCE     25    // Slow-down / caution zone
#define STOP_DISTANCE     10    // Emergency reverse + maze scan

// ─── MOTOR SPEEDS (PWM 0–255) ────────────────────────────────
#define SPEED_FULL        190   // Normal cruising speed
#define SPEED_SLOW        120   // Cautious approach speed
#define SPEED_TURN        160   // In-place turning speed

// ─── TIMING (ms) ─────────────────────────────────────────────
#define REVERSE_TIME_SHORT   400   // Quick reverse after soft stop
#define REVERSE_TIME_LONG    750   // Longer reverse when fully trapped
#define TURN_TIME_90         550   // ~90° turn  (tune for your chassis)
#define TURN_TIME_180       1100   // ~180° U-turn

// ─── SERVO POSITIONS ─────────────────────────────────────────
//   Hard-clamped to [30°, 150°] — never exceeds 120° total arc
#define SERVO_CENTER    90
#define SERVO_LEFT     150   // 90 + 60°
#define SERVO_HALF_L   120   // 90 + 30°
#define SERVO_HALF_R    60   // 90 - 30°
#define SERVO_RIGHT     30   // 90 - 60°

// ─── OBJECT ──────────────────────────────────────────────────
Servo scanServo;

// ─── STATE ───────────────────────────────────────────────────
// Maze tiebreak bias: +1 = last went right, -1 = left, 0 = none
int lastTurnBias = 0;

// ─────────────────────────────────────────────────────────────
//   SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("====================================");
  Serial.println("  Obstacle Avoidance — Maze Solver");
  Serial.println("  Servo arc: 30 deg to 150 deg (120 deg total)");
  Serial.println("  Emergency stop at: 10 cm");
  Serial.println("====================================");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  scanServo.attach(SERVO_PIN);
  centerServo();
  delay(600);

  stopMotors();
  delay(1200);
  Serial.println(">> Robot ready. Starting in 1s...");
  delay(1000);
}

// ─────────────────────────────────────────────────────────────
//   MAIN LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  int front = getDistance();
  printDist("FRONT", front);

  if (front > WARN_DISTANCE) {
    // ── CLEAR — full speed ahead ──────────────────────────────
    driveForward(SPEED_FULL);

  } else if (front > STOP_DISTANCE) {
    // ── CAUTION ZONE (10–25 cm) — slow down + pre-emptive scan
    driveForward(SPEED_SLOW);
    delay(80);
    front = getDistance();           // Recheck while crawling
    if (front <= WARN_DISTANCE) {
      stopMotors();
      delay(150);
      quickDecide();
    }

  } else {
    // ── EMERGENCY ZONE (≤ 10 cm) — maze solver ───────────────
    Serial.println("!! EMERGENCY  obstacle within 10 cm !!");
    stopMotors();
    delay(200);

    // Reverse to create sensor clearance
    driveBackward(SPEED_FULL);
    delay(REVERSE_TIME_SHORT);
    stopMotors();
    delay(250);

    // Full 5-point sweep and pick best route
    mazeSolve();
  }
}

// ─────────────────────────────────────────────────────────────
//   QUICK DECIDE  — 3-point scan used in the caution zone
// ─────────────────────────────────────────────────────────────
void quickDecide() {
  int dLeft  = scanDirection(SERVO_LEFT);
  int dFront = scanDirection(SERVO_CENTER);
  int dRight = scanDirection(SERVO_RIGHT);
  centerServo();

  Serial.print("QUICK-SCAN  L:"); Serial.print(dLeft);
  Serial.print("  F:");           Serial.print(dFront);
  Serial.print("  R:");           Serial.println(dRight);

  // If front cleared up just continue
  if (dFront > WARN_DISTANCE) return;

  if (dLeft >= dRight) {
    if (dLeft > STOP_DISTANCE) executeTurn(-1);   // left
    else                       mazeSolve();        // need full scan
  } else {
    if (dRight > STOP_DISTANCE) executeTurn(1);   // right
    else                        mazeSolve();
  }
}

// ─────────────────────────────────────────────────────────────
//   MAZE SOLVE  — 5-point sweep, steer to maximum clearance
// ─────────────────────────────────────────────────────────────
/*
 *  Scan map (left → right):
 *
 *   Index:     0         1         2         3         4
 *   Angle:   150°      120°       90°       60°       30°
 *   Label:  FAR-L    HALF-L    FRONT    HALF-R    FAR-R
 *   Dir:      -1        -1         0        +1       +1
 *
 *  Pick the index with the longest reading.
 *  Convert its angle offset to a robot turn and execute it.
 */
void mazeSolve() {
  Serial.println(">> MAZE SOLVE: 5-point sweep");

  const int   angles[5]   = { SERVO_LEFT, SERVO_HALF_L, SERVO_CENTER,
                               SERVO_HALF_R, SERVO_RIGHT };
  const int   turnDir[5]  = { -1, -1, 0, 1, 1 };
  const char* labels[5]   = { "FAR-L", "HALF-L", "FRONT", "HALF-R", "FAR-R" };

  int distances[5];

  for (int i = 0; i < 5; i++) {
    distances[i] = scanDirection(angles[i]);
    Serial.print("  "); Serial.print(labels[i]);
    Serial.print(": ");  Serial.print(distances[i]);
    Serial.println(" cm");
    delay(50);
  }

  centerServo();
  delay(300);

  // Find best (most open) direction
  int bestIdx  = 0;
  int bestDist = 0;
  for (int i = 0; i < 5; i++) {
    // Apply a small bias toward the previously successful direction
    // to help navigate corridors without oscillating
    int score = distances[i];
    if (turnDir[i] == lastTurnBias && lastTurnBias != 0) score += 5;
    if (score > bestDist) {
      bestDist = score;
      bestIdx  = i;
    }
  }

  Serial.print(">> Best: "); Serial.print(labels[bestIdx]);
  Serial.print("  dist=");   Serial.print(distances[bestIdx]);
  Serial.println(" cm");

  // ── Fully trapped ────────────────────────────────────────
  if (distances[bestIdx] <= STOP_DISTANCE) {
    Serial.println(">> ALL BLOCKED — long reverse + U-turn");
    driveBackward(SPEED_FULL);
    delay(REVERSE_TIME_LONG);
    stopMotors();
    delay(200);
    uTurn();
    return;
  }

  // ── Steer toward best direction ──────────────────────────
  int dir = turnDir[bestIdx];
  if (dir == 0) {
    Serial.println(">> Front clear — continuing forward");
    // No turn needed; loop will drive forward next cycle
  } else {
    executeTurn(dir);
  }
}

// ─────────────────────────────────────────────────────────────
//   EXECUTE TURN  (dir: -1 left, +1 right)
// ─────────────────────────────────────────────────────────────
void executeTurn(int dir) {
  if (dir < 0) {
    Serial.println(">> Turning LEFT 90 deg");
    turnLeft(SPEED_TURN);
  } else {
    Serial.println(">> Turning RIGHT 90 deg");
    turnRight(SPEED_TURN);
  }
  delay(TURN_TIME_90);
  stopMotors();
  delay(200);
  lastTurnBias = dir;
}

// ─────────────────────────────────────────────────────────────
//   U-TURN  (~180°) — opposite to last bias
// ─────────────────────────────────────────────────────────────
void uTurn() {
  Serial.println(">> U-TURN 180 deg");
  if (lastTurnBias >= 0) turnLeft(SPEED_TURN);
  else                   turnRight(SPEED_TURN);
  delay(TURN_TIME_180);
  stopMotors();
  delay(300);
  lastTurnBias = -lastTurnBias;   // Flip bias after 180
}

// ─────────────────────────────────────────────────────────────
//   ULTRASONIC  HC-SR04
// ─────────────────────────────────────────────────────────────
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30 ms timeout
  int dist = (int)(duration * 0.034 / 2);

  if (dist <= 0 || dist > 400) dist = 400;  // clamp invalid readings
  return dist;
}

// Clamp angle to servo arc [30–150] before writing
int scanDirection(int angle) {
  angle = constrain(angle, SERVO_RIGHT, SERVO_LEFT);  // enforce 120° arc
  scanServo.write(angle);
  delay(350);               // let servo physically reach position
  return getDistance();
}

void centerServo() {
  scanServo.write(SERVO_CENTER);
  delay(300);
}

// ─────────────────────────────────────────────────────────────
//   MOTOR CONTROL  (all functions accept explicit speed param)
// ─────────────────────────────────────────────────────────────
void driveForward(int spd) {
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void driveBackward(int spd) {
  analogWrite(ENA, spd);  analogWrite(ENB, spd);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void turnLeft(int spd) {
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // Left  motor: backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right motor: forward
}

void turnRight(int spd) {
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left  motor: forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Right motor: backward
}

void stopMotors() {
  analogWrite(ENA, 0);    analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ─────────────────────────────────────────────────────────────
//   DEBUG
// ─────────────────────────────────────────────────────────────
void printDist(const char* label, int dist) {
  Serial.print("["); Serial.print(label); Serial.print("] ");
  Serial.print(dist); Serial.println(" cm");
}
