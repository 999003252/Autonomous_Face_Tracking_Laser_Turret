#include <Servo.h>

Servo servoPan;  // X1 (Left/Right)
Servo servoY1;   // Y1 (Base Tilt - Locked)
Servo servoY2;   // Y2 (Top Tilt - Active aiming)

const int LASER_PIN = 12;

const bool INVERT_X = false; 
const bool INVERT_Y = false;

// --- PHYSICAL CALIBRATION ---
const float PAN_HOME  = 30.0;  // Center on your face
const int   Y1_LOCKED = 15;    // Static riser
const float Y2_HOME   = 125.0; // Starting point

// --- WORKSPACE BOUNDARIES ---
const int PAN_MIN = 5;         // West limit (protects gears)
const int PAN_MAX = 130;       // East limit (as tested)
const int Y2_MIN  = 80;
const int Y2_MAX  = 170;

// --- TUNING GAINS & PATROL SWEEP ---
const float KP_PAN  = 0.025;
const float KP_TILT = 0.025;
const int   DEAD_X  = 8;
const int   DEAD_Y  = 8;

const float SWEEP_SPEED = 0.6; // Edge-to-edge patrol pace
int sweepDirection = 1;        

float smoothPan = PAN_HOME;
float smoothY2  = Y2_HOME;
const float ALPHA = 0.25;    

void setup() {
  Serial.begin(115200);

  servoPan.attach(9);
  servoY1.attach(10);
  servoY2.attach(11);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH); // Laser stays active to hunt for targets

  servoPan.write((int)PAN_HOME);
  servoY1.write(Y1_LOCKED);
  servoY2.write((int)Y2_HOME);
  delay(500);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // strip any trailing \r or whitespace before comparing

    // --- DEDICATED LASER TOGGLE COMMAND (for the standalone blink_diff_test.py
    // calibration tool only) --- Bypasses servo control entirely. The live
    // tracking scripts no longer need this: they control the laser via the
    // 4th field in the ex,ey,mode,laser protocol below instead.
    if (data == "B1") {
      digitalWrite(LASER_PIN, HIGH);
      return;
    }
    if (data == "B0") {
      digitalWrite(LASER_PIN, LOW);
      return;
    }

    int comma1 = data.indexOf(',');
    int comma2 = data.indexOf(',', comma1 + 1);
    int comma3 = data.indexOf(',', comma2 + 1);
    if (comma1 == -1 || comma2 == -1 || comma3 == -1) return;

    int errorX     = data.substring(0, comma1).toInt();
    int errorY     = data.substring(comma1 + 1, comma2).toInt();
    int mode       = data.substring(comma2 + 1, comma3).toInt();
    int laserState = data.substring(comma3 + 1).toInt();

    // Laser state now comes from Python on every command instead of being
    // hardcoded HIGH inside each mode branch below. This is what makes
    // continuous blink-and-difference detection possible during live
    // tracking -- Python can request the laser OFF for one cycle without
    // the mode handlers immediately overriding it back to HIGH.
    digitalWrite(LASER_PIN, laserState ? HIGH : LOW);

    // --- MODE 0: SWEEP/PATROL HUNTING MODE ---
    if (errorX == 9999 && errorY == 9999) {
      smoothPan += (SWEEP_SPEED * sweepDirection);
      
      if (smoothPan >= PAN_MAX) {
        smoothPan = PAN_MAX;
        sweepDirection = -1; 
      } 
      else if (smoothPan <= PAN_MIN) {
        smoothPan = PAN_MIN;
        sweepDirection = 1;  
      }

      // Smoothly level out Y-axis during the horizontal sweep
      if (smoothY2 < Y2_HOME) smoothY2 += SWEEP_SPEED;
      if (smoothY2 > Y2_HOME) smoothY2 -= SWEEP_SPEED;
      if (abs(smoothY2 - Y2_HOME) < SWEEP_SPEED) smoothY2 = Y2_HOME;
      
      servoPan.write((int)smoothPan);
      servoY1.write(Y1_LOCKED);
      servoY2.write((int)smoothY2);
      return; 
    }

    // --- MODE 2: ACTIVE FINE TARGET TRACKING ---
    if (INVERT_X) errorX = -errorX;
    if (INVERT_Y) errorY = -errorY;

    float targetPan = smoothPan + (KP_PAN * errorX);
    float targetY2  = smoothY2 - (KP_TILT * errorY); 

    if (abs(errorX) < DEAD_X) targetPan = smoothPan;
    if (abs(errorY) < DEAD_Y) targetY2  = smoothY2;

    smoothPan += ALPHA * (targetPan - smoothPan);
    smoothY2  += ALPHA * (targetY2 - smoothY2);

    smoothPan = constrain(smoothPan, PAN_MIN, PAN_MAX);
    smoothY2  = constrain(smoothY2,  Y2_MIN,  Y2_MAX);

    servoPan.write((int)smoothPan);
    servoY1.write(Y1_LOCKED);
    servoY2.write((int)smoothY2);
  }
}
