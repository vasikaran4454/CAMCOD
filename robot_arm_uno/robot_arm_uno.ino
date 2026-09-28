/*
  ChromaPick - Arduino (ATmega328P / Uno) robot arm controller
  ------------------------------------------------------------
  Buttons (to GND, INPUT_PULLUP): D2 = BTN1 RED, D3 = BTN2 BLUE, D4 = BTN3 GREEN
  Servos (4):  D9 = base, D10 = shoulder, D11 = elbow, D6 = gripper
  LED: D13 (built-in)

  IMPORTANT
   - Power the servos from an EXTERNAL 5-6V supply (2A+), NOT the Arduino 5V pin.
   - Join the external supply GND with Arduino GND.
   - D0 / D1 are used by USB serial, do not connect anything there.
   - Close Arduino Serial Monitor before using the web page (only one app can use the COM port).

  Protocol (115200 baud)
   Arm -> PC :  BTN1 / BTN2 / BTN3   (button pressed)
                DONE                 (pick finished)
   PC -> Arm :  PICK,RED,320,240     (color, camera pixel x, y in a 640x480 frame)
                NOTFOUND,RED
*/

#include <Servo.h>

// ------------------------- PINS -------------------------
const int BTN_PINS[3] = {2, 3, 4};
const char* BTN_NAMES[3] = {"BTN1", "BTN2", "BTN3"};
const int SERVO_PINS[4] = {9, 10, 11, 6};     // base, shoulder, elbow, gripper
const int LED_PIN = 13;

// ------------------------- CALIBRATION (tune these!) -------------------------
// Camera frame is 640 x 480. Camera must stay fixed.
const int BASE_AT_X0   = 150;   // base angle when ball is at x = 0   (left of image)
const int BASE_AT_X640 = 30;    // base angle when ball is at x = 640 (right of image)
                                // if arm turns the wrong way, swap these two numbers

// Reach: y = 0 is far from camera bottom edge... set by trial. Two reference poses:
const int SH_AT_Y0   = 60,  EL_AT_Y0   = 50;    // pose for ball at y = 0
const int SH_AT_Y480 = 110, EL_AT_Y480 = 100;   // pose for ball at y = 480

const int GRIP_OPEN = 40, GRIP_CLOSED = 110;    // gripper angles
const int LIFT_SH = 80,  LIFT_EL = 70;          // "arm raised" pose while carrying

// Drop position per color: base, shoulder, elbow
struct Pose { int b, s, e; };
const Pose DROP_RED   = {20,  90, 90};
const Pose DROP_BLUE  = {90,  90, 90};
const Pose DROP_GREEN = {160, 90, 90};

const Pose HOME = {90, 80, 70};
const int STEP_DELAY = 15;                      // ms per degree (bigger = slower, smoother)
// -----------------------------------------------------------------------------

Servo sv[4];
int cur[4] = {90, 80, 70, GRIP_OPEN};           // current angles
bool lastState[3] = {HIGH, HIGH, HIGH};
unsigned long lastChange[3] = {0, 0, 0};
const unsigned long DEBOUNCE_MS = 40;
String rx = "";

// Smoothly move all 4 servos to target angles
void moveTo(int b, int s, int e, int g) {
  int tgt[4] = {constrain(b, 0, 180), constrain(s, 0, 180), constrain(e, 0, 180), constrain(g, 0, 180)};
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < 4; i++) {
      if (cur[i] < tgt[i]) { cur[i]++; moving = true; }
      else if (cur[i] > tgt[i]) { cur[i]--; moving = true; }
      sv[i].write(cur[i]);
    }
    delay(STEP_DELAY);
  }
}

void goHome() { moveTo(HOME.b, HOME.s, HOME.e, cur[3]); }

// ------------------------- PICK & PLACE -------------------------
void pickBall(String color, int x, int y) {
  x = constrain(x, 0, 640);
  y = constrain(y, 0, 480);

  int baseAngle = map(x, 0, 640, BASE_AT_X0, BASE_AT_X640);
  int shAngle   = map(y, 0, 480, SH_AT_Y0, SH_AT_Y480);
  int elAngle   = map(y, 0, 480, EL_AT_Y0, EL_AT_Y480);

  Pose drop = DROP_RED;
  if (color == "BLUE") drop = DROP_BLUE;
  else if (color == "GREEN") drop = DROP_GREEN;

  moveTo(cur[0], LIFT_SH, LIFT_EL, GRIP_OPEN);           // raise + open gripper
  moveTo(baseAngle, LIFT_SH, LIFT_EL, GRIP_OPEN);        // turn above the ball
  moveTo(baseAngle, shAngle, elAngle, GRIP_OPEN);        // lower to the ball
  delay(300);
  moveTo(baseAngle, shAngle, elAngle, GRIP_CLOSED);      // grab
  delay(400);
  moveTo(baseAngle, LIFT_SH, LIFT_EL, GRIP_CLOSED);      // lift
  moveTo(drop.b, LIFT_SH, LIFT_EL, GRIP_CLOSED);         // go above drop zone
  moveTo(drop.b, drop.s, drop.e, GRIP_CLOSED);           // lower
  delay(300);
  moveTo(drop.b, drop.s, drop.e, GRIP_OPEN);             // release
  delay(300);
  moveTo(drop.b, LIFT_SH, LIFT_EL, GRIP_OPEN);           // lift
  goHome();
}

// ------------------------- SETUP / LOOP -------------------------
void setup() {
  Serial.begin(115200);                                  // must match the web page (115200)
  for (int i = 0; i < 3; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    sv[i].attach(SERVO_PINS[i]);
    sv[i].write(cur[i]);
  }
  delay(500);
  goHome();
}

void blinkError() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PIN, HIGH); delay(120);
    digitalWrite(LED_PIN, LOW);  delay(120);
  }
}

void handleLine(String line) {
  line.trim();
  if (line.startsWith("PICK,")) {
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) return;
    String color = line.substring(c1 + 1, c2);
    int x = line.substring(c2 + 1, c3).toInt();
    int y = line.substring(c3 + 1).toInt();

    digitalWrite(LED_PIN, HIGH);
    pickBall(color, x, y);
    digitalWrite(LED_PIN, LOW);
    Serial.println("DONE");
  } else if (line.startsWith("NOTFOUND")) {
    blinkError();
    Serial.println("DONE");
  }
}

void loop() {
  for (int i = 0; i < 3; i++) {                          // buttons
    bool s = digitalRead(BTN_PINS[i]);
    if (s != lastState[i] && millis() - lastChange[i] > DEBOUNCE_MS) {
      lastChange[i] = millis();
      lastState[i] = s;
      if (s == LOW) Serial.println(BTN_NAMES[i]);
    }
  }
  while (Serial.available()) {                           // commands from PC
    char ch = Serial.read();
    if (ch == '\n') { handleLine(rx); rx = ""; }
    else if (ch != '\r') rx += ch;
  }
}
