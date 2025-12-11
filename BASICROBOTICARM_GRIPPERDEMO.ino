// Base servo  = 3
// Shoulder    = 5
// Elbow       = 6
// Gripper     = 9
// ======================================================
//  ROBOT 3: ROBOTIC ARM DEMO (3 joints + gripper)
//  Autonomous demo + optional keyboard control
// ======================================================

#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

int basePin = 3;
int shoulderPin = 5;
int elbowPin = 6;
int gripPin = 9;

void setup() {
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  gripperServo.attach(gripPin);

  Serial.begin(9600);
  Serial.println("ROBOT 3: ROBOTIC ARM STARTED...");

  // Initial pose
  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  gripperServo.write(30);  // open
}

// AUTONOMOUS small demo sequence
void demoSequence() {
  // Close gripper
  gripperServo.write(90);
  delay(500);

  // Move somewhere
  baseServo.write(60);
  shoulderServo.write(120);
  elbowServo.write(70);
  delay(700);

  // Open gripper
  gripperServo.write(30);
  delay(500);

  // Back to home
  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  delay(1000);
}

// OPTIONAL keyboard control
// w/s -> shoulder up/down
// e/d -> elbow up/down
// a/d -> base left/right
// o/c -> open/close gripper
int basePos = 90;
int shoulderPos = 90;
int elbowPos = 90;
int gripPos = 30;

void handleKeyArm(char c) {
  if (c == 'a') basePos -= 5;
  if (c == 'd') basePos += 5;

  if (c == 'w') shoulderPos -= 5;
  if (c == 's') shoulderPos += 5;

  if (c == 'e') elbowPos -= 5;
  if (c == 'x') elbowPos += 5;

  if (c == 'o') gripPos = 30;
  if (c == 'c') gripPos = 90;

  basePos = constrain(basePos, 0, 180);
  shoulderPos = constrain(shoulderPos, 0, 180);
  elbowPos = constrain(elbowPos, 0, 180);

  baseServo.write(basePos);
  shoulderServo.write(shoulderPos);
  elbowServo.write(elbowPos);
  gripperServo.write(gripPos);

  Serial.print("Base:"); Serial.print(basePos);
  Serial.print(" Sh:"); Serial.print(shoulderPos);
  Serial.print(" El:"); Serial.print(elbowPos);
  Serial.print(" Gr:"); Serial.println(gripPos);
}

void loop() {
  // AUTONOMOUS:
  demoSequence();

  // OPTIONAL KEYBOARD:
  /*
  if (Serial.available()) {
    char c = Serial.read();
    handleKeyArm(c);
  }
  */
}
