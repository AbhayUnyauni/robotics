// ======================================================
//  ROBOT 5: MASTER BOT (LINE + OBSTACLE + COLOR + GRIPPER)
// ======================================================

#include <Servo.h>

// Motors
const int IN1m = 2;
const int IN2m = 3;
const int IN3m = 4;
const int IN4m = 5;
const int ENAm = 6;
const int ENBm = 7;

// IR sensors
int IRM[5] = {8, 9, 10, 11, 12};

// Ultrasonic
const int TRIGm = A0;
const int ECHOm = A1;

// Color sensor (TCS3200)
const int S0m = A2;
const int S1m = A3;
const int S2m = A4;
const int S3m = A5;
const int OUTm = 13;

// Gripper
Servo gripM;
int GRIP_M_PIN = 9;

int baseSpd = 90;
long OBSTm = 15;
enum MColor { MC_UNKNOWN, MC_RED, MC_GREEN, MC_BLUE };

void setSpeedM(int L, int R) {
  L = constrain(L, 0, 255);
  R = constrain(R, 0, 255);
  analogWrite(ENAm, L);
  analogWrite(ENBm, R);
}

void forwardM() {
  digitalWrite(IN1m, HIGH); digitalWrite(IN2m, LOW);
  digitalWrite(IN3m, HIGH); digitalWrite(IN4m, LOW);
  setSpeedM(baseSpd, baseSpd);
}

void stopM() {
  digitalWrite(IN1m, LOW); digitalWrite(IN2m, LOW);
  digitalWrite(IN3m, LOW); digitalWrite(IN4m, LOW);
  setSpeedM(0, 0);
}

void slightLeftM() {
  digitalWrite(IN1m, HIGH); digitalWrite(IN2m, LOW);
  digitalWrite(IN3m, HIGH); digitalWrite(IN4m, LOW);
  setSpeedM(70, 110);
}

void slightRightM() {
  digitalWrite(IN1m, HIGH); digitalWrite(IN2m, LOW);
  digitalWrite(IN3m, HIGH); digitalWrite(IN4m, LOW);
  setSpeedM(110, 70);
}

long distM() {
  digitalWrite(TRIGm, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGm, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGm, LOW);
  long d = pulseIn(ECHOm, HIGH, 30000);
  if (d == 0) return 999;
  return d / 58;
}

unsigned long readFreqM(bool s2, bool s3) {
  digitalWrite(S2m, s2 ? HIGH : LOW);
  digitalWrite(S3m, s3 ? HIGH : LOW);
  delay(20);
  unsigned long dur = pulseIn(OUTm, LOW, 25000);
  if (dur == 0) dur = 25000;
  return dur;
}

MColor detectColorM() {
  unsigned long r = readFreqM(LOW, LOW);
  unsigned long b = readFreqM(LOW, HIGH);
  unsigned long g = readFreqM(HIGH, HIGH);

  Serial.print("R:"); Serial.print(r);
  Serial.print(" G:"); Serial.print(g);
  Serial.print(" B:"); Serial.println(b);

  if (r < g && r < b) return MC_RED;
  if (g < r && g < b) return MC_GREEN;
  if (b < r && b < g) return MC_BLUE;
  return MC_UNKNOWN;
}

void setup() {
  pinMode(IN1m, OUTPUT); pinMode(IN2m, OUTPUT);
  pinMode(IN3m, OUTPUT); pinMode(IN4m, OUTPUT);
  pinMode(ENAm, OUTPUT); pinMode(ENBm, OUTPUT);

  for (int i = 0; i < 5; i++) pinMode(IRM[i], INPUT);

  pinMode(TRIGm, OUTPUT);
  pinMode(ECHOm, INPUT);

  pinMode(S0m, OUTPUT);
  pinMode(S1m, OUTPUT);
  pinMode(S2m, OUTPUT);
  pinMode(S3m, OUTPUT);
  pinMode(OUTm, INPUT);

  digitalWrite(S0m, HIGH);
  digitalWrite(S1m, LOW);

  gripM.attach(GRIP_M_PIN);
  gripM.write(30);  // open

  Serial.begin(9600);
  Serial.println("ROBOT 5: MASTER BOT STARTED...");
}

// Simple line follower step
void lineStepM() {
  int s[5];
  for (int i = 0; i < 5; i++) s[i] = digitalRead(IRM[i]);

  long d = distM();

  Serial.print("IR: ");
  for (int i = 0; i < 5; i++) { Serial.print(s[i]); Serial.print(" "); }
  Serial.print("| D:"); Serial.print(d); Serial.print(" -> ");

  if (d < OBSTm) {
    Serial.println("Obstacle stop");
    stopM();
    return;
  }

  if (s[1] == 1 && s[2] == 1 && s[3] == 1) {
    Serial.println("Straight");
    forwardM();
  } else if (s[0] == 1 && s[1] == 1) {
    Serial.println("Left");
    slightLeftM();
  } else if (s[3] == 1 && s[4] == 1) {
    Serial.println("Right");
    slightRightM();
  } else {
    Serial.println("Default Fwd");
    forwardM();
  }
}

// Simple workflow demo:
// 1. follow line for some time
// 2. stop, close gripper, detect color
void runWorkflow() {
  unsigned long start = millis();
  while (millis() - start < 5000) {
    lineStepM();
    delay(5);
  }

  stopM();
  delay(500);

  // pick
  gripM.write(90); delay(500);

  // color
  MColor c = detectColorM();
  if (c == MC_RED)   Serial.println("Workflow: RED");
  if (c == MC_GREEN) Serial.println("Workflow: GREEN");
  if (c == MC_BLUE)  Serial.println("Workflow: BLUE");
  if (c == MC_UNKNOWN) Serial.println("Workflow: UNKNOWN");

  gripM.write(30); delay(500);
}

void loop() {
  runWorkflow();

  // OPTIONAL keyboard control stub:
  /*
  if (Serial.available()) {
    char c = Serial.read();
    // e.g., 'w' -> forwardM(), 's' -> stopM(), 'o'->open grip, 'c'->close grip
  }
  */

  while (1) {
    stopM();
  }
}
