// ======================================================
//  ROBOT 2: LINE FOLLOWER + ULTRASONIC OBSTACLE AVOID
// ======================================================

const int IN1_2 = 2;
const int IN2_2 = 3;
const int IN3_2 = 4;
const int IN4_2 = 5;
const int ENA_2 = 6;
const int ENB_2 = 7;

int IR1_2 = 8;
int IR2_2 = 9;
int IR3_2 = 10;
int IR4_2 = 11;
int IR5_2 = 12;

const int TRIG = A0;
const int ECHO = A1;

int baseSpeed2 = 90;
long OBST_DIST = 15; // cm

void setup() {
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_2, OUTPUT);
  pinMode(ENA_2, OUTPUT);
  pinMode(ENB_2, OUTPUT);

  pinMode(IR1_2, INPUT);
  pinMode(IR2_2, INPUT);
  pinMode(IR3_2, INPUT);
  pinMode(IR4_2, INPUT);
  pinMode(IR5_2, INPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  Serial.begin(9600);
  Serial.println("ROBOT 2: LINE + OBSTACLE STARTED...");
}

void setSpeed2(int left, int right) {
  left  = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  analogWrite(ENA_2, left);
  analogWrite(ENB_2, right);
}

void forward2() {
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  setSpeed2(baseSpeed2, baseSpeed2);
}

void stopMotors2() {
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, LOW);
  setSpeed2(0, 0);
}

void slightLeft2() {
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  setSpeed2(70, 110);
}

void slightRight2() {
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  setSpeed2(110, 70);
}

long measureDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 999;
  return duration / 58;
}

// OPTIONAL keyboard
void handleKey2(char c) {
  if (c == 'w') forward2();
  else if (c == 's') stopMotors2();
}

void loop() {
  int s1 = digitalRead(IR1_2);
  int s2 = digitalRead(IR2_2);
  int s3 = digitalRead(IR3_2);
  int s4 = digitalRead(IR4_2);
  int s5 = digitalRead(IR5_2);

  long dist = measureDistance();

  Serial.print("IR: ");
  Serial.print(s1); Serial.print(" ");
  Serial.print(s2); Serial.print(" ");
  Serial.print(s3); Serial.print(" ");
  Serial.print(s4); Serial.print(" ");
  Serial.print(s5);
  Serial.print(" | D:");
  Serial.print(dist);
  Serial.print(" -> ");

  if (dist < OBST_DIST) {
    Serial.println("Obstacle -> Stop");
    stopMotors2();
  } else {
    // simple line logic (same as robot 1 short version)
    if (s2 == 1 && s3 == 1 && s4 == 1) {
      Serial.println("Straight");
      forward2();
    } else if (s1 == 1 && s2 == 1) {
      Serial.println("Left");
      slightLeft2();
    } else if (s4 == 1 && s5 == 1) {
      Serial.println("Right");
      slightRight2();
    } else {
      Serial.println("Default -> Forward");
      forward2();
    }
  }

  // OPTIONAL KEYBOARD
  /*
  if (Serial.available()) {
    char c = Serial.read();
    handleKey2(c);
  }
  */

  delay(5);
}
