// -------------------- PIN DEFINITIONS --------------------
int IN1 = 2;
int IN2 = 3;
int IN3 = 4;
int IN4 = 5;

int ENA = 6;
int ENB = 7;

int IR1 = 8;   // Right-most
int IR2 = 9;
int IR3 = 10;  // Center
int IR4 = 11;
int IR5 = 12;  // Left-most

// -------------------- SPEEDS --------------------
int baseSpeed = 120;
int maxSpeed  = 180;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  Serial.begin(9600);
}

// -------------------- MOTOR FUNCTIONS --------------------
void setMotor(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    leftSpeed = -leftSpeed;
  }

  // Right Motor
  if (rightSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    rightSpeed = -rightSpeed;
  }

  analogWrite(ENA, constrain(rightSpeed, 0, 255));
  analogWrite(ENB, constrain(leftSpeed, 0, 255));
}

void loop() {

  int s1 = !digitalRead(IR1); // Convert WHITE=1 → BLAlfCK=1
  int s2 = !digitalRead(IR2);
  int s3 = !digitalRead(IR3);
  int s4 = !digitalRead(IR4);
  int s5 = !digitalRead(IR5);

  // Weighted error
  int error =
      s1 * -4 +
      s2 * -2 +
      s3 *  0 +
      s4 *  2 +
      s5 *  4;

  int correction = error * 20;  // tune 10–30

  int leftMotor  = baseSpeed + correction;
  int rightMotor = baseSpeed - correction;

  setMotor(leftMotor, rightMotor);
}
