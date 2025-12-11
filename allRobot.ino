// Mega 2560 → Motor Driver
// -------------------------------------
// Pin 22 → IN1 (Left motor forward)
// Pin 23 → IN2 (Left motor backward)
// Pin 24 → IN3 (Right motor forward)
// Pin 25 → IN4 (Right motor backward)

// Pin 6  → ENA (Left Motor PWM)
// Pin 7  → ENB (Right Motor PWM)

// IR SENSOR ARRAY (select 5/6/8 sensors)
// Mega Pins Suggested for IR Array:

// IR1 → Pin 30  
// IR2 → Pin 31  
// IR3 → Pin 32  
// IR4 → Pin 33  
// IR5 → Pin 34  
// IR6 → Pin 35  
// IR7 → Pin 36  
// IR8 → Pin 37  

// Ultrasonic Sensor
// TRIG → Pin 26  
// ECHO → Pin 27

// TCS3200 Color Sensor
// S0  → Pin 40  
// S1  → Pin 41  
// S2  → Pin 42  
// S3  → Pin 43  
// OUT → Pin 44

// TCS34725 Color Sensor (I2C)
// SDA → Pin 20  
// SCL → Pin 21

// Servos (Gripper + Sorter Arm)
// Gripper Servo → Pin 8  
// Sorter Servo  → Pin 9


==================== MOTORS =====================
MEGA Pin 22 → IN1
MEGA Pin 23 → IN2
MEGA Pin 24 → IN3
MEGA Pin 25 → IN4
MEGA Pin 6  → ENA (PWM)
MEGA Pin 7  → ENB (PWM)

==================== IR ARRAY =====================
IR1 → 30
IR2 → 31
IR3 → 32
IR4 → 33
IR5 → 34
IR6 → 35
IR7 → 36
IR8 → 37

==================== ULTRASONIC =====================
TRIG → 26
ECHO → 27

==================== SERVOS =====================
Gripper → 8
Sorter  → 9

==================== TCS3200 =====================
S0  → 40
S1  → 41
S2  → 42
S3  → 43
OUT → 44

==================== TCS34725 =====================
SDA → 20
SCL → 21


// =======================================================
//   UNIVERSAL INDIA-SKILLS SUPER ROBOT (MEGA VERSION)
//   Supports: 5/6/8 IR sensors, Analog/Digital
//   Line Follow (IF/PID), Sorting, TCS3200/TCS34725,
//   Junction Detection + Master Mode
// =======================================================

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// -------------------------------------------------------
//                CONFIGURATIONS
// -------------------------------------------------------

// ----- Robot Modes -----
#define MODE_LINE_ONLY       1
#define MODE_LINE_OBSTACLE   2
#define MODE_SORT_ONLY       3
#define MODE_ARM_DEMO        4
#define MODE_MASTER          5

int ROBOT_MODE = MODE_MASTER;

// ----- IR Sensor Settings -----
int sensorCount = 5;   // 5,6,8
bool useAnalog = false;
int analogThreshold = 600;

// ----- Color Sensor -----
bool useTCS3200  = false;
bool useTCS34725 = true;

// ----- Keyboard -----
bool enableKeyboard = false;


// -------------------------------------------------------
//                PIN DEFINITIONS (MEGA)
// -------------------------------------------------------

// Motors
int IN1 = 22;
int IN2 = 23;
int IN3 = 24;
int IN4 = 25;
int ENA = 6;
int ENB = 7;

// IR Array pins
int sensorPins[8] = {30, 31, 32, 33, 34, 35, 36, 37};

// Ultrasonic
int TRIG_PIN = 26;
int ECHO_PIN = 27;

// Servos
Servo gripper;
Servo sorter;
int GRIP_PIN = 8;
int SORTER_PIN = 9;

// Sorting Angles
int gripOpen = 30;
int gripClose = 90;
int posHome = 90;
int posRed = 140;
int posGreen = 90;
int posBlue = 40;

// TCS3200 pins
int S0 = 40;
int S1 = 41;
int S2 = 42;
int S3 = 43;
int OUT_PIN = 44;

// TCS34725
Adafruit_TCS34725 tcs34725 =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


// -------------------------------------------------------
//                GLOBAL VARIABLES
// -------------------------------------------------------
int digitalValues[8];
int rawAnalog[8];
int baseSpeed = 90;

float Kp = 25;
float Kd = 15;
int lastError = 0;


// =======================================================
//                MOTOR CONTROL
// =======================================================
void setMotor(int L, int R) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  if (L >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, L);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -L);
  }

  if (R >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, R);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, -R);
  }
}

void forward()  { setMotor(baseSpeed, baseSpeed); }
void stopBot()  { setMotor(0, 0); }
void slightLeft() { setMotor(baseSpeed - 40, baseSpeed + 40); }
void slightRight(){ setMotor(baseSpeed + 40, baseSpeed - 40); }


// =======================================================
//               READ IR SENSORS
// =======================================================
void readIRSensors() {
  for (int i = 0; i < sensorCount; i++) {
    if (useAnalog) {
      rawAnalog[i] = analogRead(sensorPins[i]);
      digitalValues[i] = (rawAnalog[i] < analogThreshold) ? 1 : 0;
    } else {
      digitalValues[i] = digitalRead(sensorPins[i]);
    }
  }

  Serial.print("IR: ");
  for (int i = 0; i < sensorCount; i++)
    Serial.print(String(digitalValues[i]) + " ");
  Serial.println();
}


// =======================================================
//               IF–ELSE LINE FOLLOWER
// =======================================================
void lineFollow_IF() {
  readIRSensors();

  int midL = (sensorCount - 1) / 2;
  int midR = sensorCount / 2;

  if (digitalValues[midL] && digitalValues[midR]) {
    forward();
    return;
  }
  for (int i = midR + 1; i < sensorCount; i++)
    if (digitalValues[i]) { slightLeft(); return; }

  for (int i = midL - 1; i >= 0; i--)
    if (digitalValues[i]) { slightRight(); return; }

  stopBot();
}


// =======================================================
//                 PID LINE FOLLOWER
// =======================================================
void lineFollow_PID() {
  readIRSensors();

  int sum = 0, count = 0;
  for (int i = 0; i < sensorCount; i++)
    if (digitalValues[i]) { sum += i; count++; }

  if (count == 0) { stopBot(); return; }

  int pos = sum / count;
  int center = (sensorCount - 1) / 2;

  int error = pos - center;
  int correction = Kp * error + Kd * (error - lastError);
  lastError = error;

  setMotor(baseSpeed - correction, baseSpeed + correction);
}


// =======================================================
//                 ULTRASONIC
// =======================================================
long getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long t = pulseIn(ECHO_PIN, HIGH);
  return (t == 0) ? 999 : t / 58;
}


// =======================================================
//             JUNCTION DETECTION
// =======================================================
bool isJunction() {
  int sum = 0;
  for (int i = 0; i < sensorCount; i++) sum += digitalValues[i];
  return (sum >= sensorCount - 1);
}

void goToStation(int station) {
  int count = 0;
  unsigned long last = 0;

  while (count < station) {
    lineFollow_IF();

    if (isJunction() && millis() - last > 500) {
      count++;
      last = millis();
      Serial.println("Junction " + String(count));
    }
  }

  forward(); delay(400); stopBot();
}


// =======================================================
//     COLOR SENSOR: TCS3200 (FREQUENCY)
// =======================================================
unsigned long readFreq(bool s2, bool s3) {
  digitalWrite(S2, s2); 
  digitalWrite(S3, s3);
  delay(20);

  long duration = pulseIn(OUT_PIN, LOW, 25000);
  if (duration == 0) duration = 25000;

  return duration;
}

String getColor_TCS3200() {
  long r = readFreq(LOW, LOW);
  long g = readFreq(HIGH, HIGH);
  long b = readFreq(LOW, HIGH);

  if (r < g && r < b) return "RED";
  if (g < r && g < b) return "GREEN";
  if (b < r && b < g) return "BLUE";

  return "UNKNOWN";
}


// =======================================================
//     COLOR SENSOR: TCS34725 (I2C)
// =======================================================
String getColor_TCS34725() {
  uint16_t r, g, b, c;
  tcs34725.getRawData(&r, &g, &b, &c);

  float R = r / (float)c;
  float G = g / (float)c;
  float B = b / (float)c;

  if (R > G && R > B) return "RED";
  if (G > R && G > B) return "GREEN";
  if (B > R && B > G) return "BLUE";

  return "UNKNOWN";
}

String detectColor() {
  if (useTCS3200)  return getColor_TCS3200();
  if (useTCS34725) return getColor_TCS34725();
  return "UNKNOWN";
}


// =======================================================
//         PICKUP & SORTING FUNCTIONS
// =======================================================
void pickObject() {
  gripper.write(gripOpen); delay(300);
  gripper.write(gripClose); delay(700);
}

void sortObject(String col) {
  if (col == "RED") sorter.write(posRed);
  else if (col == "GREEN") sorter.write(posGreen);
  else if (col == "BLUE") sorter.write(posBlue);
  else sorter.write(posGreen);

  delay(800);
  gripper.write(gripOpen); delay(400);
  sorter.write(posHome);
}


// =======================================================
//                KEYBOARD CONTROL
// =======================================================
void handleKeyboard(char k) {
  if (k=='w') forward();
  if (k=='s') stopBot();
  if (k=='a') slightLeft();
  if (k=='d') slightRight();

  if (k=='o') gripper.write(gripOpen);
  if (k=='c') gripper.write(gripClose);

  if (k=='r') sorter.write(posRed);
  if (k=='g') sorter.write(posGreen);
  if (k=='b') sorter.write(posBlue);
  if (k=='h') sorter.write(posHome);

  if (k=='x') {
    Serial.println("Color: " + detectColor());
  }
}


// =======================================================
//                        SETUP
// =======================================================
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  for (int i = 0; i < sensorCount; i++)
    pinMode(sensorPins[i], INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  gripper.attach(GRIP_PIN);
  sorter.attach(SORTER_PIN);

  gripper.write(gripOpen);
  sorter.write(posHome);

  if (useTCS3200) {
    pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
    pinMode(OUT_PIN, INPUT);

    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);
  }

  if (useTCS34725) {
    if (!tcs34725.begin()) Serial.println("TCS34725 Not Found");
    else Serial.println("TCS34725 Ready");
  }

  Serial.println("MEGA SUPER ROBOT READY!");
}


// =======================================================
//                         LOOP
// =======================================================
void loop() {

  if (enableKeyboard && Serial.available())
    handleKeyboard(Serial.read());

  if (ROBOT_MODE == MODE_LINE_ONLY)
      lineFollow_IF();

  else if (ROBOT_MODE == MODE_LINE_OBSTACLE) {
    if (getDistance() < 15) stopBot();
    else lineFollow_IF();
  }

  else if (ROBOT_MODE == MODE_SORT_ONLY) {
    pickObject();
    sortObject(detectColor());
  }

  else if (ROBOT_MODE == MODE_ARM_DEMO) {
    sorter.write(posRed); delay(700);
    sorter.write(posGreen); delay(700);
    sorter.write(posBlue); delay(700);
    sorter.write(posHome); delay(700);
  }

  else if (ROBOT_MODE == MODE_MASTER) {
    goToStation(1);
    pickObject();
    String color = detectColor();
    goToStation(2);
    sortObject(color);
    goToStation(3);
    stopBot();
    while (1);
  }
}
