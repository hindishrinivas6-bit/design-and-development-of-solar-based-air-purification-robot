#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

// --- PIN DEFINITIONS ---
// Dust Sensor Pins
const int ledPin = 2;     
const int dustPin = A2;   // MOVED to A2 to avoid conflict with Echo pin

// Ultrasonic Pins
#define TRIG_PIN A1
#define ECHO_PIN A0
#define MAX_DISTANCE 200

// --- OBJECT INITIALIZATION ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
NewPing radar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
Servo myservo;

// --- VARIABLES ---
int sensorValue = 0;
float voltage = 0;
float dustDensity = 0;
int distance = 100;
int pos = 0;

void setup() {
  // Serial and I2C Setup
  Serial.begin(9600);
  Wire.begin();
  
  // LCD Setup
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Starting");
  
  // Dust Sensor Pin Setup
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Motor Setup
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);

  // Servo Setup
  myservo.attach(9);
  myservo.write(100);
  
  delay(2000);
  lcd.clear();
}

void loop() {
  // 1. UPDATE DUST SENSOR DATA
  updateDustSensor();

  // 2. OBSTACLE AVOIDANCE LOGIC
  int distanceRight = 0;
  int distanceLeft = 0;

  if (distance <= 15) {
    stopMovement();
    delay(100);
    moveBackward();
    delay(300);
    stopMovement();
    delay(200);
    
    distanceRight = checkRightDistance();
    delay(200);
    distanceLeft = checkLeftDistance();
    delay(200);

    if (distanceRight >= distanceLeft) {
      turnRight();
      stopMovement();
    } else {
      turnLeft();
      stopMovement();
    }
  } else {
    moveForward();
  }
  
  distance = checkDistance();
  
  // 3. LOG TO SERIAL
  Serial.print("Dist: ");
  Serial.print(distance);
  Serial.print("cm | Dust: ");
  Serial.println(dustDensity);
}

// --- DUST SENSOR FUNCTION ---
void updateDustSensor() {
  digitalWrite(ledPin, LOW); // Turn ON LED
  delayMicroseconds(280);

  sensorValue = analogRead(dustPin);

  delayMicroseconds(40);
  digitalWrite(ledPin, HIGH); // Turn OFF LED
  delayMicroseconds(9680);

  voltage = sensorValue * (5.0 / 1024.0);
  dustDensity = (voltage - 0.6) * 1000 / 0.5;
  if (dustDensity < 0) dustDensity = 0;

  // Update LCD
  lcd.setCursor(0, 0);
  lcd.print("Dust: ");
  lcd.print(dustDensity);
  lcd.print(" ug/m3   ");
  
  lcd.setCursor(0, 1);
  if(dustDensity > 150) lcd.print("Status: POOR   ");
  else lcd.print("Status: GOOD   ");
}

// --- NAVIGATION FUNCTIONS ---
int checkRightDistance() {
  for (pos = 100; pos >= 30; pos -= 1) {
    myservo.write(pos);
    delay(5);
  }
  int d = checkDistance();
  for (pos = 30; pos <= 100; pos += 1) {
    myservo.write(pos);
    delay(5);
  }
  return d;
}

int checkLeftDistance() {
  for (pos = 100; pos <= 170; pos += 1) {
    myservo.write(pos);
    delay(5);
  }
  int d = checkDistance();
  for (pos = 170; pos >= 100; pos -= 1) {
    myservo.write(pos);
    delay(5);
  }
  return d;
}

int checkDistance() {
  delay(70);
  int cm = radar.ping_cm();
  if (cm == 0) cm = 250;
  return cm;
}

void stopMovement() {
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
}

void moveForward() {
  motor1.run(FORWARD); motor2.run(FORWARD);
  motor3.run(FORWARD); motor4.run(FORWARD);
}

void moveBackward() {
  motor1.run(BACKWARD); motor2.run(BACKWARD);
  motor3.run(BACKWARD); motor4.run(BACKWARD);
}

void turnLeft() {
  motor1.run(FORWARD); motor2.run(FORWARD);
  motor3.run(BACKWARD); motor4.run(BACKWARD);
  delay(500);
}

void turnRight() {
  motor1.run(BACKWARD); motor2.run(BACKWARD);
  motor3.run(FORWARD); motor4.run(FORWARD);
  delay(500);
}
