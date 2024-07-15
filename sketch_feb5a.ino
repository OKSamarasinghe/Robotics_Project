#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>

#define servo1 0
#define servo2 1
#define servo3 2
#define servo4 3
#define IR_SENSOR_PIN 4  // Replace with the actual pin connected to the IR sensor

#define RED 0
#define GREEN 1
#define BLUE 2

const int S0 = 5;
const int S1 = 6;
const int S2 = 7;
const int S3 = 8;
const int OUT = 9;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address and pin numbers

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();

void moveServo(int servoNumber, int startPos, int endPos, int delayTime) {
  int step = (startPos < endPos) ? 1 : -1;
  for (int pos = startPos; pos != endPos + step; pos += step) {
    srituhobby.setPWM(servoNumber, 0, pos);
    delay(delayTime);
  }
}

void setup() {
  Serial.begin(9600);
  srituhobby.begin();
  srituhobby.setPWMFreq(60);

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Setting frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Set initial LCD message
  Serial.println("Initializing LCD...");
  lcd.begin(16, 2);  // Initialize the LCD with 16 columns and 2 rows
  lcd.backlight();   // Turn on the backlight if your LCD has one
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  delay(3000);
  lcd.clear();
}

void loop() {
  int irSensorValue = digitalRead(IR_SENSOR_PIN);

  Serial.print("IR Sensor Value: ");
  Serial.println(irSensorValue);

  if (irSensorValue == LOW) {

    lcd.setCursor(0, 0);
    lcd.print("DANGER KEEP CLEAR");
    // Color sensor active only when IR sensor is low
    int red = getColor(RED);
    int green = getColor(GREEN);
    int blue = getColor(BLUE);

    Serial.print("Red: "); Serial.print(red);
    Serial.print(", Green: "); Serial.print(green);
    Serial.print(", Blue: "); Serial.println(blue);

    // Perform movements based on color
    if (red >= 60 || red < 75) {
      // Red detected, perform movements for red
      for (int S1value = 360; S1value >= 180; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }

  for (int S2value = 410; S2value <= 450; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S3value = 180; S3value <= 220; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S4value = 560; S4value <= 610; S4value++) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }
  ////////////////////////
  delay(1000);
  for (int S4value = 610; S4value > 560; S4value--) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }

  for (int S3value = 220; S3value > 180; S3value--) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S2value = 450; S2value > 410; S2value--) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S1value = 180; S1value < 450; S1value++) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }
  //////////////////////
  for (int S2value = 410; S2value <= 450; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S3value = 180; S3value <= 220; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S4value = 560; S4value <= 610; S4value++) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }

  for (int S4value = 610; S4value > 560; S4value--) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }
  ///////////////////
  for (int S3value = 220; S3value > 180; S3value--) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S2value = 450; S2value > 410; S2value--) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S1value = 450; S1value > 330; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }
      // Add other movements for red color if needed
    } else if (green > 200) {
      // Green detected, perform movements for green
      for (int S1value = 360; S1value >= 180; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }

  for (int S2value = 410; S2value <= 450; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S3value = 180; S3value <= 220; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S4value = 560; S4value <= 610; S4value++) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }
  ////////////////////////
  delay(1000);
  for (int S4value = 610; S4value > 560; S4value--) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }

  for (int S3value = 220; S3value > 180; S3value--) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S2value = 450; S2value > 410; S2value--) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S1value = 180; S1value < 450; S1value++) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }
  //////////////////////
  for (int S2value = 410; S2value <= 450; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S3value = 180; S3value <= 220; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S4value = 560; S4value <= 610; S4value++) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }

  for (int S4value = 610; S4value > 560; S4value--) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }
  ///////////////////
  for (int S3value = 220; S3value > 180; S3value--) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S2value = 450; S2value > 410; S2value--) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S1value = 450; S1value > 330; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }
      // Add other movements for green color if needed
    } else if (blue > 200) {
      // Blue detected, perform movements for blue
      for (int S1value = 360; S1value >= 180; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }

  for (int S2value = 410; S2value <= 450; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S3value = 180; S3value <= 220; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S4value = 560; S4value <= 610; S4value++) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }
  ////////////////////////
  delay(1000);
  for (int S4value = 610; S4value > 560; S4value--) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }

  for (int S3value = 220; S3value > 180; S3value--) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S2value = 450; S2value > 410; S2value--) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S1value = 180; S1value < 450; S1value++) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }
  //////////////////////
  for (int S2value = 410; S2value <= 450; S2value++) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S3value = 180; S3value <= 220; S3value++) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S4value = 560; S4value <= 610; S4value++) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }

  for (int S4value = 610; S4value > 560; S4value--) {
    srituhobby.setPWM(servo4, 0, S4value);
    delay(10);
  }
  ///////////////////
  for (int S3value = 220; S3value > 180; S3value--) {
    srituhobby.setPWM(servo3, 0, S3value);
    delay(10);
  }

  for (int S2value = 450; S2value > 410; S2value--) {
    srituhobby.setPWM(servo2, 0, S2value);
    delay(10);
  }

  for (int S1value = 450; S1value > 330; S1value--) {
    srituhobby.setPWM(servo1, 0, S1value);
    delay(10);
  }
      // Add other movements for blue color if needed
    }
  } else {

    lcd.setCursor(0, 0);
    lcd.print("No Danger");
    // No object detected by IR sensor, return to initial position
      srituhobby.begin();
  srituhobby.setPWMFreq(60);
  srituhobby.setPWM(servo1, 0, 330);
  srituhobby.setPWM(servo2, 0, 400);
  srituhobby.setPWM(servo3, 0, 160);
  srituhobby.setPWM(servo4, 0, 555);
  
  }

  delay(1000);

}

int getColor(int color) {
   switch (color) {
    case RED:
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      break;
    case GREEN:
      digitalWrite(S2, HIGH);
      digitalWrite(S3, HIGH);
      break;
    case BLUE:
      digitalWrite(S2, LOW);
      digitalWrite(S3, HIGH);
      break;
  }

  // Read the frequency
  int frequency = pulseIn(OUT, LOW);

  // Convert the frequency to color intensity
  int intensity = map(frequency, 50, 700, 0, 255);

  return intensity;
}
