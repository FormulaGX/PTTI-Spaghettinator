#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <NewPing.h>

// LCD Setup Data
#define LCD_COLS 16
#define LCD_ROWS 2

// Sonar Setup Data
#define SONAR_TRIGGER_PIN 10
#define SONAR_ECHO_PIN 13
#define SONAR_MAX_DISTANCE 400

// Proximity Sensor Setup Data
#define PROX_PIN 4
bool proxStatus;

// Motor Setup Data
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1
int motorState; // 0 = stationary/retracted, 1 = stationary/extended

// Devices
hd44780_I2Cexp lcd; //LCD display with I2C backpack extension
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE); //Sonar sensor
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin); // Stepper motor

// Time variables
unsigned long currentMillis, sonarMillis, motorMillis;

// Sonar variables
// Duration is the time it takes for the sonar recieve a signal
// Distance is half of the duration multiplied by the speed of sound in centimeters
float duration, distance;

void setup() {

  Serial.begin(9600); // Establish connection with the device at 9.6Kb/s
  pinMode(PROX_PIN, INPUT_PULLUP); // Use the proximity sensor pin

  int lcd_status; // LCD error code variable

  // Set the timers for the sonar and motors in milliseconds
  sonarMillis = 0;
  motorMillis = 0;

  // Set motor status
  motorState = 0;

  lcd_status = lcd.begin(LCD_COLS, LCD_ROWS); // Start the LCD and enable backlighting

  // Set the maximum speed and acceleration of the motor
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Move the motor to its zero position
  stepper.moveTo(80);
  stepper.runToPosition();
  stepper.moveTo(0);
  stepper.runToPosition();

  if (lcd_status) // Non-Zero status means it was unsuccesful
  {
    // Begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(lcd_status); // Does not return
  }

  Serial.print("Startup Complete");
  lcd.lineWrap(); // Enable LCD text wrapping
  lcd.print("Startup Complete");
  delay(1000);
  lcd.noLineWrap(); // Disable LCD text wrapping
}

void loop() {

  currentMillis = millis(); // Retrieve the program's uptime in milliseconds
  proxStatus = digitalRead(PROX_PIN); // Read the status of the proximity sensor pin

  // Sonar controls
  // If 250 milliseconds have elapsed, or the program has just started, do the following:
  if (currentMillis - sonarMillis >= 250 || sonarMillis == 0) {
    
    sonarMillis = currentMillis;
    duration = sonar.ping();  //Deploy the sonar sensor
    distance = (duration / 2) * 0.0343; //Convert the sonar reading to centimeters
    lcd.clear(); //Clear the LCD display before every tick.

    // If an object is within the boundaries of 2cm ~ 400cm, do the following:
    if (distance >= 400 || distance <= 2) {
      
      Serial.println("Out of range");
      lcd.setCursor(0, 0);
      lcd.print("Out of range");
    }
    else {
      
      Serial.println("Distance: " + String(distance) + "cm");
      lcd.setCursor(0, 0);
      lcd.print("Dist: " + String(distance) + "cm");
    }
  }

  // Motor controls
  // This switch statement checks the motor state integer
  // 0 = stationary/retracted, 1 = stationary/extended, 2 = moving/retracting, 4 = moving/extending

  if (motorState == 0) {
    if ((currentMillis - motorMillis >= 1000 || motorMillis == 0) && distance <= 7) {
      
      motorState = 1; // Extend the motor
      motorMillis = currentMillis;
      Serial.println("Extending motor");
      stepper.moveTo(80); // Set the target position
      stepper.runToPosition();
    }
    else if (currentMillis - motorMillis >= 250) {
      
      lcd.setCursor(0,1);
      lcd.print("Waiting");
    }
  }
  if (motorState == 1) {
    
        if (currentMillis - motorMillis >= 1000) {
        
        motorState = 0; // Retract the motor
        motorMillis = currentMillis;
        Serial.println("Retracting motor");
        stepper.moveTo(0);
        stepper.runToPosition();
      }
  }
}
