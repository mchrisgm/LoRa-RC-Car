#include <Wire.h>
#include <ESP32Servo.h>

// Servo motor pins
const int frontLeftPin = 13;
const int frontRightPin = 12;
const int backLeftPin = 14;
const int backRightPin = 27;

// Servo objects
Servo frontLeft;
Servo frontRight;
Servo backLeft;
Servo backRight;

// Define minimum and maximum angles for the servos
const int lowestAngle = 20;  // Lower bound for servos
const int highestAngle = 160; // Upper bound for servos

// Rates for movement
const int servoIncrementRate = 5;  // Rate at which servos move towards target
const int targetIncrementRate = 5; // Rate at which target angle changes with UP/DOWN commands

// Current target and position angle for all servos
int targetAngle = 90;     // Initial target angle
int currentAngle = 90;    // Initial position of servos

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX2 on GPIO 16, TX2 on GPIO 17

  // Initialize servo motors
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  frontLeft.setPeriodHertz(50); // Standard 50hz for servos
  frontRight.setPeriodHertz(50);
  backLeft.setPeriodHertz(50);
  backRight.setPeriodHertz(50);

  frontLeft.attach(frontLeftPin, 500, 2400);
  frontRight.attach(frontRightPin, 500, 2400);
  backLeft.attach(backLeftPin, 500, 2400);
  backRight.attach(backRightPin, 500, 2400);

  // Set all servos to the new current angle
  frontLeft.write(currentAngle);
  frontRight.write(currentAngle);
  backLeft.write(currentAngle);
  backRight.write(currentAngle);
}

void loop() {
  // Task 1: Check for Serial input to adjust target angle rate
  checkSerialInput();

  // Task 3: Move servos towards the target angle
  moveServos();

  delay(10); // Small delay to control servo movement speed
}

// Function to read Serial input and determine direction to adjust target angle
void checkSerialInput() {
  if (Serial2.available() > 0) {
    String receivedData = Serial2.readStringUntil('\n');  // Read data until newline
    
    // Process the received data (extract UP and DOWN states)
    int upIndex = receivedData.indexOf("UP:");
    int downIndex = receivedData.indexOf(",DOWN:");
    
    if (upIndex != -1 && downIndex != -1) {
      bool upState = receivedData.substring(upIndex + 3, downIndex).toInt();
      bool downState = receivedData.substring(downIndex + 6).toInt();
      
      // Adjust target angle based on command
      if (upState == 1) {
        targetAngle += targetIncrementRate;
      } else if (downState == 1) {
        targetAngle -= targetIncrementRate;
      }

      // Constrain target angle within predefined limits
      targetAngle = constrain(targetAngle, lowestAngle, highestAngle);
    }
  }
}

// Function to move servos gradually towards the target angle
void moveServos() {
  // Only move if the currentAngle is different from the targetAngle
  if (currentAngle != targetAngle) {
    // Determine the direction to move based on targetAngle
    if (currentAngle > targetAngle) {
      currentAngle -= servoIncrementRate;
      if (currentAngle < targetAngle) currentAngle = targetAngle;  // Prevent overshooting
    } else if (currentAngle < targetAngle) {
      currentAngle += servoIncrementRate;
      if (currentAngle > targetAngle) currentAngle = targetAngle;  // Prevent overshooting
    }

    // Set all servos to the new current angle
    frontLeft.write(currentAngle);
    frontRight.write(currentAngle);
    backLeft.write(currentAngle);
    backRight.write(currentAngle);

    // Optional: print current angle for debugging
    // Serial.print(" Current Angle: ");
    // Serial.println(currentAngle);
  }
}
