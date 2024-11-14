#include <Wire.h>
#include <ESP32Servo.h>

// Servo motor pins
const int frontLeftPin = 13;
const int frontRightPin = 12;
const int backLeftPin = 14;
const int backRightPin = 27;
const int steerPin = 25; // Steering servo pin

// Servo objects
Servo frontLeft;
Servo frontRight;
Servo backLeft;
Servo backRight;
Servo steeringServo; // Servo for steering

// Define minimum and maximum angles for the servos
const int lowestAngle = 20;     // Lower bound for main servos
const int highestAngle = 160;   // Upper bound for main servos
const int steeringMinAngle = 0; // Steering angle minimum
const int steeringMaxAngle = 180; // Steering angle maximum

// Rates for movement
const int servoIncrementRate = 1;  // Rate at which servos move towards target
const int targetIncrementRate = 1; // Rate at which target angle changes with UP/DOWN commands

// Current target and position angle for all servos
int targetAngle = 90;   // Initial target angle for main servos
int currentAngle = 90;  // Initial position of main servos
int steeringAngle = 90; // Initial angle for the steering servo

void setup() {
  Serial.begin(115200);
  Serial2.begin(19200, SERIAL_8N1, 16, 17); // RX2 on GPIO 16, TX2 on GPIO 17

  // Initialize servo motors
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  frontLeft.setPeriodHertz(50); // Standard 50Hz for servos
  frontRight.setPeriodHertz(50);
  backLeft.setPeriodHertz(50);
  backRight.setPeriodHertz(50);
  steeringServo.setPeriodHertz(50);

  frontLeft.attach(frontLeftPin, 500, 2400);
  frontRight.attach(frontRightPin, 500, 2400);
  backLeft.attach(backLeftPin, 500, 2400);
  backRight.attach(backRightPin, 500, 2400);
  steeringServo.attach(steerPin, 500, 2400);

  // Set initial positions for all servos
  frontLeft.write(currentAngle);
  frontRight.write(currentAngle);
  backLeft.write(currentAngle);
  backRight.write(currentAngle);
  steeringServo.write(steeringAngle);
}

void loop() {
  // Task 1: Check for Serial input to adjust target angle and steering angle
  checkSerialInput();

  // Task 2: Move servos towards the target angle
  moveServos();

  delay(5); // Small delay to control servo movement speed
}

// Function to read Serial input and determine directions for main servos and steering
void checkSerialInput() {
  while (Serial2.available() > 0) {
    String receivedData = Serial2.readStringUntil('>\n'); // Read until end marker
    Serial.println(receivedData);

    // Verify if the message starts with the start marker '<'
    if (receivedData.startsWith("<")) {
      receivedData = receivedData.substring(1); // Remove the start marker '<'

      // Process and validate the message format and checksum
      if (isValidMessage(receivedData)) {
        // Process the message only if it is valid
        parseMessage(receivedData);
      } else {
        Serial.println("Invalid message or checksum error!");
      }
    }
  }
}

// Function to validate message format and checksum
bool isValidMessage(String message) {
  int checksumIndex = message.indexOf(",CHECKSUM:");
  if (checksumIndex == -1) {
    return false; // Checksum not found
  }

  // Extract the checksum from the message
  String checksumStr = message.substring(checksumIndex + 10); // Skip ",CHECKSUM:"
  int receivedChecksum = checksumStr.toInt();

  // Calculate the checksum for the message (excluding ",CHECKSUM:" part)
  String dataToCheck = message.substring(0, checksumIndex);
  int calculatedChecksum = 0;
  for (int i = 0; i < dataToCheck.length(); i++) {
    calculatedChecksum += dataToCheck[i];
  }
  calculatedChecksum %= 256; // Keep within 0-255

  // Compare calculated and received checksum
  return calculatedChecksum == receivedChecksum;
}

// Function to parse and apply command values from the validated message
void parseMessage(String message) {
  int upIndex = message.indexOf("UP:");
  int downIndex = message.indexOf(",DOWN:");
  int steerIndex = message.indexOf(",STEER:");

  if (upIndex != -1 && downIndex != -1 && steerIndex != -1) {
    Serial.println(message); // For debugging

    // Extract and convert the UP, DOWN, and STEER values
    String upValueStr = message.substring(upIndex + 3, downIndex);
    String downValueStr = message.substring(downIndex + 6, steerIndex);
    String steerValueStr = message.substring(steerIndex + 7);

    // Convert the extracted strings to integers
    int upState = upValueStr.toInt();
    int downState = downValueStr.toInt();
    int steerValue = steerValueStr.toInt();

    // Adjust target angle for main servos based on UP and DOWN commands
    if (upState == 1) {
      targetAngle += targetIncrementRate;
    } else if (downState == 1) {
      targetAngle -= targetIncrementRate;
    }

    // Constrain target angle within predefined limits for main servos
    targetAngle = constrain(targetAngle, lowestAngle, highestAngle);

    // Map the steering value (leftStickX) from 0-100 to servo angle range
    steeringAngle = map(steerValue, 0, 100, steeringMinAngle, steeringMaxAngle);
    steeringAngle = constrain(steeringAngle, steeringMinAngle, steeringMaxAngle);
  } else {
    Serial.println("Parsing error: Incorrect command structure.");
  }
}

// Function to move main servos gradually towards the target angle
void moveServos() {
  // Move main servos if currentAngle is different from targetAngle
  if (currentAngle != targetAngle) {
    if (currentAngle > targetAngle) {
      currentAngle -= servoIncrementRate;
      if (currentAngle < targetAngle)
        currentAngle = targetAngle; // Prevent overshooting
    } else if (currentAngle < targetAngle) {
      currentAngle += servoIncrementRate;
      if (currentAngle > targetAngle)
        currentAngle = targetAngle; // Prevent overshooting
    }

    // Set all main servos to the new current angle
    frontLeft.write(currentAngle);
    frontRight.write(currentAngle);
    backLeft.write(currentAngle);
    backRight.write(currentAngle);
  }

  // Update steering servo if the angle has changed
  if (steeringServo.read() != steeringAngle) {
    steeringServo.write(steeringAngle);
  }

  // Optional: print current angle for debugging
  // Serial.print(" Current Angle: ");
  // Serial.print(currentAngle);

  // Serial.print(" Servo Steering Angle: ");
  // Serial.print(steeringServo.read());

  // Serial.print(" Steering Angle: ");
  // Serial.println(steeringAngle);
}
