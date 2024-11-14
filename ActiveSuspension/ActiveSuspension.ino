#include <Wire.h>
#include <ESP32Servo.h>

// Servo motor pins
const int mainServoPins[] = {13, 12, 14, 27}; // Pins for the main servos
const int numMainServos = sizeof(mainServoPins) / sizeof(mainServoPins[0]); // Number of main servos
const int steerPin = 25; // Steering servo pin

// Servo objects
Servo mainServos[numMainServos];
Servo steeringServo; // Servo for steering

// Define minimum and maximum angles for the servos
const int lowestAngle = 20;      // Lower bound for main servos
const int highestAngle = 160;    // Upper bound for main servos
const int steeringMinAngle = 0;  // Steering angle minimum
const int steeringMaxAngle = 180; // Steering angle maximum

// Rates for movement
const int servoIncrementRate = 1;  // Rate at which servos move towards target
const int targetIncrementRate = 1; // Rate at which target angle changes with UP/DOWN commands
const int steeringIncrementRate = 10;

// Current target and position angles for all servos
int targetAngles[numMainServos];    // Target angles for main servos
int currentAngles[numMainServos];   // Current positions of main servos
int steeringAngle = 90;             // Current angle for the steering servo
int targetSteeringAngle = 90;       // Target angle for steering servo

void setup() {
  Serial.begin(115200);
  Serial2.begin(19200, SERIAL_8N1, 16, 17); // RX2 on GPIO 16, TX2 on GPIO 17

  // Initialize servo motors
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Initialize main servos
  for (int i = 0; i < numMainServos; i++) {
    mainServos[i].setPeriodHertz(50); // Standard 50Hz for servos
    mainServos[i].attach(mainServoPins[i], 500, 2400);
    targetAngles[i] = 90;    // Initial target angle
    currentAngles[i] = 90;   // Initial current angle
    mainServos[i].write(currentAngles[i]); // Set initial position
  }

  // Initialize steering servo
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(steerPin, 500, 2400);
  steeringServo.write(steeringAngle);
}

void loop() {
  // Task 1: Check for Serial input to adjust target angles and steering angle
  checkSerialInput();

  // Task 2: Move servos towards their target angles
  moveServos();

  delay(5); // Small delay to control servo movement speed
}

// Function to read Serial input and determine directions for main servos and steering
void checkSerialInput() {
  while (Serial2.available() > 0) {
    String receivedData = Serial2.readStringUntil('>\n'); // Read until end marker
    receivedData += '>'; // Add the '>' back to the string

    // Verify if the message starts with the start marker '<'
    if (receivedData.startsWith("<")) {
      receivedData = receivedData.substring(1, receivedData.length() - 1); // Remove '<' and '>'
      
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
  for (unsigned int i = 0; i < dataToCheck.length(); i++) {
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
    // Extract and convert the UP, DOWN, and STEER values
    String upValueStr = message.substring(upIndex + 3, downIndex);
    String downValueStr = message.substring(downIndex + 6, steerIndex);
    String steerValueStr = message.substring(steerIndex + 7);

    // Convert the extracted strings to integers
    int upState = upValueStr.toInt();
    int downState = downValueStr.toInt();
    int steerValue = steerValueStr.toInt();

    // Adjust target angles for main servos based on UP and DOWN commands
    if (upState == 1) {
      for (int i = 0; i < numMainServos; i++) {
        targetAngles[i] += targetIncrementRate;
        targetAngles[i] = constrain(targetAngles[i], lowestAngle, highestAngle);
      }
    } else if (downState == 1) {
      for (int i = 0; i < numMainServos; i++) {
        targetAngles[i] -= targetIncrementRate;
        targetAngles[i] = constrain(targetAngles[i], lowestAngle, highestAngle);
      }
    }

    // Map the steering value (leftStickX) from 0-100 to servo angle range
    targetSteeringAngle = map(steerValue, 0, 100, steeringMinAngle, steeringMaxAngle);
    targetSteeringAngle = constrain(targetSteeringAngle, steeringMinAngle, steeringMaxAngle);
  } else {
    Serial.println("Parsing error: Incorrect command structure.");
  }
}

// Function to move servos gradually towards their target angles
void moveServos() {
  // Move main servos
  for (int i = 0; i < numMainServos; i++) {
    if (currentAngles[i] != targetAngles[i]) {
      if (currentAngles[i] > targetAngles[i]) {
        currentAngles[i] -= servoIncrementRate;
        if (currentAngles[i] < targetAngles[i]) {
          currentAngles[i] = targetAngles[i]; // Prevent overshooting
        }
      } else if (currentAngles[i] < targetAngles[i]) {
        currentAngles[i] += servoIncrementRate;
        if (currentAngles[i] > targetAngles[i]) {
          currentAngles[i] = targetAngles[i]; // Prevent overshooting
        }
      }
      // Set servo to the new current angle
      mainServos[i].write(currentAngles[i]);
    }
  }

  // Move steering servo towards target angle
  if (steeringAngle != targetSteeringAngle) {
    if (steeringAngle > targetSteeringAngle) {
      steeringAngle -= steeringIncrementRate;
      if (steeringAngle < targetSteeringAngle) {
        steeringAngle = targetSteeringAngle; // Prevent overshooting
      }
    } else if (steeringAngle < targetSteeringAngle) {
      steeringAngle += steeringIncrementRate;
      if (steeringAngle > targetSteeringAngle) {
        steeringAngle = targetSteeringAngle; // Prevent overshooting
      }
    }
    steeringServo.write(steeringAngle);
  }
}
