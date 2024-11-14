#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO08x.h>

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

// Height setup
const float pitchSetup = -7.00;   // Desired pitch angle to maintain
const float rollSetup = 0.00;     // Desired roll angle to maintain

// Current target and position angles for all servos
int targetAngles[numMainServos];    // Target angles for main servos
int currentAngles[numMainServos];   // Current positions of main servos
int steeringAngle = 90;             // Current angle for the steering servo
int targetSteeringAngle = 90;       // Target angle for steering servo

// Height offset from UP/DOWN commands
int heightOffset = 0; // Initialize height offset

// Sensor includes and definitions
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Sensor report type and interval
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

// Euler angles structure
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

// Correction factors for servo adjustments
const float pitchCorrectionFactor = 0.5; // Adjust as necessary
const float rollCorrectionFactor = 0.5;  // Adjust as necessary

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

  // Initialize BNO08x sensor
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  // Enable sensor reports
  setReports(reportType, reportIntervalUs);
}

// Function to set desired sensor reports
void setReports(sh2_SensorId_t reportType, long report_interval) {
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable desired sensor report");
  }
}

// Function to convert quaternion to Euler angles
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {
  // Task 1: Check for Serial input to adjust heightOffset and steering angle
  checkSerialInput();

  // Task 2: Read sensor data and adjust servos based on sensor readings
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }

    // Adjust servos based on sensor readings and heightOffset
    adjustServosBasedOnSensor();
  }

  // Task 3: Move servos towards their target angles
  moveServos();

  delay(5); // Small delay to control servo movement speed
}

// Function to adjust servos based on pitch and roll errors
void adjustServosBasedOnSensor() {
  float pitchError = pitchSetup - ypr.pitch; // Desired pitch - current pitch
  float rollError = rollSetup - ypr.roll;    // Desired roll - current roll

  // Calculate adjustments
  float pitchAdjustment = pitchError * pitchCorrectionFactor;
  float rollAdjustment = rollError * rollCorrectionFactor;

  // Initialize target angles to a base value (e.g., 90 degrees)
  for (int i = 0; i < numMainServos; i++) {
    targetAngles[i] = 90;
  }

  // Adjust front servos for pitch
  targetAngles[0] += pitchAdjustment; // Front Left
  targetAngles[1] += pitchAdjustment; // Front Right

  // Adjust back servos for pitch
  targetAngles[2] -= pitchAdjustment; // Back Left
  targetAngles[3] -= pitchAdjustment; // Back Right

  // Adjust left servos for roll
  targetAngles[0] += rollAdjustment; // Front Left
  targetAngles[2] += rollAdjustment; // Back Left

  // Adjust right servos for roll
  targetAngles[1] -= rollAdjustment; // Front Right
  targetAngles[3] -= rollAdjustment; // Back Right

  // Add heightOffset to all servos equally
  for (int i = 0; i < numMainServos; i++) {
    targetAngles[i] += heightOffset;
    // Constrain target angles within servo limits
    targetAngles[i] = constrain(targetAngles[i], lowestAngle, highestAngle);
  }
}

// Function to read Serial input and determine directions for heightOffset and steering
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
        // Serial.println("Invalid message or checksum error!"); // Commented out to avoid printing
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

    // Adjust heightOffset based on UP and DOWN commands
    if (upState == 1) {
      heightOffset += targetIncrementRate;
    } else if (downState == 1) {
      heightOffset -= targetIncrementRate;
    }

    // Ensure heightOffset doesn't cause servos to exceed their limits
    // You may need to adjust this logic based on your servo arrangement
    heightOffset = constrain(heightOffset, lowestAngle - 90, highestAngle - 90);

    // Map the steering value (leftStickX) from 0-100 to servo angle range
    targetSteeringAngle = map(steerValue, 0, 100, steeringMinAngle, steeringMaxAngle);
    targetSteeringAngle = constrain(targetSteeringAngle, steeringMinAngle, steeringMaxAngle);
  } else {
    // Serial.println("Parsing error: Incorrect command structure."); // Commented out to avoid printing
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