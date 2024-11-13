#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_ICM20948.h>  // Ensure this library is installed

// Initialize the ICM20948 IMU
Adafruit_ICM20948 icm;

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

// Variables for servo angles
int frontServoAngle = 90;
int backServoAngle = 90;
int angleIncrement = 1;

// Variables for orientation calculations (roll, pitch, yaw)
float roll = 0, pitch = 0, yaw = 0;
float previousTime = 0;  // Used for time delta in gyro integration

// Magnetometer data
float magX, magY, magZ;

// Complementary filter constants
const float alpha = 0.98; // Weight for accelerometer vs gyroscope
const float beta = 0.5;   // Weight for magnetometer (yaw correction)

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22

  // Initialize the ICM20948 IMU
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1);
  }
  Serial.println("ICM20948 Found!");

  // Set up IMU configurations
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

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

  previousTime = millis() / 1000.0;  // Initialize previousTime in seconds
}

void loop() {
  // Task 1: Read IMU data and calculate orientation
  calculateOrientation();

  // Task 2: Move servos
  moveServos();

  delay(10); // Small delay to prevent flooding the Serial Monitor
}

void calculateOrientation() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Calculate time delta
  float currentTime = millis() / 1000.0;
  float deltaTime = currentTime - previousTime;
  previousTime = currentTime;

  // Accelerometer-based roll and pitch calculations
  float accelRoll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
  float accelPitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;

  // Integrate gyroscope data to calculate yaw (Z-axis rotation)
  yaw += gyro.gyro.z * deltaTime;

  // Magnetometer-based heading (yaw adjustment)
  float magHeading = atan2(mag.magnetic.y, mag.magnetic.x) * 180.0 / PI;

  // Complementary filter for roll and pitch
  roll = alpha * (roll + gyro.gyro.x * deltaTime) + (1 - alpha) * accelRoll;
  pitch = alpha * (pitch + gyro.gyro.y * deltaTime) + (1 - alpha) * accelPitch;

  // Adjust yaw with magnetometer data
  yaw = (1 - beta) * yaw + beta * magHeading;

  // Print calculated orientation to Serial
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Yaw: "); Serial.println(yaw);
}

void moveServos() {
  // Set front servos to the same angle within 20 to 160 degrees
  frontLeft.write(frontServoAngle);
  frontRight.write(frontServoAngle);

  // Set back servos to the same angle within 20 to 160 degrees
  backLeft.write(backServoAngle);
  backRight.write(backServoAngle);

  // Update front and back servo angles for the next iteration
  frontServoAngle += angleIncrement;
  backServoAngle += angleIncrement;

  // Reverse direction if the angle exceeds the range for front and back servos
  if (frontServoAngle >= 160 || frontServoAngle <= 20 || backServoAngle >= 160 || backServoAngle <= 20) {
    angleIncrement = -angleIncrement; // Reverse increment direction
  }

  delay(15); // Delay to control servo movement speed
}
