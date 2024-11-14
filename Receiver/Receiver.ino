#include <SPI.h>
#include <LoRa.h>

#define ss 5    // LoRa SS pin
#define rst 2   // LoRa reset pin
#define dio0 4  // LoRa DIO0 pin

// Variables to store parsed values
int leftStickX = 0;
int leftStickY = 0;
int leftTrigger = 0;
int rightTrigger = 0;
bool XButton = false;
bool OButton = false;
bool SButton = false;
bool TButton = false;
bool UP = false;
bool DOWN = false;
bool LEFT = false;
bool RIGHT = false;

int batteryLevel = 100;                // Store battery level
unsigned long lastBatterySendTime = 0; // Timer for battery level transmission

void setup() {
  Serial.begin(115200);                      // Initialize Serial for debugging
  Serial2.begin(19200, SERIAL_8N1, 16, 17); // Initialize Serial2 on RX2 = GPIO 16, TX2 = GPIO 17

  Serial.println("LoRa Receiver");

  // Initialize LoRa
  LoRa.setPins(ss, rst, dio0); // Set LoRa pins
  if (!LoRa.begin(433E6)) {    // Start LoRa at 433 MHz
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}

void loop() {
  // Task 1: Check for LoRa packet and parse commands
  receiveAndParseLoRa();

  // Task 3: Send the UP, DOWN states and steering angle over Serial2
  sendCommandsOverSerial();
}

// Function to check for LoRa packet and parse data
void receiveAndParseLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedMessage = "";

    // Read packet and store in a string
    while (LoRa.available()) {
      receivedMessage += (char)LoRa.read();
    }

    // Check if message contains battery level data
    if (receivedMessage.startsWith("BL,")) {
      batteryLevel = receivedMessage.substring(3).toInt();
      Serial.print("Battery Level: ");
      Serial.println(batteryLevel);
    } else {
      // Parse the received message to extract control values
      parseReceivedMessage(receivedMessage);
    }
  }
}

// Function to parse received message and extract values
void parseReceivedMessage(String message) {
  // Split the message by commas
  int values[12]; // There are 12 values to parse
  int index = 0;
  int startIndex = 0;
  int commaIndex = 0;

  while (index < 12) {
    commaIndex = message.indexOf(',', startIndex);
    if (commaIndex == -1) {
      // If no more commas and index is less than 11, we have a problem
      if (index < 11) {
        Serial.println("Parsing error: Not enough data received.");
        return;
      } else {
        // Last value, take the rest of the string
        values[index] = message.substring(startIndex).toInt();
        break;
      }
    } else {
      values[index] = message.substring(startIndex, commaIndex).toInt();
      startIndex = commaIndex + 1;
      index++;
    }
  }

  // Assign parsed values to variables
  leftStickX = values[0];
  leftStickY = values[1];
  leftTrigger = values[2];
  rightTrigger = values[3];
  XButton = values[4];
  OButton = values[5];
  SButton = values[6];
  TButton = values[7];
  UP = values[8];
  DOWN = values[9];
  LEFT = values[10];
  RIGHT = values[11];

  // For debugging, print the parsed values
  Serial.printf(
      "Received LX:%d LY:%d L2:%d R2:%d X:%d O:%d S:%d T:%d UP:%d DOWN:%d LEFT:%d RIGHT:%d\n",
      leftStickX, leftStickY, leftTrigger, rightTrigger, XButton, OButton, SButton, TButton, UP,
      DOWN, LEFT, RIGHT);
}

// Function to calculate and send the UP, DOWN states and steering angle via Serial2 with checksum
void sendCommandsOverSerial() {
  // Construct the message
  String message = "UP:";
  message += UP;
  message += ",DOWN:";
  message += DOWN;
  message += ",STEER:";
  message += leftStickX;

  // Calculate checksum (sum of ASCII values of characters in message)
  int checksum = 0;
  for (int i = 0; i < message.length(); i++) {
    checksum += message[i];
  }
  checksum %= 256; // Keep it within 0-255

  // Construct the full message with start and end markers and checksum
  String fullMessage = "<" + message + ",CHECKSUM:" + String(checksum) + ">\n";

  // Send the message over Serial2
  Serial2.print(fullMessage);
}
