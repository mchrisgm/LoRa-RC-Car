#include <SPI.h>
#include <LoRa.h>

#define ss 5        // LoRa SS pin
#define rst 2       // LoRa reset pin
#define dio0 4      // LoRa DIO0 pin

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

int batteryLevel = 100;  // Store battery level
unsigned long lastBatterySendTime = 0;  // Timer for battery level transmission

void setup() {
  Serial.begin(115200);                     // Initialize Serial for debugging
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // Initialize Serial2 on RX2 = GPIO 16, TX2 = GPIO 17

  Serial.println("LoRa Receiver");

  // Initialize LoRa
  LoRa.setPins(ss, rst, dio0);  // Set LoRa pins
  if (!LoRa.begin(433E6)) {     // Start LoRa at 433 MHz
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // Task 1: Check for LoRa packet and parse commands
  receiveAndParseLoRa();

  // Task 2: Periodically send battery level
  sendBatteryLevel();

  // Simulate battery level drain
  batteryLevel *= 0.9999;
  if (batteryLevel < 1) {
    batteryLevel = 100;
  }
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

      // Send UP and DOWN states to another ESP32 via Serial2
      sendCommandsOverSerial();
    }
  }
}

// Function to parse received message and extract values
void parseReceivedMessage(String message) {
  // Split the message by commas
  int values[12];  // There are 12 values to parse
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

// Function to send UP and DOWN states via Serial2
void sendCommandsOverSerial() {
  // Send UP and DOWN button states as a string, e.g., "UP:1,DOWN:0"
  Serial2.print("UP:");
  Serial2.print(UP);
  Serial2.print(",DOWN:");
  Serial2.println(DOWN);
}

// Function to send battery level periodically
void sendBatteryLevel() {
  unsigned long currentTime = millis();
  if (currentTime - lastBatterySendTime > 10000) {  // Send every 10 seconds
    LoRa.beginPacket();
    LoRa.print("BL,");
    LoRa.print(batteryLevel);
    LoRa.endPacket();
    lastBatterySendTime = currentTime;
  }
}
