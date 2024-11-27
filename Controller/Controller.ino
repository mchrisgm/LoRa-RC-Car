#include <PS4Controller.h>
#include <SPI.h>
#include <LoRa.h>

#define ss 5        // LoRa SS pin
#define rst 2       // LoRa reset pin
#define dio0 4      // LoRa DIO0 pin

int batteryLevel = 100;  // Initialize battery level variable

void setup() {
  Serial.begin(115200);
  PS4.begin();  // Initialize PS4 controller

  Serial.println("LoRa Sender");

  // Set LoRa module pins
  LoRa.setPins(ss, rst, dio0);

  // Initialize LoRa at 433 MHz
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  if (PS4.isConnected()) {
    sendControllerData();  // Send controller data over LoRa
    delay(5);             // Adjust delay as needed for a faster update rate
  }
}

// Function to transmit controller data over LoRa and handle acknowledgment
void sendControllerData() {
  // Read and map Left Stick X and Y values from -127 to 127 range to 0 to 100
  int leftStickX = map(PS4.LStickX(), -127, 127, 0, 100);
  int leftStickY = map(PS4.LStickY(), -127, 127, 0, 100);

  // Map trigger values from 0 to 255 range to 0 to 100
  int leftTrigger = map(PS4.L2Value(), 0, 255, 0, 100);
  int rightTrigger = map(PS4.R2Value(), 0, 255, 0, 100);

  // Read button states
  bool XButton = PS4.Cross();
  bool OButton = PS4.Circle();
  bool SButton = PS4.Square();
  bool TButton = PS4.Triangle();
  bool up = PS4.Up();
  bool down = PS4.Down();
  bool left = PS4.Left();
  bool right = PS4.Right();

  // Send all values in a comma-separated format over LoRa
  LoRa.beginPacket();
  LoRa.print(leftStickX); LoRa.print(',');
  LoRa.print(leftStickY); LoRa.print(',');
  LoRa.print(leftTrigger); LoRa.print(',');
  LoRa.print(rightTrigger); LoRa.print(',');
  LoRa.print(XButton); LoRa.print(',');
  LoRa.print(OButton); LoRa.print(',');
  LoRa.print(SButton); LoRa.print(',');
  LoRa.print(TButton); LoRa.print(',');
  LoRa.print(up); LoRa.print(',');
  LoRa.print(down); LoRa.print(',');
  LoRa.print(left); LoRa.print(',');
  LoRa.print(right);
  LoRa.endPacket();

  // Debugging: Print all values to Serial Monitor
  Serial.printf("Sent LX:%d LY:%d L2:%d R2:%d X:%d O:%d S:%d T:%d UP:%d DOWN:%d LEFT:%d RIGHT:%d Battery:%d%%\n",
                leftStickX, leftStickY, leftTrigger, rightTrigger,
                XButton, OButton, SButton, TButton, up, down, left, right, batteryLevel);

  // Wait for acknowledgment from receiver
  unsigned long ackTimeout = 100;     // 50ms timeout
  unsigned long startTime = millis(); // Record the start time
  bool ackReceived = false;          // Flag to check if acknowledgment is received
  String ackMessage = "";            // String to store the acknowledgment message

  // Loop until timeout or acknowledgment is received
  while (millis() - startTime < ackTimeout) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // Read the acknowledgment packet
      while (LoRa.available()) {
        ackMessage += (char)LoRa.read();
      }
      ackReceived = true;
      break;  // Exit the loop if acknowledgment is received
    }
  }

  if (ackReceived) {
    // Process the acknowledgment message
    if (ackMessage.startsWith("ACK")) {
      int batteryIndex = ackMessage.indexOf("BAT:");
      if (batteryIndex != -1) {
        // Extract battery level from the acknowledgment message
        String batteryStr = ackMessage.substring(batteryIndex + 4);
        batteryLevel = batteryStr.toInt();
        Serial.print("Acknowledgment received. Battery Level: ");
        Serial.println(batteryLevel);
      } else {
        Serial.println("Acknowledgment received, but no battery level found.");
      }
    } else {
      Serial.println("Received unexpected message: " + ackMessage);
    }
  } else {
    // Serial.println("No acknowledgment received within 50ms.");
  }
}
