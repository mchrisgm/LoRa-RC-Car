#include <PS4Controller.h>
#include <SPI.h>
#include <LoRa.h>

#define ss 5        // LoRa SS pin
#define rst 2       // LoRa reset pin
#define dio0 4      // LoRa DIO0 pin

int batteryLevel = 100;  // Initialize battery level variable

void setup() {
  Serial.begin(115200);
  PS4.begin();

  Serial.println("LoRa Sender");

  // Set LoRa module pins
  LoRa.setPins(ss, rst, dio0);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  if (PS4.isConnected()) {
    sendControllerData();
    delay(30);  // Adjust delay as needed for a faster update rate
  }
}

// Function to transmit controller data over LoRa
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

  // Send all values in a comma-separated format
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

}
