#include <PS4Controller.h>
#include <SPI.h>
#include <LoRa.h>

#define ss 5        // LoRa SS pin
#define rst 2       // LoRa reset pin
#define dio0 4      // LoRa DIO0 pin

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
    // Read Left Stick X and Y values
    int leftStickX = PS4.LStickX();    // Left Stick X (horizontal)
    int leftStickY = PS4.LStickY();    // Left Stick Y (vertical)

    // Read Trigger values
    int leftTrigger = PS4.L2Value();   // Left trigger (L2)
    int rightTrigger = PS4.R2Value();  // Right trigger (R2)

    // Read button states
    bool XButton = PS4.Cross();
    bool OButton = PS4.Circle();
    bool SButton = PS4.Square();
    bool TButton = PS4.Triangle();

    // Map the stick values from -127 to 127 range to 0 to 100
    leftStickX = map(leftStickX, -127, 127, 0, 100);
    leftStickY = map(leftStickY, -127, 127, 0, 100);

    // Send all values in a comma-separated format
    LoRa.beginPacket();
    LoRa.print("LX,");
    LoRa.print(leftStickX);
    LoRa.print(",LY,");
    LoRa.print(leftStickY);
    LoRa.print(",L2,");
    LoRa.print(leftTrigger);
    LoRa.print(",R2,");
    LoRa.print(rightTrigger);
    LoRa.print(",X,");
    LoRa.print(XButton);
    LoRa.print(",O,");
    LoRa.print(OButton);
    LoRa.print(",S,");
    LoRa.print(SButton);
    LoRa.print(",T,");
    LoRa.print(TButton);
    LoRa.endPacket();

    // Print values to Serial Monitor for debugging
    Serial.print("Sent LX: ");
    Serial.print(leftStickX);
    Serial.print(" LY: ");
    Serial.print(leftStickY);
    Serial.print(" L2: ");
    Serial.print(leftTrigger);
    Serial.print(" R2: ");
    Serial.print(rightTrigger);
    Serial.print(" X: ");
    Serial.print(XButton);
    Serial.print(" O: ");
    Serial.print(OButton);
    Serial.print(" S: ");
    Serial.print(SButton);
    Serial.print(" T: ");
    Serial.println(TButton);

    delay(50);  // Adjust delay as needed
  }
}
