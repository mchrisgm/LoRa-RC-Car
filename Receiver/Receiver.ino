#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // I2C address for most 128x32 or 128x64 OLEDs

// Initialize the OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// LoRa module pins
#define ss 5
#define rst 2
#define dio0 4

// Variables to store parsed values
int leftStickX = 0;
int leftStickY = 0;
int leftTrigger = 0;
int rightTrigger = 0;
bool XButton = false;
bool OButton = false;
bool SButton = false;
bool TButton = false;

void setup() {
  Serial.begin(9600);

  // Initialize I2C with custom pins for the OLED display
  Wire.begin(21, 22);  // SDA = GPIO21, SCL = GPIO22 (ESP32 default I2C pins)

  // Initialize OLED display with I2C address and error check
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Stop execution if display fails to initialize
  }

  display.display(); // Display the Adafruit logo on the screen for 2 seconds
  delay(2000);
  display.clearDisplay(); // Clear the display

  Serial.println("LoRa Receiver");

  // Initialize LoRa
  LoRa.setPins(ss, rst, dio0); // Set LoRa pins
  if (!LoRa.begin(433E6)) {    // Start LoRa at 433 MHz
    Serial.println("Starting LoRa failed!");
    for (;;); // Stop execution if LoRa fails to initialize
  }

  display.display();
  delay(2000);
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedMessage = "";

    // Read packet and store in a string
    while (LoRa.available()) {
      receivedMessage += (char)LoRa.read();
    }

    // Get RSSI (signal strength)
    int rssi = LoRa.packetRssi();

    // Parse the received message to extract values
    parseReceivedMessage(receivedMessage);

    // Map the X and Y values to screen dimensions (0-100 to 0-128 and 0-64)
    int mappedX = map(leftStickX, 0, 100, 0, SCREEN_HEIGHT - 23);
    int mappedY = map(leftStickY, 0, 100, SCREEN_HEIGHT - 23, 0);

    int mappedTriggerL = map(leftTrigger, 0, 255, 0, SCREEN_HEIGHT - 10);
    int mappedTriggerR = map(rightTrigger, 0, 255, 0, SCREEN_HEIGHT - 10);

    // Print received values to Serial Monitor for debugging
    Serial.print("Received LX: ");
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
    Serial.print(" RSSI: ");
    Serial.println(rssi);

    // Update OLED display
    display.clearDisplay();

    // Joystick position graphic
    display.drawCircle((SCREEN_HEIGHT / 2) + 5, (SCREEN_HEIGHT / 2) + 5, (SCREEN_HEIGHT / 2) - 6, SSD1306_WHITE);
    display.fillCircle(13 + mappedX, 13 + mappedY, 2, SSD1306_WHITE);

    display.drawRect(SCREEN_HEIGHT + 10, 10, 10, SCREEN_HEIGHT - 10, SSD1306_WHITE);
    display.drawRect(SCREEN_HEIGHT + 25, 10, 10, SCREEN_HEIGHT - 10, SSD1306_WHITE);

    int leftTop =  10 + (SCREEN_HEIGHT - 10) - mappedTriggerL;
    int rightTop = 10 + (SCREEN_HEIGHT - 10) - mappedTriggerR;

    display.fillRect(SCREEN_HEIGHT + 10, leftTop, 10, mappedTriggerL, SSD1306_WHITE);
    display.fillRect(SCREEN_HEIGHT + 25, rightTop, 10, mappedTriggerR, SSD1306_WHITE);

    display.display(); // Show content on OLED
  }
}

// Function to parse received message and extract values
void parseReceivedMessage(String message) {
  int xIndex = message.indexOf("LX,");
  int yIndex = message.indexOf(",LY,");
  int l2Index = message.indexOf(",L2,");
  int r2Index = message.indexOf(",R2,");
  int xbIndex = message.indexOf(",X,");
  int obIndex = message.indexOf(",O,");
  int sbIndex = message.indexOf(",S,");
  int tbIndex = message.indexOf(",T,");

  if (xIndex != -1 && yIndex != -1 && l2Index != -1 && r2Index != -1 &&
      xbIndex != -1 && obIndex != -1 && sbIndex != -1 && tbIndex != -1) {
    // Parse values from the message
    leftStickX = message.substring(xIndex + 3, yIndex).toInt();  // Changed +2 to +3 for "LX,"
    leftStickY = message.substring(yIndex + 4, l2Index).toInt(); // Changed +3 to +4 for ",LY,"
    leftTrigger = message.substring(l2Index + 4, r2Index).toInt();
    rightTrigger = message.substring(r2Index + 4, xbIndex).toInt();
    XButton = message.substring(xbIndex + 3, obIndex).toInt();
    OButton = message.substring(obIndex + 3, sbIndex).toInt();
    SButton = message.substring(sbIndex + 3, tbIndex).toInt();
    TButton = message.substring(tbIndex + 3).toInt();
  } else {
    Serial.println("Parsing error: Could not find all indices in message.");
  }
}
