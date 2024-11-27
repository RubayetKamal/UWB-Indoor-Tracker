
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <WiFi.h>


// Wi-Fi credentials
const char* ssid = "Labor50";           // SSID (Wi-Fi network name)
const char* password = "?7h.82.y}]Pi";  // Password

// Server IP and port (PC IP address where you will receive the data)
const char* serverIP = "172.31.12.102";  // Replace with your PC's IP address
const int serverPort = 8080;             // Port number to send data

WiFiClient client;

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// Connection pins
const uint8_t PIN_RST = 27;  // Reset pin
const uint8_t PIN_IRQ = 34;  // IRQ pin
const uint8_t PIN_SS = 4;    // SPI select pin

char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

float AtoB = 3.77;
float AtoC;
float BtoC;
float x;
float y;

// Store previous coordinates
float prev_x = 1.8;
float prev_y = 0.5;

float coordinateThreshold = 0.8;
float coordinateAdjustment = 0.2;

float lowerXcoordinate = 1.8;
float upperXcoordinate = 2.5;
float lowerYcoordinate = 3.4;

int arduinoPin = 12;


void calculateCoordinate(float AtoC, float BtoC) {
  float cosAngleCAB = (((AtoC * AtoC) + (AtoB * AtoB) - (BtoC * BtoC)) / (2 * AtoC * AtoB));

  // Check if cosAngleCAB is within valid range
  if (cosAngleCAB < -1.0 || cosAngleCAB > 1.0) {
    cosAngleCAB = constrain(cosAngleCAB, -1.0, 1.0);  // Constrain to valid range
  }

  float angleCAB = acos(cosAngleCAB);

  // Calculate coordinates
  x = AtoC * cos(angleCAB);
  y = AtoC * sin(angleCAB);

  // Debug output to see calculated coordinates
  Serial.print("Calculated Coordinates: x = ");
  Serial.print(x);
  Serial.print(", y = ");
  Serial.println(y);
}

void connectToServer() {
  if (!client.connect(serverIP, serverPort)) {
    Serial.println("Connection to server failed!");
  } else {
    Serial.println("Connected to server!");
  }
}
void connectToWifi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void setup() {
  // For Serial Monitor
  pinMode(arduinoPin, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 is running!");
  // Initialize the SPI and UWB

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);  // Reset, CS, IRQ pin
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // Start as tag
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  connectToWifi();
  connectToServer();
}


void loop() {
    if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected, reconnecting...");
  connectToWifi();
  connectToServer();
  }
  DW1000Ranging.loop();
}


void updateRangeAndOffset() {
  float range = DW1000Ranging.getDistantDevice()->getRange();
  uint16_t shortAddress = DW1000Ranging.getDistantDevice()->getShortAddress();

  switch (shortAddress) {
    case 0x84:
      AtoC = applyOffset(range, 0.80);  // Offset for DW1000A
      break;
    case 0x85:
      BtoC = applyOffset(range, 1.10);  // Offset for DW1000B
      break;
    default:
      // Handle unknown devices or no offset
      break;
  }
}

float applyOffset(float range, float offset) {
  return range + offset;
}

void attemptCoordinateUpdate() {
  // Only calculate coordinates if both distances are known
  if (AtoC > 0 && BtoC > 0) {
    if (calculateAndValidateCoordinates()) {
      sendCoordinates();
    }
  }
}

bool calculateAndValidateCoordinates() {
  calculateCoordinate(AtoC, BtoC);  // This function modifies global vars x and y

  // Check if new coordinates are within threshold
  if ((abs(x - prev_x) <= coordinateThreshold) && (abs(y - prev_y) <= coordinateThreshold)) {
    prev_x = x;
    prev_y = y;
    return true;  // Valid update
  } else {
    adjustForBigJump();
    return false;  // Coordinate change too big, ignoring
  }
}

void sendCoordinates() {
  String message = prepareCoordinateMessage(x, y);
  if (client.connected()) {
    client.print(message);
  } else {
    Serial.println("Disconnected from server, unable to send data.");
  }

  checkAndPerformActionBasedOnCoordinates(x, y);  // Additional checks on coordinates
}

String prepareCoordinateMessage(float xCoord, float yCoord) {
  return "Coordinates: x = " + String(xCoord, 2) + ", y = " + String(yCoord, 2) + "\n";
}

void adjustForBigJump() {
  prev_x += coordinateAdjustment;
  prev_y += coordinateAdjustment;
  Serial.println("Ignoring jump in coordinates.");
}

void checkAndPerformActionBasedOnCoordinates(float xCoord, float yCoord) {
  if ((xCoord >= lowerXcoordinate && xCoord <= upperXcoordinate) && (yCoord >= lowerYcoordinate)) {
    performRequiredAction();  // For instance, turn 180 degrees
  }
  digitalWrite(arduinoPin, LOW);  // Ensure to turn it off after the action
}

void performRequiredAction() {
  Serial.println("turn_180");
  digitalWrite(arduinoPin, HIGH);
  delay(10);
}

void newRange() {
  updateRangeAndOffset();     // Get range and apply offsets
  attemptCoordinateUpdate();  // Calculate, validate, and possibly send coords
}


void newDevice(DW1000Device* device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
