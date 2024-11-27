
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

int pin12State = 0;

float AtoB = 3.77;
float AtoC;
float BtoC;
float x;
float y;

// Store previous coordinates
float prev_x = 1.8;
float prev_y = 0.5;

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

void connectToServer(){
  if (!client.connect(serverIP, serverPort)) {
    Serial.println("Connection to server failed!");
  } else {
    Serial.println("Connected to server!");
  }
}
void connectToWifi(){
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

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    connectToWifi();
    connectToServer();
  }
}

void loop() {
  DW1000Ranging.loop();
  checkWiFiConnection();
}




void newRange() {
  float range = DW1000Ranging.getDistantDevice()->getRange();
  uint16_t shortAddress = DW1000Ranging.getDistantDevice()->getShortAddress();

  // Apply offsets based on the device type
  if (shortAddress == 0x84) {
    AtoC = range + 0.80;  // Offset for DW1000A
  }
  if (shortAddress == 0x85) {
    BtoC = range + 1.10;  // Offset for DW1000B
  }

  // Only calculate coordinates if both distances are known
  if (AtoC > 0 && BtoC > 0) {
    calculateCoordinate(AtoC, BtoC);  // Calculate x and y

    // Check if new coordinates are within threshold
    if ((abs(x - prev_x) <= 0.8) && (abs(y - prev_y) <= 0.8)) {
      // Update previous coordinates if valid
      prev_x = x;
      prev_y = y;

      // Prepare a string message with the calculated coordinates
      String message = "Coordinates: x = " + String(x, 2) + ", y = " + String(y, 2) + "\n";

      // Send the coordinates to the server over Wi-Fi
      if (client.connected()) {
        client.print(message);
      } else {
        Serial.println("Disconnected from server, unable to send data.");
      }
    } else {
      // If new coordinates exceed threshold, adjust previous coordinates
      prev_x += 0.2;
      prev_y += 0.2;
      Serial.print("Ignoring jump in coordinates: new x = ");
      Serial.print(x);
      Serial.print(", new y = ");
      Serial.println(y);
    }

    if ((x >= 1.4 && x<=2.5)&& (y>=3.4 && y<=4.0)) {

      Serial.println("turn_180");
      digitalWrite(12, HIGH);
      delay(10);
    }
    digitalWrite(12,LOW);
  }
}

void newDevice(DW1000Device* device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
