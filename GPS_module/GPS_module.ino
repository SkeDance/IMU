#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Define pins for SoftwareSerial
const int RX_Pin = 5;
const int TX_Pin = 4;

const int GPS_Baud_Rate = 9600;

// Instantiate TinyGPS++ object
TinyGPSPlus gpsModule;

// Create software serial port named "gpsSerialPort"
SoftwareSerial gpsSerialPort(RX_Pin, TX_Pin);

void setup()
{
  // Initialize the hardware serial communication at 9600 baud
  Serial.begin(9600);

  // Initialize the software serial port at the GPS module's default baud rate
  gpsSerialPort.begin(GPS_Baud_Rate);
}

void loop()
{
  // Process incoming GPS data
  while (gpsSerialPort.available() > 0)
    if (gpsModule.encode(gpsSerialPort.read()))
      displayGPSData();

  // Check if no GPS data has been received for 5 seconds
  if (millis() > 5000 && gpsModule.charsProcessed() < 10)
  {
    Serial.println("Error: GPS module not detected");
    while (true);
  }
}

void displayGPSData()
{
  if (gpsModule.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gpsModule.location.lat(), 6); //print latitude
    Serial.print("Longitude: ");
    Serial.println(gpsModule.location.lng(), 6); //print longitude
    Serial.print("Altitude: ");
    Serial.println(gpsModule.altitude.meters()); //print altitude
  }
  else
  {
    Serial.println("Location: Unavailable");
  }
  
  Serial.print("Current Date: ");
  if (gpsModule.date.isValid())
  {
    Serial.print(gpsModule.date.month());
    Serial.print("/");
    Serial.print(gpsModule.date.day());
    Serial.print("/");
    Serial.println(gpsModule.date.year());
  }
  else
  {
    Serial.println("Date Unavailable");
  }

  Serial.print("Current Time: ");
  if (gpsModule.time.isValid())
  {
    if (gpsModule.time.hour() < 10) Serial.print("0");
    Serial.print(gpsModule.time.hour());
    Serial.print(":");
    if (gpsModule.time.minute() < 10) Serial.print("0");
    Serial.print(gpsModule.time.minute());
    Serial.print(":");
    if (gpsModule.time.second() < 10) Serial.print("0");
    Serial.print(gpsModule.time.second());
    Serial.print(".");
    if (gpsModule.time.centisecond() < 10) Serial.print("0");
    Serial.println(gpsModule.time.centisecond());
  }
  else
  {
    Serial.println("Unavailable");
  }

  Serial.println();
  delay(1000);  // Delay before the next data display
}