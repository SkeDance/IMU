#include <Wire.h>
#include "ICM_20948.h" // Include the library

#define AD0_VAL 0 // Set to 1 if the AD0 pin is high, 0 if low

ICM_20948_I2C myICM; // Create an ICM_20948_I2C object

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz

  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL); // Initialize the sensor
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  Serial.println("ICM-20948 connected!");
}

void loop() {
  if (myICM.dataReady()) { // Check if data is ready
    myICM.getAGMT(); // Get accelerometer, gyroscope, and magnetometer data

    // Print accelerometer data
    Serial.print("Accel X: ");
    Serial.print(myICM.accX());
    Serial.print(", Y: ");
    Serial.print(myICM.accY());
    Serial.print(", Z: ");
    Serial.println(myICM.accZ());

    // Print gyroscope data
    Serial.print("Gyro X: ");
    Serial.print(myICM.gyrX());
    Serial.print(", Y: ");
    Serial.print(myICM.gyrY());
    Serial.print(", Z: ");
    Serial.println(myICM.gyrZ());

    // Print magnetometer data
    Serial.print("Mag X: ");
    Serial.print(myICM.magX());
    Serial.print(", Y: ");
    Serial.print(myICM.magY());
    Serial.print(", Z: ");
    Serial.println(myICM.magZ());

    delay(100); // Adjust delay as needed
  }
}