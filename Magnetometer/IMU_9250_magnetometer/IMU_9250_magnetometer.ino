#include <Wire.h>

// AK8963 I2C address
#define AK8963_ADDRESS 0x0C

// AK8963 register addresses
#define AK8963_WHO_AM_I 0x00
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_HXH 0x04
#define AK8963_HYL 0x05
#define AK8963_HYH 0x06
#define AK8963_HZL 0x07
#define AK8963_HZH 0x08
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

// Sensitivity adjustment values
float asaX, asaY, asaZ;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize AK8963
    if (!initAK8963()) {
        Serial.println("AK8963 initialization failed!");
        while (1);
    }
    Serial.println("AK8963 initialized successfully!");
}

void loop() {
    // Read magnetometer data
    int16_t mx, my, mz;
    if (readAK8963Data(&mx, &my, &mz)) {
        // Apply sensitivity adjustment
        float fx = mx * ((((asaX - 128) * 0.5) / 128.0) + 1.0);
        float fy = my * ((((asaY - 128) * 0.5) / 128.0) + 1.0);
        float fz = mz * ((((asaZ - 128) * 0.5) / 128.0) + 1.0);

        // Print magnetometer data
        Serial.print("Magnetometer: ");
        Serial.print(fx); Serial.print(", ");
        Serial.print(fy); Serial.print(", ");
        Serial.println(fz);
    } else {
        Serial.println("Failed to read magnetometer data!");
    }

    delay(100); // Adjust delay as needed
}

// Initialize AK8963
bool initAK8963() {
    // Reset AK8963
    writeAK8963Register(AK8963_CNTL2, 0x01);

    // Set AK8963 to Fuse ROM access mode
    writeAK8963Register(AK8963_CNTL1, 0x0F);
    delay(100);

    // Read sensitivity adjustment values
    asaX = readAK8963Register(AK8963_ASAX);
    asaY = readAK8963Register(AK8963_ASAY);
    asaZ = readAK8963Register(AK8963_ASAZ);

    // Set AK8963 to continuous measurement mode 2 (100 Hz)
    writeAK8963Register(AK8963_CNTL1, 0x16);

    return true;
}

// Read magnetometer data
bool readAK8963Data(int16_t* mx, int16_t* my, int16_t* mz) {
    // Check if data is ready
    if (readAK8963Register(AK8963_ST1) & 0x01) {
        uint8_t buffer[7];
        readAK8963Registers(AK8963_HXL, 6, buffer);

        // Combine high and low bytes
        *mx = (int16_t)(buffer[1] << 8 | buffer[0]);
        *my = (int16_t)(buffer[3] << 8 | buffer[2]);
        *mz = (int16_t)(buffer[5] << 8 | buffer[4]);

        // Check for magnetic sensor overflow
        if (buffer[6] & 0x08) {
            return false;
        }

        return true;
    }
    return false;
}

// Write to AK8963 register
void writeAK8963Register(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Read from AK8963 register
uint8_t readAK8963Register(uint8_t reg) {
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8963_ADDRESS, 1);
    return Wire.read();
}

// Read multiple AK8963 registers
void readAK8963Registers(uint8_t reg, uint8_t count, uint8_t* data) {
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AK8963_ADDRESS, (uint8_t)count);
    for (uint8_t i = 0; i < count; i++) {
        data[i] = Wire.read();
    }
}