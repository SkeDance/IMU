#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
//https://github.com/adafruit/Adafruit_BNO08x/blob/master/examples/quaternion_yaw_pitch_roll/quaternion_yaw_pitch_roll.ino
//_____________________________________________________________________________________________________________________________
//#include <Adafruit_BNO08xM.h>
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address
//EL SEGON ES 0x4B
#define BNO08x_I2CADDR_DEFAULT2 0x4B

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT                                                       \
  9 ///< The number of current options for the activity classifier

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the BNO08x 9-DOF Orientation IMU Fusion Breakout
 */
class Adafruit_BNO08x {
public:
  Adafruit_BNO08x(int8_t reset_pin = -1);
  ~Adafruit_BNO08x();

  bool begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensor_id = 0);
  bool begin_UART(HardwareSerial *serial, int32_t sensor_id = 0);

  bool begin_SPI(uint8_t cs_pin, uint8_t int_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id = 0);

  void hardwareReset(void);
  bool wasReset(void);

  bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
  bool getSensorEvent(sh2_SensorValue_t *value);

  sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

protected:
  virtual bool _init(int32_t sensor_id);

  sh2_Hal_t
      _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
};
class Adafruit_BNO08x_2 {
public:
  Adafruit_BNO08x_2(int8_t reset_pin = -1);
  ~Adafruit_BNO08x_2();

  bool begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT2,
                 TwoWire *wire = &Wire, int32_t sensor_id2 = 2);
  bool begin_UART(HardwareSerial *serial, int32_t sensor_id2 = 2);

  bool begin_SPI(uint8_t cs_pin, uint8_t int_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id2 = 2);

  void hardwareReset(void);
  bool wasReset(void);

  bool enableReport(sh2_SensorId_t sensor2, uint32_t interval_us = 10000);
  bool getSensorEvent(sh2_SensorValue_t *value2);

  sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

protected:
  virtual bool _init(int32_t sensor_id);

  sh2_Hal_t
      _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
};

//_______________________________________________________________________________________________________________________________
//per checkejar la llibreria https://github.com/adafruit/Adafruit_BNO08x/blob/master/src/Adafruit_BNO08x.h

// For SPI mode, we need a CS pin
// EL FEM SERVIR??SÍ QUE CAL. Però jo realment  estic  possant el SDA i l'SCL a A4 i A5 de la Nano RP2040, respectivament.
#define BNO08X_CS 10
#define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

struct euler2_t {
  float yaw2;
  float pitch2;
  float roll2;
} ypr2;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorValue_t sensorValue2;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEuler2(float qr2, float qi2, float qj2, float qk2, euler2_t* ypr2, bool degrees2 = false) {

    float sqr2 = sq(qr2);
    float sqi2 = sq(qi2);
    float sqj2 = sq(qj2);
    float sqk2 = sq(qk2);

    ypr2->yaw2 = atan2(2.0 * (qi2 * qj2 + qk2 * qr2), (sqi2 - sqj2 - sqk2 + sqr2));
    ypr2->pitch2 = asin(-2.0 * (qi2 * qk2 - qj2 * qr2) / (sqi2 + sqj2 + sqk2 + sqr2));
    ypr2->roll2 = atan2(2.0 * (qj2 * qk2 + qi2 * qr2), (-sqi2 - sqj2 + sqk2 + sqr2));

    if (degrees2) {
      ypr2->yaw2 *= RAD_TO_DEG;
      ypr2->pitch2 *= RAD_TO_DEG;
      ypr2->roll2 *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerRV2(sh2_RotationVectorWAcc_t* rotational_vector2, euler2_t* ypr2, bool degrees2 = false) {
    quaternionToEuler2(rotational_vector2->real, rotational_vector2->i, rotational_vector2->j, rotational_vector2->k, ypr2, degrees2);
}

void quaternionToEulerGI2(sh2_GyroIntegratedRV_t* rotational_vector2, euler2_t* ypr2, bool degrees2 = false) {
    quaternionToEuler2(rotational_vector2->real, rotational_vector2->i, rotational_vector2->j, rotational_vector2->k, ypr2, degrees2);
}
void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV2(&sensorValue.un.arvrStabilizedRV, &ypr2, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI2(&sensorValue.un.gyroIntegratedRV, &ypr2, true);
        break;
    }
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.print(ypr.roll);               Serial.print("\t");
    //Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr2.yaw2);                Serial.print("\t");
    Serial.print(ypr2.pitch2);              Serial.print("\t");
    Serial.println(ypr2.roll2);
  }

}

enum sh2_SensorId_e {
    SH2_RAW_ACCELEROMETER = 0x14,
    SH2_ACCELEROMETER = 0x01,
    SH2_LINEAR_ACCELERATION = 0x04,
    SH2_GRAVITY = 0x06,
    SH2_RAW_GYROSCOPE = 0x15,
    SH2_GYROSCOPE_CALIBRATED = 0x02,
    SH2_GYROSCOPE_UNCALIBRATED = 0x07,
    SH2_RAW_MAGNETOMETER = 0x16,
    SH2_MAGNETIC_FIELD_CALIBRATED = 0x03,
    SH2_MAGNETIC_FIELD_UNCALIBRATED = 0x0f,
    SH2_ROTATION_VECTOR = 0x05,
    SH2_GAME_ROTATION_VECTOR = 0x08,
    SH2_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
    SH2_PRESSURE = 0x0a,
    SH2_AMBIENT_LIGHT = 0x0b,
    SH2_HUMIDITY = 0x0c,
    SH2_PROXIMITY = 0x0d,
    SH2_TEMPERATURE = 0x0e,
    SH2_RESERVED = 0x17,
    SH2_TAP_DETECTOR = 0x10,
    SH2_STEP_DETECTOR = 0x18,
    SH2_STEP_COUNTER = 0x11,
    SH2_SIGNIFICANT_MOTION = 0x12,
    SH2_STABILITY_CLASSIFIER = 0x13,
    SH2_SHAKE_DETECTOR = 0x19,
    SH2_FLIP_DETECTOR = 0x1a,
    SH2_PICKUP_DETECTOR = 0x1b,
    SH2_STABILITY_DETECTOR = 0x1c,
    SH2_PERSONAL_ACTIVITY_CLASSIFIER = 0x1e,
    SH2_SLEEP_DETECTOR = 0x1f,
    SH2_TILT_DETECTOR = 0x20,
    SH2_POCKET_DETECTOR = 0x21,
    SH2_CIRCLE_DETECTOR = 0x22,
    SH2_HEART_RATE_MONITOR = 0x23,
    SH2_ARVR_STABILIZED_RV = 0x28,
    SH2_ARVR_STABILIZED_GRV = 0x29,
    SH2_GYRO_INTEGRATED_RV = 0x2A,
    SH2_IZRO_MOTION_REQUEST = 0x2B,

    // UPDATE to reflect greatest sensor id
    SH2_MAX_SENSOR_ID = 0x2B,
};