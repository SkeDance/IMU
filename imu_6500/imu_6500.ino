#include "Wire.h"
const int MPU_addr = 0x68; // адрес датчика
// массив данных
// [accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
// acc - ускорение, gyr - угловая скорость, temp - температура (raw)
int16_t data[7];  
int16_t acc_X_data;
int16_t acc_Y_data;
int16_t acc_Z_data;
int16_t gyro_X_data;
int16_t gyro_Y_data;
int16_t gyro_Z_data;

void setup() {
  // инициализация
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.begin(9600);
}
void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3b);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_x = Wire.read() << 8 | Wire.read();
    Serial.println(acc_x, DEC);
  }

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3D);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_y = Wire.read() << 8 | Wire.read();
    Serial.println(acc_y, DEC);
  }

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_z = Wire.read() << 8 | Wire.read();
    Serial.println(acc_z, DEC);
  }

  // Wire.requestFrom(MPU_addr, 2);

  // while(Wire.available()){
  //   int16_t acc_y = Wire.read() << 8 | Wire.read();
  //   Serial.println(acc_y, DEC);
  // }

  delay(500);
  // void get_ACC_X();  // получаем
  // // выводим 
  // Serial.print(acc_X_data);
  // Serial.println();
  // delay(200);
  // Wire.endTransmission();
}

// void get_ACC_X(){
//   Wire.beginTransmission(MPU_addr);
//   Wire.write(0x3C);
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU_addr, 1, true);
//   acc_X_data = Wire.read();
// }
