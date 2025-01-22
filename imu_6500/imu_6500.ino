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

void getAccelX(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3b);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_x = Wire.read() << 8 | Wire.read();
    Serial.print("aX = "); Serial.println(acc_x, DEC);
  }
}

void getAccelY(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3D);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_y = Wire.read() << 8 | Wire.read();
    Serial.print("aY = "); Serial.println(acc_y, DEC);
  }
}

void getAccelZ(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_z = Wire.read() << 8 | Wire.read();
    Serial.print("aZ = "); Serial.println(acc_z, DEC);
  }
}

void getGyroX(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t gyro_x = Wire.read() << 8 | Wire.read();
    Serial.print("gX = "); Serial.println(gyro_x);
  }
}

void getGyroY(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x45);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t gyro_y = Wire.read() << 8 | Wire.read();
    Serial.print("gY = "); Serial.println(gyro_y);
  }
}

void getGyroZ(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x47);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t gyro_z = Wire.read() << 8 | Wire.read();
    Serial.print("gZ = "); Serial.println(gyro_z); 
  }
}

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
  //poll Accelerometr
  getAccelX();
  getAccelY();
  getAccelZ();

  //poll Gyro 
  getGyroX();
  getGyroY();
  getGyroZ();
  

  delay(100);
}

