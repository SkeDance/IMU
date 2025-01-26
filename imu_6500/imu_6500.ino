#include "Wire.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000

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

volatile int flag = 0;
volatile float OCR1 = 0;

void getAccelX(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3b);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_x = Wire.read() << 8 | Wire.read();
    float acc_X_real = (float)acc_x / 32768.0 * 2.0;
    Serial.print("aX = "); Serial.println(acc_X_real, 6);
  }
}

void getAccelY(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3D);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_y = Wire.read() << 8 | Wire.read();
    float acc_Y_real = (float)acc_y / 32768.0 * 2.0;
    Serial.print("aY = "); Serial.println(acc_Y_real, 6);
  }
}

void getAccelZ(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t acc_z = Wire.read() << 8 | Wire.read();
    float acc_Z_real = (float)acc_z / 32768.0 * 2.0;
    Serial.print("aZ = "); Serial.println(acc_Z_real, 6);
  }
}

void getGyroX(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t gyro_x = Wire.read() << 8 | Wire.read();
    float gyro_X_real = (float)gyro_x / 32768.0 * 250.0;
    Serial.print("gX = "); Serial.println(gyro_X_real, 6);
  }
}

void getGyroY(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x45);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t gyro_y = Wire.read() << 8 | Wire.read();
    float gyro_Y_real = (float)gyro_y / 32768.0 * 250.0;
    Serial.print("gY = "); Serial.println(gyro_Y_real, 6);
  }
}

void getGyroZ(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x47);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    int16_t gyro_z = Wire.read() << 8 | Wire.read();
    float gyro_Z_real = (float)gyro_z / 32768.0 * 250.0;
    Serial.print("gZ = "); Serial.println(gyro_Z_real, 6); 
  }
}

void setup() {
  Serial.begin(9600); 
  
  InitTimer(0.1);
  InitMPU();

  pinMode(7, OUTPUT);

}
void loop() {
  
  if(flag == 1){
    
  //poll Accelerometr
  getAccelX();
  getAccelY();
  getAccelZ();

  //poll Gyro 
  getGyroX();
  getGyroY();
  getGyroZ();

  flag = 0;
  }
}

void InitTimer(float time){
  cli();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12); // Compare mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // 1024 prescaler
  OCR1A = Timer1ClockSetup(time);
  TIMSK1 |= (1 << OCIE1A); // enable compare interrupts
  sei();
}

ISR(TIMER1_COMPA_vect){
  flag++;
  if(flag == 1){
    //led for interrupt test
    digitalWrite(7, digitalRead(7) ^ 1);
  }
}

void InitMPU(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

float Timer1ClockSetup(float time){
  float resolution = (1.0 / (F_CPU / 1024.0));
  OCR1 = time / resolution;
  Serial.println(OCR1);
  return OCR1;
}

