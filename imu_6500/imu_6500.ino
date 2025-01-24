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

  InitTimer();
  InitMPU();

  pinMode(7, OUTPUT);

  Serial.begin(9600); 
}
void loop() {
  //float test = 0.1 / 0.000064;
  // Serial.println(OCR1);

  if(flag == 6){
    
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

void InitTimer(){
  cli();
  //TCCR1A = 0;
  TCCR1B = (1 << WGM12); // Compare mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // 1024 prescaler
  OCR1A = 65000;
  // OCR1AH = 0b11101010; //0b00111101 - 15k; //0b11101010 - 60k
  // OCR1AL = 0b01100000; //0b00001000 - 15k; //0b01100000 - 60k
  //OCR1A = 0x61B;//Timer1ClockSetup(time);
  TIMSK1 |= (1 << OCIE1A); // enable compare interrupts
  sei();
}

ISR(TIMER1_COMPA_vect){
  flag++;
  //digitalWrite(7, digitalRead(7) ^ 1);
  if(flag == 6){
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

// float Timer1ClockSetup(float time){
//   float resolution = (1 / (F_CPU / 1024));
//   OCR1 = 1563;//(time / resolution); //(1 / (F_CPU / 1024));
//   Serial.println(OCR1);
//   return OCR1;
//   //set ocr1a to 15625
// }

