#include "Wire.h"
#include "math.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU           16000000
#define TEST_LED        7
#define ALIGNMENT_BTN   5 

const int MPU_addr = 0x68; // адрес датчика
// массив данных
// [accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
// acc - ускорение, gyr - угловая скорость, temp - температура (raw)
//int16_t data[7];  

bool alignment_flag = 0;

volatile int flag = 0;
volatile float OCR1 = 0;
float g = 9.81;

float current_aX;
float previous_aX;
float current_aY;
float previous_aY;
float current_aZ;
float previous_aZ;

float velocityX;
float prev_velocityX;
float velocityY;
float prev_velocityY;
float velocityZ;
float prev_velocityZ;

float X;
float Y;
float Z;

float wX;
float wY;
float wZ;

float PITCH_0;
float ROLL_0;
float YAW_0;

float getAccelX(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3b);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  float acc_X_real;

  while(Wire.available()){
    int16_t acc_x = Wire.read() << 8 | Wire.read();
    acc_X_real = (float)acc_x / 32768.0 * 2.0;
  }
  return acc_X_real;
}

float getAccelY(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3D);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  float acc_Y_real;

  while(Wire.available()){
    int16_t acc_y = Wire.read() << 8 | Wire.read();
    acc_Y_real = (float)acc_y / 32768.0 * 2.0;
  }
  return acc_Y_real;
}

float getAccelZ(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  float acc_Z_real;

  while(Wire.available()){
    int16_t acc_z = Wire.read() << 8 | Wire.read();
    acc_Z_real = (float)acc_z / 32768.0 * 2.0;
  }
  return acc_Z_real;
}

float getGyroX(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  float gyro_X_real;

  while(Wire.available()){
    int16_t gyro_x = Wire.read() << 8 | Wire.read();
    gyro_X_real = (float)gyro_x / 32768.0 * 250.0;
  }
  return gyro_X_real;
}

float getGyroY(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x45);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  float gyro_Y_real;

  while(Wire.available()){
    int16_t gyro_y = Wire.read() << 8 | Wire.read();
    gyro_Y_real = (float)gyro_y / 32768.0 * 250.0;
  }
  return gyro_Y_real;
}

float getGyroZ(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x47);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  float gyro_Z_real;

  while(Wire.available()){
    int16_t gyro_z = Wire.read() << 8 | Wire.read();
    gyro_Z_real = (float)gyro_z / 32768.0 * 250.0; 
  }
  return gyro_Z_real;
}

float getX(){
  previous_aX = current_aX;
  current_aX = getAccelX();
  prev_velocityX = velocityX;
  velocityX = velocityX + (current_aX + previous_aX) * 0.01;
  X = X + (velocityX + prev_velocityX) * 0.01;
  return X;
}

float getY(){
  previous_aY = current_aY;
  current_aY = getAccelY();
  prev_velocityY = velocityY;
  velocityY = velocityY + (current_aY + previous_aY) * 0.01;
  Y = Y + (velocityY + prev_velocityY) * 0.01;
  return Y;
}

float getZ(){
  previous_aZ = current_aZ;
  current_aZ = getAccelZ();
  prev_velocityZ = velocityZ;
  velocityZ = velocityZ + (current_aZ + previous_aZ) * 0.01;
  Z = Z + (velocityZ + prev_velocityZ) * 0.01;
  return Z;
}

float getPITCH(){
  PITCH_0 = -getAccelX() / g;
  return PITCH_0;
}

float getROLL(){
  ROLL_0 = -getAccelY() / g;
  return ROLL_0;
}

float getYAW(){
  YAW_0 = -atan(getGyroX() / getGyroY());
  return YAW_0;
}

void setup() {
  Serial.begin(9600); 
  
  InitTimer(0.01);
  InitMPU();

  pinMode(TEST_LED, OUTPUT);
  pinMode(ALIGNMENT_BTN, INPUT);
  digitalWrite(ALIGNMENT_BTN, LOW);

}

void loop() {

  if(digitalRead(ALIGNMENT_BTN) == HIGH && alignment_flag == 0){
    alignment_flag = true;
    PITCH_0 = getPITCH();
    ROLL_0 = getROLL();
    YAW_0 = getYAW();
    Serial.print("PITCH_0 = "); Serial.print(PITCH_0, 6); Serial.print("    ROLL_0 = "); Serial.print(ROLL_0, 6); Serial.print("    YAW_0 = "); Serial.println(YAW_0, 6);
  }

  if(flag == 1 && alignment_flag == 1){

    //poll Accelerometr
    X = getX();
    Y = getY();
    Z = getZ();
    Serial.print("X = "); Serial.print(X); Serial.print("    Y = "); Serial.print(Y); Serial.print("    Z = "); Serial.println(Z);
  
    //poll Gyro 
    wX = getGyroX();
    wY = getGyroY();
    wZ = getGyroZ(); 

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
    digitalWrite(TEST_LED, digitalRead(TEST_LED) ^ 1);
  }
  if(flag == 20){
    flag = 0;
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
  return OCR1;
}

