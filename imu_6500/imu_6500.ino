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

volatile int flag = 0;
bool alignment_flag = 0;
float OCR1 = 0;
float g = 9.81;

float Acc_matrix_ENUp[3][1] = {0, 0, 0};

//shirota & dolgota
float latitude_0 = 55.44;
float longitude_0 = 37.36 ;

float R_Earth = 6.371 * (10 ^ 6);
float R_latitude = (R_Earth * (1 - square(M_E))) / pow(1 - square(M_E) * square(sin(latitude_0)), 3 / 2);
float R_longitude = (R_Earth * (1 - square(M_E))) / pow(1 - square(M_E) * square(sin(longitude_0)), 1 / 2);

int16_t acc_x;
int16_t acc_y;
int16_t acc_z;

int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;

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

//float matrix_LL[3][3];

float getAccelX(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3b);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    acc_x = Wire.read() << 8 | Wire.read();
  }
  return (float)acc_x / 32768 * 2.0;
}

float getAccelY(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3D);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    acc_y = Wire.read() << 8 | Wire.read();
  }
  return (float)acc_y / 32768.0 * 2.0;
}

float getAccelZ(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    acc_z = Wire.read() << 8 | Wire.read();
  }
  return (float)acc_z / 32768.0 * 2.0;
}

float getGyroX(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    gyro_x = Wire.read() << 8 | Wire.read();
  }
  return (float)gyro_x / 32768.0 * 250.0;
}

float getGyroY(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x45);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    gyro_y = Wire.read() << 8 | Wire.read();
  }
  return (float)gyro_y / 32768.0 * 250.0;
}

float getGyroZ(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x47);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    gyro_z = Wire.read() << 8 | Wire.read();
  }
  return (float)gyro_z / 32768.0 * 250.0;
}

float getX(float accel){
  previous_aX = current_aX;
  current_aX = accel;
  prev_velocityX = velocityX;
  velocityX = velocityX + (current_aX + previous_aX) * 0.01;
  //checkout logic of math operations below - divide and integrate
  X = latitude_0 + ((velocityX + prev_velocityX) * 0.01) / R_latitude;
  return X;
}

float getY(float accel){
  previous_aY = current_aY;
  current_aY = accel;
  prev_velocityY = velocityY;
  velocityY = velocityY + (current_aY + previous_aY) * 0.01;
  //checkout logic of math operations below - divide and integrate
  Y = longitude_0 + ((velocityY + prev_velocityY) * 0.01) / R_longitude;
  return Y;
}

float getZ(float accel){
  previous_aZ = current_aZ;
  current_aZ = accel;
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

float matrix(int i, int j){
  float C11 = cos(ROLL_0) * cos(YAW_0) + sin(PITCH_0) * sin(ROLL_0) * sin(YAW_0);
  float C12 = -cos(ROLL_0) * sin(YAW_0) + sin(PITCH_0) * sin(ROLL_0) * cos(YAW_0);
  float C13 = cos(PITCH_0) * sin(ROLL_0);
  float C21 = cos(PITCH_0) * sin(YAW_0);
  float C22 = cos(PITCH_0) * cos(YAW_0);
  float C23 = sin(PITCH_0);
  float C31 = sin(ROLL_0) * cos(YAW_0) - sin(PITCH_0) * cos(ROLL_0) * sin(YAW_0);
  float C32 = -sin(ROLL_0) * sin(YAW_0) - sin(PITCH_0) * cos(ROLL_0) * sin(YAW_0);
  float C33 = cos(PITCH_0) * cos(ROLL_0);
  float matrix_LL[3][3] = {C11, C12, C13, C21, C22, C23, C31, C32, C33};
  // for(int i = 0; i < 3; i++){
  //   for(int j = 0; j < 3; j++){
  //     Serial.println(matrix_LL[i][j], 6);
  //   }
  // }
  return matrix_LL[i][j];
}

void bodyToLocal(){
  float Acc_matrix_BL[3][1] = {getAccelX(), getAccelX(), getAccelZ()};
  for(int i = 0, j = 0, k = 0; j <= 2; j++){
    while(k <= 2){
      Acc_matrix_ENUp[j][i] += ((matrix(j, k) * Acc_matrix_BL[k][i]));
      k++;
    }
    k = 0;
  }
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

    //try to combine into 1 func
    PITCH_0 = getPITCH();
    ROLL_0 = getROLL();
    YAW_0 = getYAW();

    Serial.print("PITCH_0 = "); Serial.print(PITCH_0, 6); Serial.print("    ROLL_0 = "); Serial.print(ROLL_0, 6); Serial.print("    YAW_0 = "); Serial.println(YAW_0, 6);
  }

  if(flag == 1 && alignment_flag == 1){

    //poll Accelerometr
    //get coordinates
    X = getX(Acc_matrix_ENUp[0][0]);
    Y = getY(Acc_matrix_ENUp[1][0]);
    Z = getZ(Acc_matrix_ENUp[2][0]);
    
    //update pitch, roll, yaw
    //update from first integration of acceleration and other math operations with velocity
    //better replace _0 variables with other variables, because _0 variables used for alignment
    // PITCH_0 =
    // ROLL_0 = 
    // YAW_0 = 

    //update matrix consisted of accelerations
    //update only after new pitch, roll and yaw variables
    bodyToLocal();
    
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

