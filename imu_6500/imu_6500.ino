#include "Wire.h"
#include "math.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU           16000000
#define TEST_LED        7
#define TEST_LED_MAGN   6
#define ALIGNMENT_BTN   5 

#define AK8963_ADDRESS  0x0C
#define MPU_addr        0x4A//0x68
//const int MPU_addr = 0x68; // адрес датчика
// массив данных
// [accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
// acc - ускорение, gyr - угловая скорость, temp - температура (raw)
//int16_t data[7];  

volatile int flag = 0;
bool alignment_flag = 0;
float OCR1 = 0;
float g = 9.81;

float Acc_matrix_BL[3][1]; //матрица с показаниями акселерометров
float Acc_matrix_ENUp[3][1] = {0, 0, 0}; //матрица ускорений в системе ENUp

float matrix_LL[3][3]; //матрица для выставки на перовм такте 
float matrix_W_LL[3][3]; //матрица посчитанных угловых скоростей из показаний акслерометра 
float matrix_W_B[3][3]; //матрица показаний ДУСа

float NEW_LL_MATRIX[3][3]; // новая матрица, которая используется для перевода в систему ENUp на последующих тактах работы алгоритма

//shirota & dolgota / f % a
float latitude_0 = 55.44;
float longitude_0 = 37.36 ;

float R_Earth = 6.371 * (10 ^ 6);
float R_latitude = (R_Earth * (1 - square(M_E))) / pow(1 - square(M_E) * square(sin(latitude_0)), 3 / 2);
float R_longitude = (R_Earth * (1 - square(M_E))) / pow(1 - square(M_E) * square(sin(longitude_0)), 1 / 2);

int16_t acc_x;
int16_t acc_y;
int16_t acc_z;

float aX;
float aY;
float aZ; 

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

float wE;
float wN;
float wUp;

float PITCH_0;
float ROLL_0;
float YAW_0;

float PITCH;
float ROLL;
float YAW;

float C_0;

int16_t magn_x;

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
  velocityX += (((current_aX + previous_aX) / 2 ) * 0.01);
  //checkout logic of math operations below - divide and integrate
  X = latitude_0;
  X += ((((velocityX + prev_velocityX)  / 2 ) * 0.01) / R_latitude);
  return X;
}

float getY(float accel){
  previous_aY = current_aY;
  current_aY = accel;
  prev_velocityY = velocityY;
  velocityY += (((current_aY + previous_aY) / 2 ) * 0.01);
  //checkout logic of math operations below - divide and integrate
  Y = longitude_0;
  Y += ((((velocityY + prev_velocityY) / 2 ) * 0.01) / R_longitude);
  return Y;
}

float getZ(float accel){
  previous_aZ = current_aZ;
  current_aZ = accel;
  prev_velocityZ = velocityZ;
  velocityZ +=(((current_aZ + previous_aZ) / 2 ) * 0.01);
  Z = Z + ((velocityZ + prev_velocityZ) / 2) * 0.01;
  return Z;
}

float getPITCH(float aX){
  PITCH_0 = -aX / g;
  return PITCH_0;
}

float getROLL(float aY){
  ROLL_0 = -aY / g;
  return ROLL_0;
}

float getYAW(float wX, float wY){
  YAW_0 = -atan(wX / wY);
  return YAW_0;
}

void bodyToLocal(float aX, float aY, float aZ){
  Acc_matrix_BL[0][0] = aX;
  Acc_matrix_BL[1][0] = aY;
  Acc_matrix_BL[2][0] = aZ;
  for(int i = 0, j = 0, k = 0; j <= 2; j++){
    while(k <= 2){
      Acc_matrix_ENUp[j][i] += ((matrix(j, k) * Acc_matrix_BL[k][i]));
      k++;
    }
    k = 0;
  }
}

float matrix(int i, int j){
  matrix_LL[0][0] = cos(ROLL_0) * cos(YAW_0) + sin(PITCH_0) * sin(ROLL_0) * sin(YAW_0);
  matrix_LL[0][1] = -cos(ROLL_0) * sin(YAW_0) + sin(PITCH_0) * sin(ROLL_0) * cos(YAW_0);
  matrix_LL[0][2] = cos(PITCH_0) * sin(ROLL_0);
  matrix_LL[1][0] = cos(PITCH_0) * sin(YAW_0);
  matrix_LL[1][1] = cos(PITCH_0) * cos(YAW_0);
  matrix_LL[1][2] = sin(PITCH_0);
  matrix_LL[2][0] = sin(ROLL_0) * cos(YAW_0) - sin(PITCH_0) * cos(ROLL_0) * sin(YAW_0);
  matrix_LL[2][1] = -sin(ROLL_0) * sin(YAW_0) - sin(PITCH_0) * cos(ROLL_0) * sin(YAW_0);
  matrix_LL[2][2] = cos(PITCH_0) * cos(ROLL_0);
  return matrix_LL[i][j];
}

float matrix_W_ENUp(int i, int j){
  matrix_W_LL[0][0] = 0;
  matrix_W_LL[0][1] = -wUp;
  matrix_W_LL[0][2] = wN;
  matrix_W_LL[1][0] = wUp;
  matrix_W_LL[1][1] = 0;
  matrix_W_LL[1][2] = -wE;
  matrix_W_LL[2][0] = -wN;
  matrix_W_LL[2][1] = wE;
  matrix_W_LL[2][2] = 0;
  return matrix_W_LL[i][j];
}

float matrix_W_DUS(int i, int j){
  matrix_W_B[0][0] = 0;
  matrix_W_B[0][1] = -wZ;
  matrix_W_B[0][2] = wY;
  matrix_W_B[1][0] = wZ;
  matrix_W_B[1][1] = 0;
  matrix_W_B[1][2] = -wX;
  matrix_W_B[2][0] = -wY;
  matrix_W_B[2][1] = wX;
  matrix_W_B[2][2] = 0;
  return matrix_W_B[i][j];
}

void Poisson(){
  for(int i = 0, j = 0, k = 0; i <= 2; i++){
    for(; j <= 2; j++){
      for(; k <= 2; k++){
        NEW_LL_MATRIX[i][j] += ((matrix(i, k) * matrix_W_DUS(k, j)) - matrix(i, k) * matrix_W_ENUp(k ,j));
      }
      k = 0;
    }
    j = 0;
  }
}

float getMagnX(){
  digitalWrite(TEST_LED_MAGN, digitalRead(TEST_LED_MAGN) ^1);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x16);
  Wire.endTransmission();

  Wire.requestFrom(MPU_addr, 2);

  while(Wire.available()){
    magn_x = Wire.read() << 8 | Wire.read();
  }
  return magn_x;
}

void setup() {
  Serial.begin(9600); 
  
  InitTimer(0.01);
  InitMPU();
  InitAK8963();

  pinMode(TEST_LED, OUTPUT);
  pinMode(TEST_LED_MAGN, OUTPUT);
  pinMode(ALIGNMENT_BTN, INPUT);
  digitalWrite(ALIGNMENT_BTN, LOW);

  magn_x = getMagnX();
  Serial.print("magn_X = "); Serial.println(magn_x);

}

void loop() {

  if(digitalRead(ALIGNMENT_BTN) == HIGH && alignment_flag == 0){
    alignment_flag = true;

    //poll Accelerometer
    aX = getAccelX();
    aY = getAccelY();
    aZ = getAccelZ();

    //poll Gyro 
    wX = getGyroX();
    wY = getGyroY();
    wZ = getGyroZ(); 

    //get first angles
    //try to combine into 1 func
    PITCH_0 = getPITCH(aX);
    ROLL_0 = getROLL(aY);
    YAW_0 = getYAW(wX, wY);

    bodyToLocal(aX, aY, aY);

    //get coordinates
    X = getX(Acc_matrix_ENUp[0][0]);
    Y = getY(Acc_matrix_ENUp[1][0]);
    Z = getZ(Acc_matrix_ENUp[2][0]);

    //calculate angular velocity
    wE = -velocityY / R_latitude;
    wN = velocityX / R_longitude;
    wUp = velocityX / R_longitude * tan(latitude_0);

    //resolve Poisson equation
    Poisson();
    
    //coefficient for PITCH calculation
    C_0 = sqrt(((NEW_LL_MATRIX[2][0] * NEW_LL_MATRIX[2][0]) + (NEW_LL_MATRIX[2][2] * NEW_LL_MATRIX[2][2])));

    //get angles
    PITCH = atan(fmod((NEW_LL_MATRIX[2][1] / C_0), 90.0));//(NEW_LL_MATRIX[2][1] / C_0);
    ROLL = atan(fmod((NEW_LL_MATRIX[2][0] / NEW_LL_MATRIX[2][2]), 180.0));//(NEW_LL_MATRIX[2][0] / NEW_LL_MATRIX[2][2]);
    YAW = atan(fmod((NEW_LL_MATRIX[0][1] / NEW_LL_MATRIX[1][1]), 360.0));//(NEW_LL_MATRIX[0][1] / NEW_LL_MATRIX[1][1]);

    // Serial.print("PITCH_0 = "); Serial.print(PITCH_0 * 180.0 / M_PI, 6); Serial.print("    ROLL_0 = "); Serial.print(ROLL_0 * 180.0 / M_PI, 6); Serial.print("    YAW_0 = "); Serial.println(YAW_0 * 180.0 / M_PI, 6);

    // Serial.print("aX = "); Serial.print(Acc_matrix_ENUp[0][0]); Serial.print("    aY = "); Serial.print(Acc_matrix_ENUp[1][0]); Serial.print("    aZ = "); Serial.println(Acc_matrix_ENUp[2][0]);

    // Serial.print("X1 = "); Serial.print(X); Serial.print("    Y1 = "); Serial.print(Y); Serial.print("    Z1 = "); Serial.println(Z);

    // for(int i = 0; i < 3; i++){
    //   for(int j = 0; j < 3; j++){
    //     Serial.println(NEW_LL_MATRIX[i][j], 6);
    //   }
    // }

    // Serial.print("C_0 = "); Serial.println(C_0, 6);

    // Serial.print("PITCH = "); Serial.print(PITCH * 180.0 / M_PI, 6); Serial.print("    ROLL = "); Serial.print(ROLL * 180.0 / M_PI, 6); Serial.print("    YAW = "); Serial.println(YAW * 180.0 / M_PI, 6);

  }

  
  if(flag == 1 && alignment_flag == 1){

    //poll Accelerometer
    aX = getAccelX();
    aY = getAccelY();
    aZ = getAccelZ();
    
    //poll Gyro 
    wX = getGyroX();
    wY = getGyroY();
    wZ = getGyroZ(); 

    //update pitch, roll, yaw
    //update from first integration of acceleration and other math operations with velocity
    //better replace _0 variables with other variables, because _0 variables used for alignment
    PITCH_0 = PITCH;
    ROLL_0 = ROLL;
    YAW_0 = YAW; 

    //update matrix consisted of accelerations
    //update only after new pitch, roll and yaw variables
    bodyToLocal(aX, aY, aY);

    //get coordinates
    X = getX(Acc_matrix_ENUp[0][0]);
    Y = getY(Acc_matrix_ENUp[1][0]);
    Z = getZ(Acc_matrix_ENUp[2][0]);   

    //calculate angular velocity
    wE = -velocityY / R_latitude;
    wN = velocityX / R_longitude;
    wUp = velocityX / R_longitude * tan(X);

    //resolve Poisson equation
    Poisson();

    //coefficinet for PITCH calculation
    C_0 = sqrt(((NEW_LL_MATRIX[2][0] * NEW_LL_MATRIX[2][0]) + (NEW_LL_MATRIX[2][2] * NEW_LL_MATRIX[2][2])));

    //get angles
    // PITCH = atan((NEW_LL_MATRIX[2][1] / C_0));//fmod((NEW_LL_MATRIX[2][1] / C_0), 90.0)
    // ROLL = atan((NEW_LL_MATRIX[2][0] / NEW_LL_MATRIX[2][2]));//fmod((NEW_LL_MATRIX[2][0] / NEW_LL_MATRIX[2][2]), 180.0)
    // YAW = atan((NEW_LL_MATRIX[0][1] / NEW_LL_MATRIX[1][1]));//fmod((NEW_LL_MATRIX[0][1] / NEW_LL_MATRIX[1][1]), 360.0)

    PITCH = atan(fmod((NEW_LL_MATRIX[2][1] / C_0), 90.0));//(NEW_LL_MATRIX[2][1] / C_0);
    ROLL = atan(fmod((NEW_LL_MATRIX[2][0] / NEW_LL_MATRIX[2][2]), 180.0));//(NEW_LL_MATRIX[2][0] / NEW_LL_MATRIX[2][2]);
    YAW = atan(fmod((NEW_LL_MATRIX[0][1] / NEW_LL_MATRIX[1][1]), 360.0));//(NEW_LL_MATRIX[0][1] / NEW_LL_MATRIX[1][1]);
    
    // Serial.print("X = "); Serial.print(X); Serial.print("    Y = "); Serial.print(Y); Serial.print("    Z = "); Serial.print(Z); 
    // Serial.print("    PITCH = "); Serial.print(PITCH * 180.0 / M_PI, 6); Serial.print("    ROLL = "); Serial.print(ROLL * 180.0 / M_PI, 6); Serial.print("    YAW = "); Serial.println(YAW * 180.0 / M_PI, 6);

    magn_x = getMagnX();
    Serial.print("magn_X = "); Serial.println(magn_x);

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

void InitAK8963(){
  Wire.beginTransmission(AK8963_ADDRESS);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission(true);

  Wire.beginTransmission(AK8963_ADDRESS);
  Wire.write(0x0A);
  Wire.write(0x0F);
  Wire.endTransmission(true);

  Wire.beginTransmission(AK8963_ADDRESS);
  Wire.write(0x0A);
  Wire.write(0x16);
  Wire.endTransmission(true); 
}

float Timer1ClockSetup(float time){
  float resolution = (1.0 / (F_CPU / 1024.0));
  OCR1 = time / resolution;
  return OCR1;
}

