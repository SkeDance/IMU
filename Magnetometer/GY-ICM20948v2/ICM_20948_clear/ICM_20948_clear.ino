// #include <Wire.h>

// #define ICM_ADDR    0x68
// #define MAG_ADDR    0x0c

// float acc_x; 

// void InitICM(){
//   Wire.begin();
//   Wire.beginTransmission(ICM_ADDR);
//   Wire.write(0x06);  // PWR_MGMT_1 register
//   Wire.write(0x00);     // set to zero (wakes up the ICM20948)
//   Wire.endTransmission(true);
// }

// float getAccelX(){
//   Wire.beginTransmission(ICM_ADDR);
//   Wire.write(0x2D);
//   Wire.endTransmission(false);

//   Wire.requestFrom(ICM_ADDR, 2);

//   while(Wire.available()){
//     acc_x = Wire.read() << 8 | Wire.read();
//   }
//   return (float)acc_x / 32768 * 2.0;
// }

// void setup() {
//   Serial.begin(9600);

//   InitICM();
  
// }

// void loop() {
//   float X = getAccelX();
//   Serial.println(X);
  
// }


#include <Wire.h>

#define ICM20948_ADDRESS 0x68  // Адрес датчика ICM20948 по умолчанию
#define AK09916_ADDRESS 0x0C   // Адрес магнитометра AK09916

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Инициализация ICM20948 для работы с магнитометром
  ICM20948_init();
  
  // Инициализация магнитометра AK09916
  AK09916_init();
}

void loop() {
  // Чтение данных с магнитометра
  int16_t mx, my, mz;
  readMagData(&mx, &my, &mz);
  
  // Вывод данных в Serial Monitor
  Serial.print("Mag: ");
  Serial.print(mx); Serial.print(", ");
  Serial.print(my); Serial.print(", ");
  Serial.print(mz); Serial.println();
  
  delay(100);  // Задержка для удобства чтения
}

void ICM20948_init() {
  // Сброс устройства
  writeRegister(ICM20948_ADDRESS, 0x06, 0x01);
  delay(100);
  
  // Выбор банка 0
  writeRegister(ICM20948_ADDRESS, 0x7F, 0x00);
  
  // Включение I2C Master для работы с магнитометром
  writeRegister(ICM20948_ADDRESS, 0x0F, 0x20);  // Включение I2C Master
  
  // Выбор банка 3
  writeRegister(ICM20948_ADDRESS, 0x7F, 0x03);
  
  // Настройка I2C Master для работы с магнитометром
  writeRegister(ICM20948_ADDRESS, 0x01, 0x0C);  // Настройка I2C_MST_CTRL
}

void AK09916_init() {
  // Сброс магнитометра
  writeMagRegister(AK09916_ADDRESS, 0x32, 0x01);
  delay(100);
  
  // Переключение в режим непрерывного измерения
  writeMagRegister(AK09916_ADDRESS, 0x31, 0x02);
}

void readMagData(int16_t* mx, int16_t* my, int16_t* mz) {
  // Чтение данных магнитометра
  uint8_t buffer[6];
  readMagRegisters(AK09916_ADDRESS, 0x11, 6, buffer);
  
  *mx = (buffer[1] << 8) | buffer[0];
  *my = (buffer[3] << 8) | buffer[2];
  *mz = (buffer[5] << 8) | buffer[4];
}

void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t address, uint8_t reg, uint8_t count, uint8_t* data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(address, count);
  for (uint8_t i = 0; i < count; i++) {
    data[i] = Wire.read();
  }
}

void writeMagRegister(uint8_t address, uint8_t reg, uint8_t value) {
  // Настройка I2C Master для работы с магнитометром
  writeRegister(ICM20948_ADDRESS, 0x7F, 0x03);  // Банк 3
  writeRegister(ICM20948_ADDRESS, 0x01, 0x80 | (address << 1));  // Адрес магнитометра
  writeRegister(ICM20948_ADDRESS, 0x02, reg);  // Регистр магнитометра
  writeRegister(ICM20948_ADDRESS, 0x03, value);  // Значение для записи
  writeRegister(ICM20948_ADDRESS, 0x05, 0x01);  // Запуск записи
}

void readMagRegisters(uint8_t address, uint8_t reg, uint8_t count, uint8_t* data) {
  // Настройка I2C Master для работы с магнитометром
  writeRegister(ICM20948_ADDRESS, 0x7F, 0x03);  // Банк 3
  writeRegister(ICM20948_ADDRESS, 0x01, 0x80 | (address << 1));  // Адрес магнитометра
  writeRegister(ICM20948_ADDRESS, 0x02, reg);  // Регистр магнитометра
  writeRegister(ICM20948_ADDRESS, 0x05, 0x01);  // Запуск чтения
  
  // Ожидание завершения чтения
  delay(10);
  
  // Чтение данных
  writeRegister(ICM20948_ADDRESS, 0x7F, 0x03);  // Банк 3
  for (uint8_t i = 0; i < count; i++) {
    data[i] = readRegister(ICM20948_ADDRESS, 0x04);  // Чтение данных из регистра I2C_MST_DATA
  }
}

uint8_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(address, (uint8_t)1);
  return Wire.read();
}