#include <Wire.h>

// Адреса датчиков
#define MPU9250_ADDR 0x68
#define MAG_ADDR 0x0C  // Адрес магнитометра AK8963

// Регистры MPU9250
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define MAG_WHO_AM_I 0x00  // Регистр идентификатора магнитометра
#define MAG_ST1 0x02
#define MAG_XOUT_L 0x03

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Инициализация MPU9250
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0);     // Включаем MPU9250
    Wire.endTransmission(true);

    // Проверка подключения к магнитометру
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 1, true);
    
    if (Wire.available()) {
        byte whoami = Wire.read();
        if (whoami == 0x48) {  // Ожидаемое значение WHO_AM_I = 0x48
            Serial.println("Магнитометр AK8963 найден!");
        } else {
            Serial.print("Ошибка! Магнитометр не найден. Код: 0x");
            Serial.println(whoami, HEX);
            while (1); // Остановка программы
        }
    } else {
        Serial.println("Ошибка! Магнитометр не отвечает!");
        while (1);
    }

    // Инициализация магнитометра AK8963
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x0A);  // Контрольный регистр 1
    Wire.write(0x16);  // 16-bit, режим измерений
    Wire.endTransmission(true);
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

    // Считываем акселерометр и гироскоп
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 14, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();  // Пропускаем температуру
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    // Считываем магнитометр
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_ST1);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 1, true);

    if (Wire.read() & 0x01) {  // Доступны данные
        Wire.beginTransmission(MAG_ADDR);
        Wire.write(MAG_XOUT_L);
        Wire.endTransmission(false);
        Wire.requestFrom(MAG_ADDR, 7, true);

        mx = Wire.read() | Wire.read() << 8;
        my = Wire.read() | Wire.read() << 8;
        mz = Wire.read() | Wire.read() << 8;
    }

    // Выводим значения
    Serial.print("AX: "); Serial.print(ax);
    Serial.print(" AY: "); Serial.print(ay);
    Serial.print(" AZ: "); Serial.print(az);
    Serial.print(" | GX: "); Serial.print(gx);
    Serial.print(" GY: "); Serial.print(gy);
    Serial.print(" GZ: "); Serial.print(gz);
    Serial.print(" | MX: "); Serial.print(mx);
    Serial.print(" MY: "); Serial.print(my);
    Serial.print(" MZ: "); Serial.println(mz);

    delay(500);
}