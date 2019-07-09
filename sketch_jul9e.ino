/**/#include "Wire.h" // This library allows you to communicate with I2C devices.#include <Kalman.h>const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.KalmanFilter filter;float angulo;void setup() {  Serial.begin(9600);  Wire.begin();  pinMode(LED_BUILTIN, OUTPUT);  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)  Wire.write(0x6B); // PWR_MGMT_1 register  Wire.write(0); // set to zero (wakes up the MPU-6050)  Wire.endTransmission(true);}void loop() {  int temp=0;  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)  Wire.beginTransmission(MPU_ADDR);  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers  accX = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)  accY = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)  accZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)  temp = Wire.read()<<8 | Wire.read();  //gyro code  gyroX = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)  gyroY = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)  gyroZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)  angulo=filter.update(accX, accY, accZ, gyroX, gyroY, gyroZ);}