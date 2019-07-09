#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <Servo.h> 
#include <PID_v1.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
// Declare the Servo pin 
int servoPin = 4; 
// Create a servo object 
Servo Servo1; 

/*********Tune these 4 values for your BOT*********/
double setpoint= 94; //set the value when the bot is perpendicular to ground using serial monitor. 
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 21; //Set this first
double Kd = 0.8; //Set this secound
double Ki = 140; //Finally set this 
/******End of values setting*********/

double input, output;
PID mypid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
char tmp_str[7];

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Servo1.attach(servoPin); 
    
  //Define pid values
  mypid.SetMode(AUTOMATIC);
  mypid.SetSampleTime(10);
  mypid.SetOutputLimits(50, 100);  
}

void loop() {
  
  //Teste para PID

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  input = 94;

  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  
  if(accelerometer_y > 1000){
    Serial.println("Titlting left");
    input = 70;
  }

  if(accelerometer_y < - 1000){
    Serial.println("Titlting right");
    input = 100;
  }
    
  mypid.Compute();
    
  //Alterar o que o servo escreve
  Servo1.write(output);
  /*
  if(accelerometer_y > 1300){
    Serial.println("Titlting left");
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    Servo1.write(70); 
  }

  if(accelerometer_y < - 1300){
    Serial.println("Titlting right");
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    Servo1.write(110); 
  }*/

  Serial.print("Output= "); Serial.print(output);
  Serial.print(" | Input = "); Serial.print(input);
  Serial.print(" | Setpoint = "); Serial.print(setpoint);
  Serial.println();

  // print out data
  //Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print("Y = "); Serial.print(convert_int16_to_str(accelerometer_y));
  //Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  
  Serial.println();

  // delay
  delay(200);
}
