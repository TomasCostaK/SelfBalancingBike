#include <Wire.h>
#include "Kalman.h"

namespace N
{
class KalmanFilter{ 
public:
  //KalmanFilter();
  float update(int16_t accX,int16_t accY,int16_t accZ,int16_t gyroX,int16_t gyroY,int16_t gyroZ,double dt);
  char* convert_int16_to_str(int16_t i);

  #define RESTRICT_PITCH
private:
  Kalman kalmanX; // Create the Kalman instances
  Kalman kalmanY;
  
  double gyroXangle, gyroYangle; // Angle calculate using the gyro only
  double compAngleX, compAngleY; // Calculated angle using a complementary filter
  double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
  
  char tmp_str[7];
};
}