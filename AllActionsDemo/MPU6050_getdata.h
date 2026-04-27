
#ifndef _MPU6050_getdata_H_
#define _MPU6050_getdata_H_
#include <Arduino.h>
class MPU6050_getdata
{
public:
  bool MPU6050_dveInit(void);
  bool MPU6050_calibration(void);
  bool MPU6050_dveGetEulerAngles(float *Yaw);

public:

  int16_t gz;

  unsigned long now, lastTime = 0;
  float dt;
  float agz = 0;
  long gzo = 0;
};

extern MPU6050_getdata MPU6050Getdata;
#endif
