
#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_Follow(void);

private:
  volatile uint16_t UltrasoundData_mm;
  volatile uint16_t UltrasoundData_cm;
  boolean UltrasoundDetectionStatus = false;
public:
  boolean Car_LeaveTheGround = true;
  const int ObstacleDetection = 20;
};
extern ApplicationFunctionSet Application_FunctionSet;
#endif
