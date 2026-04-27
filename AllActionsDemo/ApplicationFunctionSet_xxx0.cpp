
#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h"
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;

static boolean
function_xxx(long x, long s, long e)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

enum SmartRobotCarMotionControl
{
  Forward,
  Backward,
  Left,
  Right,
  LeftForward,
  LeftBackward,
  RightForward,
  RightBackward,
  stop_it
};

enum SmartRobotCarFunctionalModel
{
  Standby_mode,
  TraceBased_mode,
  ObstacleAvoidance_mode,
  Follow_mode,
  Rocker_mode,
};

struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();
  AppServo.DeviceDriverSet_Servo_Init(90);

  while (Serial.read() >= 0)
  {

  }
  delay(2000);
  Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
}

static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw;
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control( direction_void,  0,
                                            direction_void,  0,  control_enable);
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }

  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }

  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward)
  {
    AppMotor.DeviceDriverSet_Motor_control( direction_just,  R,
                                            direction_just,  L,  control_enable);
  }
  else if (direction == Backward)
  {
    AppMotor.DeviceDriverSet_Motor_control( direction_back,  L,
                                            direction_back,  R,  control_enable);
  }
}

static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;

    Kp = 2;
    UpperLimit = 180;

  switch (direction)
  {
  case
      Forward:

    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control( direction_just,  speed,
                                              direction_just,  speed,  control_enable);
    }
    else
    {
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case  Backward:

    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control( direction_back,  speed,
                                              direction_back,  speed,  control_enable);
    }
    else
    {
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case  Left:

    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control( direction_just,  speed,
                                            direction_back,  speed,  control_enable);
    break;
  case  Right:

    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control( direction_back,  speed,
                                            direction_just,  speed,  control_enable);
    break;
  case  LeftForward:

    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control( direction_just,  speed,
                                            direction_just,  speed / 2,  control_enable);
    break;
  case  LeftBackward:

    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control( direction_back,  speed,
                                            direction_back,  speed / 2,  control_enable);
    break;
  case  RightForward:

    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control( direction_just,  speed / 2,
                                            direction_just,  speed,  control_enable);
    break;
  case  RightBackward:

    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control( direction_back,  speed / 2,
                                            direction_back,  speed,  control_enable);
    break;
  case  stop_it:

    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control( direction_void,  0,
                                            direction_void,  0,  control_enable);
    break;
  default:
    directionRecord = 10;
    break;
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_Follow(void)
{
  static uint16_t ULTRASONIC_Get = 0;
  static unsigned long ULTRASONIC_time = 0;
  static uint8_t Position_Servo = 1;
  static uint8_t timestamp = 3;
  static uint8_t OneCycle = 1;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Follow_mode)
  {

    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&ULTRASONIC_Get );
    if (false == function_xxx(ULTRASONIC_Get, 0, 20))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      static unsigned long time_Servo = 0;
      static uint8_t Position_Servo_xx = 0;

      if (timestamp == 3)
      {
        if (Position_Servo_xx != Position_Servo)
        {
          Position_Servo_xx = Position_Servo;

          if (Position_Servo == 1)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(80 );
          }
          else if (Position_Servo == 2)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(20 );
          }
          else if (Position_Servo == 3)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(80 );
          }
          else if (Position_Servo == 4)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(150 );
          }
        }
      }
      else
      {
        if (timestamp == 1)
        {
          timestamp = 2;
          time_Servo = millis();
        }
      }
      if (millis() - time_Servo > 1000)
      {
        timestamp = 3;
        Position_Servo += 1;
        OneCycle += 1;
        if (OneCycle > 4)
        {
          Position_Servo = 1;
          OneCycle = 5;
        }
      }
    }
    else
    {
      OneCycle = 1;
      timestamp = 1;
      if ((Position_Servo == 1))
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      }
      else if ((Position_Servo == 2))
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
      }
      else if ((Position_Servo == 3))
      {

        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      }
      else if ((Position_Servo == 4))
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
      }
    }
  }
  else
  {
    ULTRASONIC_Get = 0;
    ULTRASONIC_time = 0;
  }
}
