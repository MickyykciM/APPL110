
#include <avr/wdt.h>

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
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;

static boolean
function_xxx(long x, long s, long e)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}
static void
delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
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
  CMD_inspect,
  CMD_Programming_mode,
  CMD_ClearAllFunctions_Standby_mode,
  CMD_ClearAllFunctions_Programming_mode,
  CMD_MotorControl,
  CMD_CarControl_TimeLimit,
  CMD_CarControl_NoTimeLimit,
  CMD_MotorControl_Speed,
  CMD_ServoControl,
  CMD_LightingControl_TimeLimit,
  CMD_LightingControl_NoTimeLimit,

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
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  AppIRrecv.DeviceDriverSet_IRrecv_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
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

  switch (Application_SmartRobotCarxxx0.Functional_Mode)
  {
  case Rocker_mode:
    Kp = 10;
    UpperLimit = 255;
    break;
  case ObstacleAvoidance_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case Follow_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_TimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_NoTimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  default:
    Kp = 10;
    UpperLimit = 255;
    break;
  }
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

void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{

  {
    static unsigned long VoltageData_time = 0;
    static int VoltageData_number = 1;
    if (millis() - VoltageData_time > 10)
    {
      VoltageData_time = millis();
      VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
      if (VoltageData_V < VoltageDetection)
      {
        VoltageData_number++;
        if (VoltageData_number == 500)
        {
          VoltageDetectionStatus = true;
          VoltageData_number = 0;
        }
      }
      else
      {
        VoltageDetectionStatus = false;
      }
    }
  }

  {
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingDetectionStatus_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);

    ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
  }

}

void ApplicationFunctionSet::ApplicationFunctionSet_Bootup(void)
{
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

static void CMD_Lighting(uint8_t is_LightingSequence, int8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B)
{
  switch (is_LightingSequence)
  {
  case 0:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(NUM_LEDS, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 1:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(3, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 2:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(2, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 3:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(1, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 4:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(0, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 5:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(4, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  default:
    break;
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void)
{
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);
  if (true == VoltageDetectionStatus)
  {
    if ((millis() - getAnalogue_time) > 3000)
    {
      getAnalogue_time = millis();
    }
  }
  unsigned long temp = millis() - getAnalogue_time;
  if (function_xxx((temp), 0, 500) && VoltageDetectionStatus == true)
  {
    switch (temp)
    {
    case  0 ... 49:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Red);
      break;
    case  50 ... 99:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Black);
      break;
    case  100 ... 149:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Red);
      break;
    case  150 ... 199:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Black);
      break;
    case  200 ... 249:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Red);
      break;
    case  250 ... 299:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Red);
      break;
    case  300 ... 349:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Black);
      break;
    case  350 ... 399:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Red);
      break;
    case  400 ... 449:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Black);
      break;
    case  450 ... 499:

      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Red);
      break;
    default:
      break;
    }
  }
  else if (((function_xxx((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false)
  {
    switch (Application_SmartRobotCarxxx0.Functional_Mode)
    {
    case  Standby_mode:

      {
        if (VoltageDetectionStatus == true)
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Red);
          delay(30);
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Black);
          delay(30);
        }
        else
        {
          static uint8_t setBrightness = 0;
          static boolean et = false;
          static unsigned long time = 0;

          if ((millis() - time) > 10)
          {
            time = millis();
            if (et == false)
            {
              setBrightness += 1;
              if (setBrightness == 100)
                et = true;
            }
            else if (et == true)
            {
              setBrightness -= 1;
              if (setBrightness == 0)
                et = false;
            }
          }

          AppRBG_LED.leds[0] = CRGB::Violet;
          FastLED.setBrightness(setBrightness);
          FastLED.show();
        }
      }
      break;
    case  CMD_Programming_mode:

      {
      }
      break;
    case  TraceBased_mode:

      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Green);
      }
      break;
    case  ObstacleAvoidance_mode:

      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Yellow);
      }
      break;
    case  Follow_mode:

      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Blue);
      }
      break;
    case  Rocker_mode:

      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , 2 , CRGB::Violet);
      }
      break;
    default:
      break;
    }
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_Rocker(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == Rocker_mode)
  {

    uint8_t spd = Rocker_CarSpeed;
    SmartRobotCarMotionControl dir = Application_SmartRobotCarxxx0.Motion_Control;
    if (dir == Left || dir == Right)
    {
      spd = (uint8_t)((uint16_t)spd * 60 / 100);
      if (spd < 70) spd = 70;
    }
    ApplicationFunctionSet_SmartRobotCarMotionControl(dir, spd);
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(void)
{
  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }

#if _Test_print
    static unsigned long print_time = 0;
    if (millis() - print_time > 500)
    {
      print_time = millis();
      Serial.print("ITR20001_getAnaloguexxx_L=");
      Serial.println(getAnaloguexxx_L);
      Serial.print("ITR20001_getAnaloguexxx_M=");
      Serial.println(getAnaloguexxx_M);
      Serial.print("ITR20001_getAnaloguexxx_R=");
      Serial.println(getAnaloguexxx_R);
    }
#endif
    if (function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E))
    {

      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E))
    {

      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E))
    {

      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else
    {
      if (timestamp == true)
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }

      if ((function_xxx((millis() - MotorRL_time), 0, 200) || function_xxx((millis() - MotorRL_time), 1600, 2000)) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      }
      else if (((function_xxx((millis() - MotorRL_time), 200, 1600))) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      }
      else if ((function_xxx((millis() - MotorRL_time), 3000, 3500)))
      {
        BlindDetection = false;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
    }
  }
  else if (false == timestamp)
  {
    BlindDetection = true;
    timestamp = true;
    MotorRL_time = 0;
  }
}

static void ApplicationFunctionSet_GyroTurn(bool turn_right, uint16_t degrees)
{
  float yawStart = 0, yawNow = 0;
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&yawStart);

  ApplicationFunctionSet_SmartRobotCarMotionControl(
      turn_right ? Right : Left, 150);

  unsigned long t0 = millis();
  while (millis() - t0 < 1600UL)
  {
    wdt_reset();
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&yawNow);
    float delta = yawNow - yawStart;
    while (delta >  180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    if (fabs(delta) >= (float)degrees) break;
    delay(5);
  }
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
}

void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{

  static boolean first_is = true;
  static uint8_t backup_count = 0;
  static unsigned long backup_window_start = 0;

  if (Application_SmartRobotCarxxx0.Functional_Mode != ObstacleAvoidance_mode)
  {
    first_is = true;
    backup_count = 0;
    return;
  }
  if (Car_LeaveTheGround == false)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    return;
  }
  if (first_is)
  {
    AppServo.DeviceDriverSet_Servo_control(90);
    first_is = false;
    backup_window_start = millis();
    backup_count = 0;
  }
  if (millis() - backup_window_start > 5000UL)
  {
    backup_window_start = millis();
    backup_count = 0;
  }

  uint16_t dist = 0;

  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&dist);
  if (dist > 25)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 130);
    return;
  }

  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  wdt_reset();

  const uint8_t angles[4] = {30, 60, 90, 110};
  uint8_t best_idx = 0;
  uint16_t best_dist = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
    AppServo.DeviceDriverSet_Servo_control(angles[i]);
    wdt_reset();
    delay(60);
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&dist);
    if (dist > best_dist)
    {
      best_dist = dist;
      best_idx = i;
    }
  }
  AppServo.DeviceDriverSet_Servo_control(90);
  wdt_reset();

  if (best_dist > 30)
  {
    int16_t turn_deg = (int16_t)angles[best_idx] - 90;
    if (turn_deg > 5)
      ApplicationFunctionSet_GyroTurn(false , (uint16_t)turn_deg);
    else if (turn_deg < -5)
      ApplicationFunctionSet_GyroTurn(true , (uint16_t)(-turn_deg));
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 130);
    return;
  }

  backup_count++;
  if (backup_count > 3)
  {
    ApplicationFunctionSet_GyroTurn(false , 170);
    backup_count = 0;
    return;
  }
  ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 130);
  delay_xxx(600);
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
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

void ApplicationFunctionSet::ApplicationFunctionSet_Servo(uint8_t Set_Servo)
{
  static int z_angle = 9;
  static int y_angle = 9;
  uint8_t temp_Set_Servo = Set_Servo;

  switch (temp_Set_Servo)
  {
  case 1 ... 2:
  {
    if (1 == temp_Set_Servo)
    {
      y_angle -= 1;
    }
    else if (2 == temp_Set_Servo)
    {
      y_angle += 1;
    }
    if (y_angle <= 3)
    {
      y_angle = 3;
    }
    if (y_angle >= 11)
    {
      y_angle = 11;
    }
    AppServo.DeviceDriverSet_Servo_controls( 2,  y_angle);
  }
  break;

  case 3 ... 4:
  {
    if (3 == temp_Set_Servo)
    {
      z_angle += 1;
    }
    else if (4 == temp_Set_Servo)
    {
      z_angle -= 1;
    }

    if (z_angle <= 1)
    {
      z_angle = 1;
    }
    if (z_angle >= 17)
    {
      z_angle = 17;
    }
    AppServo.DeviceDriverSet_Servo_controls( 1,  z_angle);
  }
  break;
  case 5:
    AppServo.DeviceDriverSet_Servo_controls( 2,  9);
    AppServo.DeviceDriverSet_Servo_controls( 1,  9);
    break;
  default:
    break;
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_Standby(void)
{
  static bool is_ED = true;
  static uint8_t cout = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Standby_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    if (true == is_ED)
    {
      static unsigned long timestamp;
      if (millis() - timestamp > 20)
      {
        timestamp = millis();
        if (ApplicationFunctionSet_SmartRobotCarLeaveTheGround() )
        {
          cout += 1;
        }
        else
        {
          cout = 0;
        }
        if (cout > 10)
        {
          is_ED = false;
          AppMPU6050getdata.MPU6050_calibration();
        }
      }
    }
  }
}

void ApplicationFunctionSet::CMD_inspect_xxx0(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_inspect)
  {
    Serial.println("CMD_inspect");
    delay(100);
  }
}

void ApplicationFunctionSet::CMD_MotorControl_xxx0(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed)
{
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl)
  {
    MotorControl = true;
    if (0 == is_MotorDirection)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      switch (is_MotorSelection)
      {
      case 0:
      {
        is_MotorSpeed_A = is_MotorSpeed;
        is_MotorSpeed_B = is_MotorSpeed;
        if (1 == is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_just,  is_MotorSpeed_A,
                                                  direction_just,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else if (2 == is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_back,  is_MotorSpeed_A,
                                                  direction_back,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else
        {
          return;
        }
      }
      break;
      case 1:
      {
        is_MotorSpeed_A = is_MotorSpeed;
        if (1 == is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_just,  is_MotorSpeed_A,
                                                  direction_void,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else if (2 == is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_back,  is_MotorSpeed_A,
                                                  direction_void,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else
        {
          return;
        }
      }
      break;
      case 2:
      {
        is_MotorSpeed_B = is_MotorSpeed;
        if (1 == is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_void,  is_MotorSpeed_A,
                                                  direction_just,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else if (2 == is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_void,  is_MotorSpeed_A,
                                                  direction_back,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else
        {
          return;
        }
      }
      break;
      default:
        break;
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControl_xxx0(void)
{
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl)
  {
    MotorControl = true;
    if (0 == CMD_is_MotorDirection)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      switch (CMD_is_MotorSelection)
      {
      case 0:
      {
        is_MotorSpeed_A = CMD_is_MotorSpeed;
        is_MotorSpeed_B = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_just,  is_MotorSpeed_A,
                                                  direction_just,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else if (2 == CMD_is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_back,  is_MotorSpeed_A,
                                                  direction_back,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else
        {
          return;
        }
      }
      break;
      case 1:
      {
        is_MotorSpeed_A = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_just,  is_MotorSpeed_A,
                                                  direction_void,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else if (2 == CMD_is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_back,  is_MotorSpeed_A,
                                                  direction_void,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else
        {
          return;
        }
      }
      break;
      case 2:
      {
        is_MotorSpeed_B = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_void,  is_MotorSpeed_A,
                                                  direction_just,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else if (2 == CMD_is_MotorDirection)
        {
          AppMotor.DeviceDriverSet_Motor_control( direction_void,  is_MotorSpeed_A,
                                                  direction_back,  is_MotorSpeed_B,
                                                  control_enable);
        }
        else
        {
          return;
        }
      }
      break;
      default:
        break;
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}

static void CMD_CarControl(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  switch (is_CarDirection)
  {
  case 1:
    ApplicationFunctionSet_SmartRobotCarMotionControl(Left, is_CarSpeed);
    break;
  case 2:
    ApplicationFunctionSet_SmartRobotCarMotionControl(Right, is_CarSpeed);
    break;
  case 3:
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, is_CarSpeed);
    break;
  case 4:
    ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, is_CarSpeed);
    break;
  default:
    break;
  }
}

void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint32_t is_Timer)
{
  static boolean CarControl = false;
  static boolean CarControl_TE = false;
  static boolean CarControl_return = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit)
  {
    CarControl = true;
    if (is_Timer != 0)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_CarControl_Millis) > (is_Timer))
      {
        CarControl_TE = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
        if (CarControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false;
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      CMD_CarControl(is_CarDirection, is_CarSpeed);
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      Application_SmartRobotCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(void)
{
  static boolean CarControl = false;
  static boolean CarControl_TE = false;
  static boolean CarControl_return = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit)
  {
    CarControl = true;
    if (CMD_is_CarTimer != 0)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_CarControl_Millis) > (CMD_is_CarTimer))
      {
        CarControl_TE = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
        if (CarControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false;
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      Application_SmartRobotCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  static boolean CarControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit)
  {
    CarControl = true;
    CMD_CarControl(is_CarDirection, is_CarSpeed);
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(void)
{
  static boolean CarControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit)
  {
    CarControl = true;
    CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
    }
  }
}

void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(uint8_t is_Speed_L, uint8_t is_Speed_R)
{
  static boolean MotorControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl_Speed)
  {
    MotorControl = true;
    if (is_Speed_L == 0 && is_Speed_R == 0)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      AppMotor.DeviceDriverSet_Motor_control( direction_just,  is_Speed_L,
                                              direction_just,  is_Speed_R,
                                              control_enable);
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(void)
{
  static boolean MotorControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl_Speed)
  {
    MotorControl = true;
    if (CMD_is_MotorSpeed_L == 0 && CMD_is_MotorSpeed_R == 0)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      AppMotor.DeviceDriverSet_Motor_control( direction_just,  CMD_is_MotorSpeed_L,
                                              direction_just,  CMD_is_MotorSpeed_R,
                                              control_enable);
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
    }
  }
}

void ApplicationFunctionSet::CMD_ServoControl_xxx0(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ServoControl)
  {
    AppServo.DeviceDriverSet_Servo_controls( CMD_is_Servo,  CMD_is_Servo_angle / 10);
    Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
  }
}

void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B,
                                                               uint32_t is_LightingTimer)
{
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false;
  static boolean LightingControl_return = false;

  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit)
  {
    LightingControl = true;
    if (is_LightingTimer != 0)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_LightingControl_Millis) > (is_LightingTimer))
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      }
      else
      {
        LightingControl_TE = false;
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false)
    {
      CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    }
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl_return = false;
      LightingControl = false;
      Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(void)
{
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false;
  static boolean LightingControl_return = false;

  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit)
  {
    LightingControl = true;
    if (CMD_is_LightingTimer != 0)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_LightingControl_Millis) > (CMD_is_LightingTimer))
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      }
      else
      {
        LightingControl_TE = false;
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false)
    {
      CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
    }
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl_return = false;
      LightingControl = false;
      Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B)
{
  static boolean LightingControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit)
  {
    LightingControl = true;
    CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(void)
{
  static boolean LightingControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit)
  {
    LightingControl = true;
    CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl = false;
    }
  }
}

void ApplicationFunctionSet::CMD_ClearAllFunctions_xxx0(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Standby_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , NUM_LEDS , CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
  }
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Programming_mode)
  {

    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 , NUM_LEDS , CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
  }
}

void ApplicationFunctionSet::CMD_UltrasoundModuleStatus_xxx0(uint8_t is_get)
{
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm );
  UltrasoundDetectionStatus = function_xxx(UltrasoundData_cm, 0, ObstacleDetection);
  if (1 == is_get)
  {
    if (true == UltrasoundDetectionStatus)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }
  }
  else if (2 == is_get)
  {
    char toString[10];
    sprintf(toString, "%d", UltrasoundData_cm);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  }
}

void ApplicationFunctionSet::CMD_TraceModuleStatus_xxx0(uint8_t is_get)
{
  char toString[10];
  if (0 == is_get)
  {
    sprintf(toString, "%d", TrackingData_L);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif

  }
  else if (1 == is_get)
  {
    sprintf(toString, "%d", TrackingData_M);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif

  }
  else if (2 == is_get)
  {
    sprintf(toString, "%d", TrackingData_R);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif

  }
  Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
}

void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
{
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue)
  {
    temp_keyValue = get_keyValue;
    switch (get_keyValue)
    {
    case  1:

      Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case  2:

      Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
      break;
    case  3:

      Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
      break;
    case  4:

      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    default:

      break;
    }
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_IRrecv(void)
{
  uint8_t IRrecv_button;
  static bool IRrecv_en = false;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_button ))
  {
    IRrecv_en = true;

  }
  if (true == IRrecv_en)
  {
    switch (IRrecv_button)
    {
    case  1:

      Application_SmartRobotCarxxx0.Motion_Control = Forward;
      break;
    case  2:

      Application_SmartRobotCarxxx0.Motion_Control = Backward;
      break;
    case  3:

      Application_SmartRobotCarxxx0.Motion_Control = Left;
      break;
    case  4:

      Application_SmartRobotCarxxx0.Motion_Control = Right;
      break;
    case  5:

      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    case  6:
       Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case  7:
       Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
      break;
    case  8:
       Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
      break;
    case  9:
       if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        if (TrackingDetection_S < 600)
        {
          TrackingDetection_S += 10;
        }
      }

      break;
    case  10:
       if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        TrackingDetection_S = 250;
      }
      break;
    case  11:
       if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        if (TrackingDetection_S > 30)
        {
          TrackingDetection_S -= 10;
        }
      }
      break;

    case  12:
    {

      if (Rocker_CarSpeed < 245) Rocker_CarSpeed += 10;
      else                       Rocker_CarSpeed = 255;
    }
    break;
    case  13:
    {
      Rocker_CarSpeed = 150;
    }
    break;
    case  14:
    {

      if (Rocker_CarSpeed > 60) Rocker_CarSpeed -= 10;
      else                      Rocker_CarSpeed = 50;
    }
    break;
    case  15:
    {

      extern void phoneStandTilt(int);
      phoneStandTilt(+1);
    }
    break;
    case  16:
    {

      extern void phoneStandTilt(int);
      phoneStandTilt(-1);
    }
    break;

    default:
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    }

    if (IRrecv_button < 5)
    {
      Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;
      if (millis() - AppIRrecv.IR_PreMillis > 300)
      {
        IRrecv_en = false;
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        AppIRrecv.IR_PreMillis = millis();
      }
    }
    else
    {
      IRrecv_en = false;
      AppIRrecv.IR_PreMillis = millis();
    }
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_SerialPortDataAnalysis(void)
{
  static String SerialPortData = "";
  uint8_t c = "";
  if (Serial.available() > 0)
  {
    while (c != '}' && Serial.available() > 0)
    {

      c = Serial.read();
      SerialPortData += (char)c;
    }
  }
  if (c == '}')
  {
#if _Test_print
    Serial.println(SerialPortData);
#endif

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, SerialPortData);
    SerialPortData = "";
    if (error)
    {
      Serial.println("error:deserializeJson");
    }
    else if (!error)
    {
      int control_mode_N = doc["N"];
      char *temp = doc["H"];
      CommandSerialNumber = temp;

      switch (control_mode_N)
      {
      case 1:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_MotorControl;
        CMD_is_MotorSelection = doc["D1"];
        CMD_is_MotorSpeed = doc["D2"];
        CMD_is_MotorDirection = doc["D3"];

#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 2:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_CarControl_TimeLimit;
        CMD_is_CarDirection = doc["D1"];
        CMD_is_CarSpeed = doc["D2"];
        CMD_is_CarTimer = doc["T"];
        Application_SmartRobotCarxxx0.CMD_CarControl_Millis = millis();
#if _is_print

#endif
        break;

      case 3:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_CarControl_NoTimeLimit;
        CMD_is_CarDirection = doc["D1"];
        CMD_is_CarSpeed = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 4:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_MotorControl_Speed;
        CMD_is_MotorSpeed_L = doc["D1"];
        CMD_is_MotorSpeed_R = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;
      case 5:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_ServoControl;
        CMD_is_Servo = doc["D1"];
        CMD_is_Servo_angle = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;
      case 7:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_LightingControl_TimeLimit;

        CMD_is_LightingSequence = doc["D1"];
        CMD_is_LightingColorValue_R = doc["D2"];
        CMD_is_LightingColorValue_G = doc["D3"];
        CMD_is_LightingColorValue_B = doc["D4"];
        CMD_is_LightingTimer = doc["T"];
        Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = millis();
#if _is_print

#endif
        break;

      case 8:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_LightingControl_NoTimeLimit;

        CMD_is_LightingSequence = doc["D1"];
        CMD_is_LightingColorValue_R = doc["D2"];
        CMD_is_LightingColorValue_G = doc["D3"];
        CMD_is_LightingColorValue_B = doc["D4"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 21:
        CMD_UltrasoundModuleStatus_xxx0(doc["D1"]);
#if _is_print

#endif
        break;

      case 22:
        CMD_TraceModuleStatus_xxx0(doc["D1"]);
#if _is_print

#endif
        break;

      case 23:
        if (true == Car_LeaveTheGround)
        {
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_false}");
#endif
        }
        else if (false == Car_LeaveTheGround)
        {
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_true}");
#endif
        }
        break;

      case 110:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Programming_mode;
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;
      case 100:
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Standby_mode;
#if _is_print
        Serial.print("{ok}");

#endif
        break;

      case 101:
        if (1 == doc["D1"])
        {
          Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
        }
        else if (2 == doc["D1"])
        {
          Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
        }
        else if (3 == doc["D1"])
        {
          Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
        }

#if _is_print
        Serial.print("{ok}");

#endif
        break;

      case 105:
        if (1 == doc["D1"] && (CMD_is_FastLED_setBrightness < 250))
        {
          CMD_is_FastLED_setBrightness += 5;
        }
        else if (2 == doc["D1"] && (CMD_is_FastLED_setBrightness > 0))
        {
          CMD_is_FastLED_setBrightness -= 5;
        }
        FastLED.setBrightness(CMD_is_FastLED_setBrightness);

#if _Test_print

        Serial.print("{ok}");
#endif
        break;

      case 106:
      {
        uint8_t temp_Set_Servo = doc["D1"];
        if (temp_Set_Servo > 5 || temp_Set_Servo < 1)
          return;
        ApplicationFunctionSet_Servo(temp_Set_Servo);
      }

#if _is_print

        Serial.print("{ok}");
#endif
        break;
      case 102:
        Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;
        Rocker_temp = doc["D1"];
        Rocker_CarSpeed = doc["D2"];

        switch (Rocker_temp)
        {
        case 1:
          Application_SmartRobotCarxxx0.Motion_Control = Forward;
          break;
        case 2:
          Application_SmartRobotCarxxx0.Motion_Control = Backward;
          break;
        case 3:
          Application_SmartRobotCarxxx0.Motion_Control = Left;
          break;
        case 4:
          Application_SmartRobotCarxxx0.Motion_Control = Right;
          break;
        case 5:
          Application_SmartRobotCarxxx0.Motion_Control = LeftForward;
          break;
        case 6:
          Application_SmartRobotCarxxx0.Motion_Control = LeftBackward;
          break;
        case 7:
          Application_SmartRobotCarxxx0.Motion_Control = RightForward;
          break;
        case 8:
          Application_SmartRobotCarxxx0.Motion_Control = RightBackward;
          break;
        case 9:
          Application_SmartRobotCarxxx0.Motion_Control = stop_it;
          Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
          break;
        default:
          Application_SmartRobotCarxxx0.Motion_Control = stop_it;
          break;
        }
#if _is_print

#endif
        break;

      default:
        break;
      }
    }
  }
}
