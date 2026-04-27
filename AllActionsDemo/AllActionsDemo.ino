
#include <avr/wdt.h>
#include <Servo.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"
#include "MPU6050_getdata.h"
#include "IRremote.h"

extern DeviceDriverSet_Motor       AppMotor;
extern DeviceDriverSet_Servo       AppServo;
extern DeviceDriverSet_ULTRASONIC  AppULTRASONIC;
extern MPU6050_getdata             AppMPU6050getdata;
extern ApplicationFunctionSet      Application_FunctionSet;

#define PIN_LINE_L  A2
#define PIN_LINE_M  A1
#define PIN_LINE_R  A0

#define PHONE_SERVO_PIN   11
#define TILT_STEP         5
#define TILT_MIN          0
#define TILT_MAX          180
Servo phoneServo;
int   tiltAngle = 90;

#define IR_RECV_PIN       9
IRrecv         irrecv(IR_RECV_PIN);
decode_results irResults;

#define IR_BTN_7_A   16716015UL
#define IR_BTN_7_B   2351064443UL
#define IR_BTN_9_A   16734885UL
#define IR_BTN_9_B   71952287UL

int           currentDemo    = 0;
unsigned long demoStartTime  = 0;
const int     DEMO_DURATION  = 5000;

void setup() {
  Serial.begin(9600);
  Serial.println(F("ELEGOO Smart Robot Car - All Actions Demo"));

  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppMPU6050getdata.MPU6050_dveInit();

  phoneServo.attach(PHONE_SERVO_PIN);
  phoneServo.write(tiltAngle);

  irrecv.enableIRIn();

  delay(2000);
  demoStartTime = millis();
}

void handlePhoneStandRemote() {
  if (!irrecv.decode(&irResults)) return;

  unsigned long code = irResults.value;

  if (code == IR_BTN_7_A || code == IR_BTN_7_B) {
    tiltAngle -= TILT_STEP;
    if (tiltAngle < TILT_MIN) tiltAngle = TILT_MIN;
    phoneServo.write(tiltAngle);
    Serial.print(F("[Remote 7] Tilt -> ")); Serial.println(tiltAngle);
  }
  else if (code == IR_BTN_9_A || code == IR_BTN_9_B) {
    tiltAngle += TILT_STEP;
    if (tiltAngle > TILT_MAX) tiltAngle = TILT_MAX;
    phoneServo.write(tiltAngle);
    Serial.print(F("[Remote 9] Tilt -> ")); Serial.println(tiltAngle);
  }

  irrecv.resume();
}

void loop() {

  handlePhoneStandRemote();

  unsigned long now = millis();

  if (now - demoStartTime >= DEMO_DURATION) {
    currentDemo++;
    demoStartTime = now;
    AppMotor.DeviceDriverSet_Motor_control(0, 0, 0, 0, 0x01);
    delay(500);
  }

  switch (currentDemo) {
    case 0: demo1_BasicMovement();        break;
    case 1: demo2_AdvancedMovement();     break;
    case 2: demo3_ServoControl();         break;
    case 3: demo4_UltrasonicSensor();     break;
    case 4: demo5_LineTrackingSensor();   break;
    case 5: demo6_IMUSensor();            break;
    case 6: demo7_ObstacleAvoidance();    break;
    case 7: demo8_FollowMode();           break;
    default:
      Serial.println(F("All demos complete. Restarting..."));
      currentDemo = 0;
      delay(2000);
      break;
  }
}

void demo1_BasicMovement() {
  static bool initialized = false;
  static unsigned long stepTime = 0;
  static int step = 0;

  if (!initialized) {
    Serial.println(F("\n[Demo 1] Basic Movement: Forward -> Backward -> Stop"));
    initialized = true;
    stepTime = millis();
    step = 0;
  }

  if (millis() - stepTime >= 1500) {
    step++;
    stepTime = millis();
    switch (step) {
      case 0:
        Serial.println(F("Forward"));
        AppMotor.DeviceDriverSet_Motor_control(1, 200, 1, 200, 0x01);
        break;
      case 1:
        Serial.println(F("Backward"));
        AppMotor.DeviceDriverSet_Motor_control(0, 200, 0, 200, 0x01);
        break;
      case 2:
        Serial.println(F("Stop"));
        AppMotor.DeviceDriverSet_Motor_control(0, 0, 0, 0, 0x01);
        initialized = false;
        break;
    }
  }
}

void demo2_AdvancedMovement() {
  static bool initialized = false;
  static unsigned long stepTime = 0;
  static int step = 0;

  if (!initialized) {
    Serial.println(F("\n[Demo 2] Turns and Spins"));
    initialized = true;
    stepTime = millis();
    step = 0;
  }

  if (millis() - stepTime >= 1000) {
    step++;
    stepTime = millis();
    switch (step) {
      case 0:
        Serial.println(F("Turn Left"));
        AppMotor.DeviceDriverSet_Motor_control(1, 100, 1, 200, 0x01);
        break;
      case 1:
        Serial.println(F("Turn Right"));
        AppMotor.DeviceDriverSet_Motor_control(1, 200, 1, 100, 0x01);
        break;
      case 2:
        Serial.println(F("Spin Left"));
        AppMotor.DeviceDriverSet_Motor_control(0, 180, 1, 180, 0x01);
        break;
      case 3:
        Serial.println(F("Spin Right"));
        AppMotor.DeviceDriverSet_Motor_control(1, 180, 0, 180, 0x01);
        break;
      case 4:
        Serial.println(F("Stop"));
        AppMotor.DeviceDriverSet_Motor_control(0, 0, 0, 0, 0x01);
        initialized = false;
        break;
    }
  }
}

void demo3_ServoControl() {
  static bool initialized = false;
  static int angle = 90;
  static int dir = 1;

  if (!initialized) {
    Serial.println(F("\n[Demo 3] Servo Scan"));
    initialized = true;
    angle = 90;
    dir = 1;
  }

  AppServo.DeviceDriverSet_Servo_control(angle);
  angle += dir * 10;
  if (angle >= 180) { angle = 180; dir = -1; }
  else if (angle <= 0) { angle = 0; dir = 1; }
  delay(100);

  if (millis() - demoStartTime >= DEMO_DURATION - 500) {
    AppServo.DeviceDriverSet_Servo_control(90);
    initialized = false;
  }
}

void demo4_UltrasonicSensor() {
  static bool initialized = false;
  static unsigned long lastPrint = 0;

  if (!initialized) {
    Serial.println(F("\n[Demo 4] Ultrasonic Distance"));
    initialized = true;
    lastPrint = millis();
  }

  if (millis() - lastPrint >= 500) {
    uint16_t distance = 0;
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance);
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));
    lastPrint = millis();
  }

  if (millis() - demoStartTime >= DEMO_DURATION - 100) initialized = false;
}

void demo5_LineTrackingSensor() {
  static bool initialized = false;
  static unsigned long lastPrint = 0;

  if (!initialized) {
    Serial.println(F("\n[Demo 5] IR Line Sensors (L/M/R)"));
    initialized = true;
    lastPrint = millis();
  }

  if (millis() - lastPrint >= 500) {
    int l = analogRead(PIN_LINE_L);
    int m = analogRead(PIN_LINE_M);
    int r = analogRead(PIN_LINE_R);
    Serial.print(F("L=")); Serial.print(l);
    Serial.print(F(" M=")); Serial.print(m);
    Serial.print(F(" R=")); Serial.println(r);
    lastPrint = millis();
  }

  if (millis() - demoStartTime >= DEMO_DURATION - 100) initialized = false;
}

void demo6_IMUSensor() {
  static bool initialized = false;
  static bool calibrated = false;
  static unsigned long lastPrint = 0;

  if (!initialized) {
    Serial.println(F("\n[Demo 6] IMU - calibrating..."));
    initialized = true;
    calibrated = false;
    lastPrint = millis();
  }

  if (!calibrated) {
    AppMPU6050getdata.MPU6050_calibration();
    calibrated = true;
    Serial.println(F("IMU calibrated."));
  }

  if (millis() - lastPrint >= 500) {
    float yaw;
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&yaw);
    Serial.print(F("Yaw: ")); Serial.println(yaw);
    lastPrint = millis();
  }

  if (millis() - demoStartTime >= DEMO_DURATION - 100) initialized = false;
}

void demo7_ObstacleAvoidance() {
  static bool initialized = false;
  static unsigned long lastCheck = 0;

  if (!initialized) {
    Serial.println(F("\n[Demo 7] Obstacle Avoidance"));
    AppServo.DeviceDriverSet_Servo_control(90);
    initialized = true;
    lastCheck = 0;
  }

  if (millis() - lastCheck >= 200) {
    lastCheck = millis();
    uint16_t d = 0;
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&d);

    if (d == 0 || d > 25) {

      AppMotor.DeviceDriverSet_Motor_control(1, 180, 1, 180, 0x01);
    } else if (d > 12) {

      AppMotor.DeviceDriverSet_Motor_control(1, 180, 0, 180, 0x01);
    } else {

      AppMotor.DeviceDriverSet_Motor_control(0, 200, 0, 200, 0x01);
    }
  }

  if (millis() - demoStartTime >= DEMO_DURATION - 100) {
    AppMotor.DeviceDriverSet_Motor_control(0, 0, 0, 0, 0x01);
    initialized = false;
  }
}

void demo8_FollowMode() {
  static bool initialized = false;

  if (!initialized) {
    Serial.println(F("\n[Demo 8] Follow Mode"));
    Application_FunctionSet.ApplicationFunctionSet_Init();
    initialized = true;
  }

  Application_FunctionSet.ApplicationFunctionSet_Follow();

  if (millis() - demoStartTime >= DEMO_DURATION - 100) {
    AppMotor.DeviceDriverSet_Motor_control(0, 0, 0, 0, 0x01);
    initialized = false;
  }
}
