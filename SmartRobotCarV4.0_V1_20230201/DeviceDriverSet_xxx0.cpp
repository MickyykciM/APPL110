
#include "DeviceDriverSet_xxx0.h"

#include <avr/wdt.h>
static void
delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

static uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_xxx(uint16_t Duration, uint8_t Traversal_Number, CRGB colour)
{
  if (NUM_LEDS < Traversal_Number)
  {
    Traversal_Number = NUM_LEDS;
  }
  for (int Number = 0; Number < Traversal_Number; Number++)
  {
    leds[Number] = colour;
    FastLED.show();
    delay_xxx(Duration);
  }
}
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Init(uint8_t set_Brightness)
{
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(set_Brightness);
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Test(void)
{
  leds[0] = CRGB::White;
  FastLED.show();
  delay_xxx(50);
  leds[1] = CRGB::Red;
  FastLED.show();
  delay_xxx(50);
  DeviceDriverSet_RBGLED_xxx(50 , 5 , CRGB::Black);
}
#endif

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Color(uint8_t LED_s, uint8_t r, uint8_t g, uint8_t b)
{
  if (LED_s > NUM_LEDS)
    return;
  if (LED_s == NUM_LEDS)
  {
    FastLED.showColor(Color(r, g, b));
  }
  else
  {
    leds[LED_s] = Color(r, g, b);
  }
  FastLED.show();
}

uint8_t DeviceDriverSet_Key::keyValue = 0;

static void attachPinChangeInterrupt_GetKeyValue(void)
{
  DeviceDriverSet_Key Key;
  static uint32_t keyValue_time = 0;
  static uint8_t keyValue_temp = 0;
  if ((millis() - keyValue_time) > 500)
  {
    keyValue_temp++;
    keyValue_time = millis();
    if (keyValue_temp > keyValue_Max)
    {
      keyValue_temp = 0;
    }
    Key.keyValue = keyValue_temp;
  }
}
void DeviceDriverSet_Key::DeviceDriverSet_Key_Init(void)
{
  pinMode(PIN_Key, INPUT_PULLUP);

  attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Key::DeviceDriverSet_Key_Test(void)
{
  Serial.println(DeviceDriverSet_Key::keyValue);
}
#endif

void DeviceDriverSet_Key::DeviceDriverSet_key_Get(uint8_t *get_keyValue)
{
  *get_keyValue = keyValue;
}

bool DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Init(void)
{
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  return false;
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_L(void)
{
  return analogRead(PIN_ITR20001xxxL);
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_M(void)
{
  return analogRead(PIN_ITR20001xxxM);
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_R(void)
{
  return analogRead(PIN_ITR20001xxxR);
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Test(void)
{
  Serial.print("\tL=");
  Serial.print(analogRead(PIN_ITR20001xxxL));

  Serial.print("\tM=");
  Serial.print(analogRead(PIN_ITR20001xxxM));

  Serial.print("\tR=");
  Serial.println(analogRead(PIN_ITR20001xxxR));
}
#endif

void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Init(void)
{
  pinMode(PIN_Voltage, INPUT);

}
float DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_getAnalogue(void)
{

  float Voltage = (analogRead(PIN_Voltage) * 0.0375);
  Voltage = Voltage + (Voltage * 0.08);

  return Voltage;
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Test(void)
{

  float Voltage = (analogRead(PIN_Voltage) * 0.0375);
  Voltage = Voltage + (Voltage * 0.08);

  Serial.println(Voltage);
}
#endif

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Test(void)
{

  digitalWrite(PIN_Motor_STBY, HIGH);

  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, 100);
  delay_xxx(1000);

  digitalWrite(PIN_Motor_STBY, LOW);
  delay_xxx(1000);
  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 100);

  delay_xxx(1000);
}
#endif

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A,
                                                          boolean direction_B, uint8_t speed_B,
                                                          boolean controlED
                                                          )
{

  {
    uint8_t orig_dirA = direction_A;
    uint8_t orig_dirB = direction_B;
    uint8_t orig_spA  = speed_A;
    uint8_t orig_spB  = speed_B;

    if (orig_dirB == direction_just)      direction_A = direction_back;
    else if (orig_dirB == direction_back) direction_A = direction_just;
    else                                  direction_A = orig_dirB;
    speed_A = orig_spB;

    if (orig_dirA == direction_just)      direction_B = direction_back;
    else if (orig_dirA == direction_back) direction_B = direction_just;
    else                                  direction_B = orig_dirA;
    speed_B = orig_spA;
  }

  if (controlED == control_enable)
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    {

      switch (direction_A)
      {
      case direction_just:
        digitalWrite(PIN_Motor_AIN_1, HIGH);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_back:

        digitalWrite(PIN_Motor_AIN_1, LOW);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }

    {
      switch (direction_B)
      {
      case direction_just:
        digitalWrite(PIN_Motor_BIN_1, HIGH);

        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_back:
        digitalWrite(PIN_Motor_BIN_1, LOW);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }
  }
  else
  {
    digitalWrite(PIN_Motor_STBY, LOW);
    return;
  }
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init(void)
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get )
{
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);

  if (tempda_x > 150)
  {
    *ULTRASONIC_Get = 150;
  }
  else
  {
    *ULTRASONIC_Get = tempda_x;
  }

}

#if _Test_DeviceDriverSet
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Test(void)
{

  unsigned int tempda = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);

  Serial.print("ULTRASONIC=");
  Serial.print(tempda);
  Serial.println("cm");
}

#endif

Servo myservo;
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Init(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z, 500, 2400);
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle);
  delay_xxx(500);

  myservo.attach(PIN_Servo_y, 500, 2400);
  myservo.attach(PIN_Servo_y);
  myservo.write(Position_angle);
  delay_xxx(500);
  myservo.detach();
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Test(void)
{
  for (;;)
  {
    myservo.attach(PIN_Servo_z);
    myservo.write(180);
    delay_xxx(500);
    myservo.write(0);
    delay_xxx(500);
  }

}
#endif

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_control(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle);
  delay_xxx(450);
  myservo.detach();
}

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_controls(uint8_t Servo, unsigned int Position_angle)
{
  if (Servo == 1 || Servo == 3)
  {
    if (Position_angle <= 1)
    {
      Position_angle = 1;
    }
    if (Position_angle >= 17)
    {
      Position_angle = 17;
    }
    myservo.attach(PIN_Servo_z);
    myservo.write(10 * Position_angle);
    delay_xxx(500);
  }
  if (Servo == 2 || Servo == 3)
  {

    if (Position_angle <= 3)
    {
      Position_angle = 3;
    }
    if (Position_angle >= 11)
    {
      Position_angle = 11;
    }
    myservo.attach(PIN_Servo_y);
    myservo.write(10 * Position_angle);
    delay_xxx(500);
  }
  myservo.detach();
}

IRrecv irrecv(RECV_PIN);
decode_results results;
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Init(void)
{
  irrecv.enableIRIn();
}
bool DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get )
{
  if (irrecv.decode(&results))
  {
    IR_PreMillis = millis();
    switch (results.value)
    {
    case  aRECV_upper:
    case  bRECV_upper:
       *IRrecv_Get = 1;
      break;
    case  aRECV_lower:
    case  bRECV_lower:
       *IRrecv_Get = 2;
      break;
    case  aRECV_Left:
    case  bRECV_Left:
       *IRrecv_Get = 3;
      break;
    case  aRECV_right:
    case  bRECV_right:
       *IRrecv_Get = 4;
      break;
    case  aRECV_ok:
    case  bRECV_ok:
       *IRrecv_Get = 5;
      break;

    case  aRECV_1:
    case  bRECV_1:
       *IRrecv_Get = 6;
      break;
    case  aRECV_2:
    case  bRECV_2:
       *IRrecv_Get = 7;
      break;
    case  aRECV_3:
    case  bRECV_3:
       *IRrecv_Get = 8;
      break;
    case  aRECV_4:
    case  bRECV_4:
       *IRrecv_Get = 9;
      break;
    case  aRECV_5:
    case  bRECV_5:
       *IRrecv_Get = 10;
      break;
    case  aRECV_6:
    case  bRECV_6:
       *IRrecv_Get = 11;
      break;
    case  aRECV_7:
    case  bRECV_7:
       *IRrecv_Get = 12;
      break;
    case  aRECV_8:
    case  bRECV_8:
       *IRrecv_Get = 13;
      break;
    case  aRECV_9:
    case  bRECV_9:
       *IRrecv_Get = 14;
      break;
    case  aRECV_star:
    case  bRECV_star:
       *IRrecv_Get = 15;
      break;
    case  aRECV_hash:
    case  bRECV_hash:
       *IRrecv_Get = 16;
      break;
    default:

      irrecv.resume();
      return false;
      break;
    }
    irrecv.resume();
    return true;
  }
  else
  {
    return false;
  }
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Test(void)
{
  if (irrecv.decode(&results))
  {
    Serial.print("IRrecv_Test:");
    Serial.println(results.value);
    irrecv.resume();
  }
}
#endif
