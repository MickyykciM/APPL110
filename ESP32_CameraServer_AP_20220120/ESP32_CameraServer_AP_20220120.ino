
#include "CameraWebServer_AP.h"
#include <WiFi.h>
#include "esp_camera.h"
WiFiServer server(100);

#define RXD2 33
#define TXD2 4
CameraWebServer_AP CameraWebServerAP;

bool WA_en = false;

void SocketServer_Test(void)
{
  static bool ED_client = true;
  WiFiClient client = server.available();
  if (client)
  {
    WA_en = true;
    ED_client = true;
    Serial.println("[Client connected]");
    String readBuff;
    String sendBuff;
    uint8_t Heartbeat_count = 0;
    bool Heartbeat_status = false;
    bool data_begin = true;
    while (client.connected())
    {
      if (client.available())
      {
        char c = client.read();
        Serial.print(c);
        if (true == data_begin && c == '{')
        {
          data_begin = false;
        }
        if (false == data_begin && c != ' ')
        {
          readBuff += c;
        }
        if (false == data_begin && c == '}')
        {
          data_begin = true;
          if (true == readBuff.equals("{Heartbeat}"))
          {
            Heartbeat_status = true;
          }
          else
          {
            Serial2.print(readBuff);
          }

          readBuff = "";
        }
      }
      if (Serial2.available())
      {
        char c = Serial2.read();
        sendBuff += c;
        if (c == '}')
        {
          client.print(sendBuff);
          Serial.print(sendBuff);
          sendBuff = "";
        }
      }

      static unsigned long Heartbeat_time = 0;
      if (millis() - Heartbeat_time > 1000)
      {
        client.print("{Heartbeat}");
        if (true == Heartbeat_status)
        {
          Heartbeat_status = false;
          Heartbeat_count = 0;
        }
        else if (false == Heartbeat_status)
        {
          Heartbeat_count += 1;
        }
        if (Heartbeat_count > 3)
        {
          Heartbeat_count = 0;
          Heartbeat_status = false;
          break;
        }
        Heartbeat_time = millis();
      }
      static unsigned long Test_time = 0;
      if (millis() - Test_time > 1000)
      {
        Test_time = millis();

        if (0 == (WiFi.softAPgetStationNum()))
        {
          Serial2.print("{\"N\":100}");
          break;
        }
      }
    }
    Serial2.print("{\"N\":100}");
    client.stop();
    Serial.println("[Client disconnected]");
  }
  else
  {
    if (ED_client == true)
    {
      ED_client = false;
      Serial2.print("{\"N\":100}");
    }
  }
}

void FactoryTest(void)
{
  static String readBuff;
  String sendBuff;
  if (Serial2.available())
  {
    char c = Serial2.read();
    readBuff += c;
    if (c == '}')
    {
      if (true == readBuff.equals("{BT_detection}"))
      {
        Serial2.print("{BT_OK}");
        Serial.println("Factory...");
      }
      else if (true == readBuff.equals("{WA_detection}"))
      {
        Serial2.print("{");
        Serial2.print(CameraWebServerAP.wifi_name);
        Serial2.print("}");
        Serial.println("Factory...");
      }
      readBuff = "";
    }
  }
  {
    if ((WiFi.softAPgetStationNum()))
    {
      if (true == WA_en)
      {
        digitalWrite(13, LOW);
        Serial2.print("{WA_OK}");
        WA_en = false;
      }
    }
    else
    {

      static unsigned long Test_time;
      static bool en = true;
      if (millis() - Test_time > 100)
      {
        if (false == WA_en)
        {
          Serial2.print("{WA_NO}");
          WA_en = true;
        }
        if (en == true)
        {
          en = false;
          digitalWrite(13, HIGH);
        }
        else
        {
          en = true;
          digitalWrite(13, LOW);
        }
        Test_time = millis();
      }
    }
  }
}
void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  CameraWebServerAP.CameraWebServer_AP_Init();
  server.begin();
  delay(100);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.println("Elegoo-2020...");
  Serial2.print("{Factory}");
}
void loop()
{
  SocketServer_Test();
  FactoryTest();
}
