
#ifndef _CameraWebServer_AP_H
#define _CameraWebServer_AP_H
#include "esp_camera.h"
#include <WiFi.h>

class CameraWebServer_AP
{

public:
  void CameraWebServer_AP_Init(void);
  String wifi_name;

private:

  char *ssid = "APPL-110 Group 5";

  char *password = "";
};

#endif
