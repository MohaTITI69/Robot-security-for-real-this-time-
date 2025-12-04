
#include <algorithm>
#include <functional>
#include <vector>
#include <Arduino.h>
#include "esp_system.h"
#include <WiFi.h>
#include "test.h"
#include "aktivitetsplanering.h"





void setup() {
  Serial.begin(115200);

  setUp();

  Serial.println("Setup klar");
}

void loop()
{

}