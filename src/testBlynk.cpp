#define BLYNK_FIRMWARE_VERSION "0.0.1"
#define BLYNK_TEMPLATE_ID "TMPLAtcxMrjL"
#define BLYNK_DEVICE_NAME "Smart Home Mesh"
#define USE_NODE_MCU_BOARD
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

#include <Arduino.h>
#include "BlynkEdgent.h"

void setup()
{
  Serial.begin(115200);
  delay(100);

  BlynkEdgent.begin();
}

void loop() {
  BlynkEdgent.run();
}