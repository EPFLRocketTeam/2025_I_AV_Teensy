#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <utils.h>
#include <iostream>

void setup(void)
{
  Serial.begin(115200); // 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
}

void loop(void)
{ 
  Serial.println("test");
  // blink led
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
