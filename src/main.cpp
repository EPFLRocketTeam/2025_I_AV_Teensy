#include <Arduino.h>
// #include <bmp581.h>

// BMP581 barometer;
// bmp5_sensor_data data;

void setup() {
  // put your setup code here, to run once:
   
  // barometer.beginI2C();
}

void loop() {
  // blink the led
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  // print barometer data
 
  // barometer.getSensorData(&data);
  Serial.print("Pressure: ");
  // Serial.print(data.pressure);
  // Serial.print(" Temperature: ");
  // Serial.println(data.temperature);
  // delay(1000);
}