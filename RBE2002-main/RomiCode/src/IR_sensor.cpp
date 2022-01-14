#include <Romi32U4.h>
#include "IR_sensor.h"

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
    return ReadData();
}

float IRsensor::ReadData(void)
{
  int adc = analogRead(pin_IR);
  float voltage = (5.0/1024.0)*adc;
  return 1/((voltage-0.2326)/21.317);
}