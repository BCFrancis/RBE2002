#include <Romi32U4.h>
#include "Sonar_sensor.h"

void SonarSensor::Init(void)
{
    pinMode(pin_TRIG,OUTPUT);
    pinMode(pin_ECHO, INPUT);   
}

float SonarSensor::PrintData(void)
{
    Serial.println(ReadData());
    return ReadData();
}

float SonarSensor::ReadData(void)
{
    digitalWrite(pin_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_TRIG, LOW);
    float pulse = pulseIn(pin_ECHO, HIGH);
    return (pulse*0.018)-0.3858;
}