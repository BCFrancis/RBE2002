#include "sonar_sensor.h"

/**
 * Initializes pin modes for Sonar sensor
 * */
void SonarSensor::Init() {
    pinMode(pin_trig, OUTPUT);
    pinMode(pin_echo, INPUT);
}
/**
 * Prints and returns current distance read by sonar
 * @return Distance in cm with type float
 * */
float SonarSensor::PrintData() {
    float data = ReadData();
    Serial.println(data);
    return data;
}
/**
 * Calculates and returns the distance currently read by the sonar sensor
 * @return Distance read by sonar sensor in millimeters
 * */
float SonarSensor::ReadData() {
    // Send out sonar Ping for 10 Microseconds
    digitalWrite(pin_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_trig, LOW);
    // record length of pulse recieved by echo pin
    return (pulseIn(pin_echo, HIGH) * 0.018) - 0.3858;
}