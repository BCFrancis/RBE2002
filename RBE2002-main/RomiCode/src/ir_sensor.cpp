#include "ir_sensor.h"

/**
 * Initializes pin modes for IR sensor
 * */
void IRsensor::Init() {
    pinMode(pin_ir, INPUT);
}

/**
 * Prints and returns current distance read by IR sensor
 * @return Distance in cm with type float
 * */
float IRsensor::PrintData() {
    float data = ReadData();
    Serial.println(1 / data);
    return data;
}

/**
 * Calculates and returns the current distance read by IR sensor
 * @return value corresponding to calculated IR distance with type float
 * */
float IRsensor::ReadData() {
    int adc       = analogRead(pin_ir);     // outputted ADC value of IRSensor
    float voltage = ir_voltage_conv * adc;  // Calculates voltage returned by IR sensor using VREF/2^10 * adc
    return 1 / ((voltage - 0.2326) / 21.317);
}