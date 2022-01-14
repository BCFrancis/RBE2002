#include <Arduino.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "Encoders.h"
#include "IMU.h"
#include "IR_sensor.h"

Behaviors State;
SpeedController robo;
Encoder magEnc;
Romi32U4Motors testmotors;
IMU_sensor accel;
IRsensor IRSen;

void setup() {
  State.Init();
}

void loop() {
State.Run();
// accel.PrintAcceleration();
// delay(20);

}