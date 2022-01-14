#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"

IRsensor SharpIR;
SonarSensor HCSR04;
void WallFollowingController::Init(void)
{
    SharpIR.Init();
    HCSR04.Init();
}

float WallFollowingController::Process(float target_distance)
{
  current_time = millis();
  wall_error = target_distance-SharpIR.ReadData();
  deriv_error = (wall_error - prev_error)/(current_time-prev_time);
  float speed = (Kp*wall_error);// + (Kd * deriv_error);
  prev_error = wall_error;
  prev_time = current_time;
  return speed;
}