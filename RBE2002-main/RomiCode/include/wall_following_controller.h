#ifndef WALL_FOLLOWING_CONTROLLER
#define WALL_FOLLOWING_CONTROLLER

#include <Romi32U4.h>

#include "ir_sensor.h"
#include "sonar_sensor.h"

class WallFollowingController {
   private:
    const float kp = 7;  // Adapt parameters kp and Kd until your robot consistently drives along a wall
    const float Kd = 1;

    float e_left          = 0,
          e_right         = 0,
          e_distance      = 0,
          prev_e_distance = 0,
          wall_error      = 0,
          deriv_error     = 0,
          prev_error      = 0,
          current_time    = 0,
          prev_time       = 0;
    IRsensor SharpIR;
    SonarSensor HCSR04;

   public:
    void Init();
    float Process(float);
};

#endif