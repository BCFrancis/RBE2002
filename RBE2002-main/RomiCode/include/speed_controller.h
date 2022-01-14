#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>

#include "encoders.h"
#include "position_estimation.h"

class SpeedController {
   private:
    // Adapt the parameters until your robot moves at the speed you command it to drive
    const float kp              = 0.5,
                ki              = 0.1,
                kp_distance     = 300,
                ki_distance     = 1.5,
                kp_angle        = 150,
                ki_angle        = .13,
                gain            = 1;
    float e_left                = 0,
          e_right               = 0,
          goal_time             = 0,
          error_distance        = 0,
          error_theta           = 0,
          left_velo             = 0,
          right_velo            = 0,
          sum_of_error_theta    = 0,
          sum_of_error_distance = 0,
          acceleration_timer    = 0,
          int_left_speed        = 0,
          int_right_speed       = 0,
          current_velocity      = 0,
          prev_velocity         = 0,
          acceleration          = 0,
          time_track            = 0;
    int counts                  = 1450;

    Romi32U4Motors motors;
    Encoder MagneticEncoder;

   public:
    struct constrained_acceleration {
        float constrained_velocity_left,
            constrained_velocity_right;
    };
    void Init(void);
    void Run(float, float);
    bool Turn(int, int);  // degrees, direction of rotation: 0->left, 1->right
    bool Straight(int, int);
    bool Curved(int, int, int);
    bool MoveToPosition(float, float);
    void Stop(void);
    bool AccelerationLimit(float, float);
};

#endif