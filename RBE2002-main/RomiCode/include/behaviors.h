#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

#include "speed_controller.h"
#include "wall_following_controller.h"
#include "imu.h"
#include "median_filter.h"
#include "sonar_sensor.h"

class Behaviors {
   private:
    int threshold   = 90,
        drop_thresh = 0,
        data[3]     = {0};
    float goal_time = 0.0;

    enum ROBOT_STATE { IDLE,
                       DRIVE,
                       REVERSE,
                       WAITING,
                       TURN,
                       CENTI,
                       TRACK };
    ROBOT_STATE robot_state = IDLE;  //initial state: IDLE

   public:
    void Init();
    void Stop();
    void Run();
    bool DetectCollision();
    bool DetectDrop();
};
#endif