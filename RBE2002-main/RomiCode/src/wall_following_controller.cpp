#include "wall_following_controller.h"

void WallFollowingController::Init() {
    SharpIR.Init();
    HCSR04.Init();
}
/**
 * Driving method for Wall Following Controller
 * @return Speed required to move robot to correct distance from wall with type
 * float
 **/
float WallFollowingController::Process(float target_distance) {
    current_time = millis();
    wall_error   = target_distance - SharpIR.ReadData();
    deriv_error  = (wall_error - prev_error) / (current_time - prev_time);
    float speed  = kp * wall_error + Kd * deriv_error;
    prev_error   = wall_error;
    prev_time    = current_time;
    return speed;
}