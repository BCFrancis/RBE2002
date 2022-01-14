#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>

class SpeedController{
    private:
        const float Kp = .5;
        const float Ki = .1;
        const float Kpdistance = 300; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Kidistance = 1.5; 
        const float Kpangle = 150; 
        const float Kiangle = .13; 
        const float gain = 1;
        float E_left = 0; 
        float E_right = 0;
        float goaltime = 0;
        int counts = 1450; //number of counts for a 180 degree turn; you will likely have to change this
        float error_distance = 0;
        float error_theta = 0;
        float left_velo= 0;
        float right_velo= 0;
        float sum_of_error_theta = 0;
        float sum_of_error_distance = 0;
        float acceleration_timer = 0;
        float int_left_speed = 0;
        float int_right_speed = 0;
        float current_velocity = 0;
        float old_velocity = 0;
        float acceleration = 0;
    public:
        struct constrained_acceleration {
            float constrained_velocity_left;
            float constrained_velocity_right;

        };
        void Init(void);
        void Run(float, float); 
        boolean Turn(int,int); //degrees, direction of rotation: 0->left, 1->right
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        boolean MoveToPosition(float,float); //target_x, target_y
        void Stop(void);
        boolean AccelerationLimit(float, float);
};

#endif