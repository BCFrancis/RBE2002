#ifndef POSITION_ESTIMATION
#define POSITION_ESTIMATION

#include <Romi32U4.h>

#include "encoders.h"

class Position {
   private:
    float x     = 0,
          y     = 0,
          theta = 0;
    unsigned long time_prev, time_now = 0;
    const float l = 0.142875;  //distance from wheel-to-wheel in m
   public:
    struct pose_data {
        float X;
        float Y;
        float THETA;
    };

    void Init();
    void UpdatePose(float, float);
    pose_data ReadPose();
    void PrintPose();
    void Stop();
    float GetX();
    float GetY();
    float GetTheta();
};

#endif