#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        int threshold = 250;
        int dropThresh = 10;
        int gthreshold = 5000;
        int data[3] = {0};
        enum ROBOT_STATE {IDLE, DRIVE, REVERSE, WAITING, TURN, CENTI,  TRACK};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
        float goalTime=0.0;
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        bool DetectCollision();
        bool DetectDrop();
        bool DetectRot();
};
#endif