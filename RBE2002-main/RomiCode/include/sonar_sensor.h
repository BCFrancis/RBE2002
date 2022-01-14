#ifndef SONAR_SENSOR
#define SONAR_SENSOR

#include <Romi32U4.h>

class SonarSensor {
   private:
    const int pin_trig = 0,
              pin_echo = 1;

   public:
    void Init();
    float ReadData();
    float PrintData();
};

#endif