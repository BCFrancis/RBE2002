#ifndef IR_SENSOR
#define IR_SENSOR

#include <Romi32U4.h>

class IRsensor {
   private:
    const int pin_ir            = A0;
    const float ir_voltage_conv = 5.0 / 1024.0;  // Reference voltage over ADC bit value
   public:
    void Init();
    float ReadData();
    float PrintData();
};

#endif