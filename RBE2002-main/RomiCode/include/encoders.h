#ifndef ENCODER
#define ENCODER

#include <Romi32U4.h>

class Encoder {
   private:
    const float n_wheel    = 1440.0;            // counts per wheel revolution
    const float r_wheel    = 35.0;              // radius of wheel in [mm]
    const float c_wheel    = 2 * PI * r_wheel;  // circumference of wheel
    const uint8_t interval = 50;                // time in [ms], how often encoders are being updated
    int count_left         = 0,
        count_right        = 0,
        prev_count_left    = 0,
        prev_count_right   = 0;
    float previous_time    = 0;
    uint32_t last_update   = 0;
    Romi32U4Encoders encoders;

   public:
    void Init();
    int ReadEncoderCountLeft();
    int ReadEncoderCountRight();
    float ReadVelocityLeft();
    float ReadVelocityRight();
    float PrintVelocities();
    boolean UpdateEncoderCounts();
};

#endif