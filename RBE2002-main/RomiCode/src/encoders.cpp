#include "encoders.h"

#include <Romi32U4.h>

Romi32U4Encoders encoders;

void Encoder::Init() {
    //nothing to initialize, however, good practice to have a init function anyway
}

float Encoder::PrintVelocities() {
    Serial.print("Velocity of left wheel: ");
    Serial.print(ReadVelocityLeft());
    Serial.print('\t');
    Serial.print("Velocity of right wheel: ");
    Serial.println(ReadVelocityRight());
}

int Encoder::ReadEncoderCountLeft() {
    return count_left;
}

int Encoder::ReadEncoderCountRight() {
    return count_right;
}

float Encoder::ReadVelocityLeft() {
    float measurement = (c_wheel / n_wheel) * (count_left - prev_count_left) / ((float)interval / 1000);
    return measurement;
}

float Encoder::ReadVelocityRight() {
    float measurement = (c_wheel / n_wheel) * (count_right - prev_count_right) / ((float)interval / 1000);
    return measurement;
}

boolean Encoder::UpdateEncoderCounts() {
    uint32_t now = millis();
    if (now - last_update >= interval) {
        prev_count_left  = count_left;
        prev_count_right = count_right;
        count_left       = encoders.getCountsLeft();
        count_right      = encoders.getCountsRight();
        previous_time    = millis();
        last_update       = now;
        return true;
    }
    return false;
}