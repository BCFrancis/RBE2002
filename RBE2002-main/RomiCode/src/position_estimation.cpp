#include "position_estimation.h"

Encoder RomiEncoders;

void Position::Init() {
    x         = 0;
    y         = 0;
    theta     = 0;
    time_prev = millis();
}

void Position::Stop() {
    x         = 0;
    y         = 0;
    theta     = 0;
    time_prev = millis();
}

Position::pose_data Position::ReadPose() {
    return {x, y, theta};
}

float Position::GetTheta() {
    return (theta);
}
float Position::GetX() {
    return (x);
}
float Position::GetY() {
    return (y);
}

void Position::PrintPose() {
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(theta);
}

void Position::UpdatePose(float target_velocity_left, float target_velocity_right) {
    time_now = millis();
    if (time_now - time_prev >= 50) {                                      //update every 50ms for practical reasons
        float d_time         = (time_now - time_prev) / 1000.0;            //convert to s
        float velocity_left  = RomiEncoders.ReadVelocityLeft() / 1000.0;   //in m/s
        float velocity_right = RomiEncoders.ReadVelocityRight() / 1000.0;  //in m/s
        float velocity       = (velocity_left + velocity_right) / 2.0;     //in m/s

        float w       = (velocity_right - velocity_left) / (float)l;
        float R       = (l / 2.0) * (velocity_right + velocity_left) / (float)(velocity_right - velocity_left);
        float d_theta = w * d_time;

        if (velocity_left == velocity_right) {  //straight movement
            x     = x + velocity * cos(theta) * d_time;
            y     = y + velocity * sin(theta) * d_time;
            theta = theta + d_theta;
        } else {  //curved movement or point turn
            x = x - R * sin(theta) + R * sin(theta + d_theta);
            y = y + R * cos(theta) - R * cos(theta + d_theta);
            theta += d_theta;
        }
        time_prev = time_now;
    }
}