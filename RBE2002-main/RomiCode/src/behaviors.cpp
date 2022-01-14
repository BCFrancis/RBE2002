#include "behaviors.h"

#include <Romi32U4.h>

//sensors
Romi32U4ButtonA buttonA;
IMU_sensor LSM6;
SonarSensor sonar;

// median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

// controllers
SpeedController robot;
WallFollowingController tracker;
/**
 * @brief Initializes all sensors, controllers, and median filters
 * 
 */
void Behaviors::Init() {
    robot.Init();
    sonar.Init();
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    tracker.Init();
}
/**
 * @brief Stops all robot behaviors
 * 
 */
void Behaviors::Stop() {
    robot.Stop();
}
/**
 * @brief Utilizes acceleration values in order to detect a collision with an object
 * 
 * @return boolean based on whether x or y acceleration is greater than the defined threashold
 */
bool Behaviors::DetectCollision() {
    auto data_acc = LSM6.ReadAcceleration();
    data[0]       = med_x.Filter(data_acc.X) * 0.061;
    data[1]       = med_y.Filter(data_acc.Y) * 0.061;
    data[2]       = med_z.Filter(data_acc.Z) * 0.061;

    return (abs(data[0]) > threshold) || (abs(data[1]) > threshold);
}
/**
 * @brief Utiliizes Z acceleration values in order to detect a drop
 * 
 * @return boolean based on whether or not the Z acceleration is below the defined threshold
 */
bool Behaviors::DetectDrop() {
    auto data_acc = LSM6.ReadAcceleration();
    data[0]       = med_x.Filter(data_acc.X) * 0.061;
    data[1]       = med_y.Filter(data_acc.Y) * 0.061;
    data[2]       = med_z.Filter(data_acc.Z) * 0.061;

    return data[2] < drop_thresh;
}
/**
 * @brief Main state machine to control robot for final project
 * 
 */
void Behaviors::Run() {
    switch (robot_state) {
        case IDLE:
            if (buttonA.getSingleDebouncedRelease()) {
                robot.Stop();
                robot_state = DRIVE;
                delay(500);
                goal_time = millis() + 500;
            } else {
                robot_state = IDLE;
                //LSM6.PrintAcceleration();
                robot.Stop();
            }
            break;

        case DRIVE:
            if (buttonA.getSingleDebouncedRelease()) {
                robot_state = IDLE;
                robot.Stop();
            } else {
                robot.Run(70, 70);
                if (DetectCollision() && millis() > goal_time) {
                    robot.Stop();
                    robot_state = WAITING;
                }
            }
            break;

        case WAITING:
            if (buttonA.getSingleDebouncedRelease()) {
                goal_time   = millis() + 3000;
                robot_state = REVERSE;
                // Serial.println(goalTime);
                // Serial.println(millis());
            }
            break;

        case REVERSE:
            //Serial.println("RESVERSE");
            while (millis() < goal_time) {
                robot.Run(-50, -50);
            }
            Serial.println(millis());
            Serial.println("TURN");
            goal_time   = millis() + 4200;
            robot_state = TURN;
            break;

        case TURN:
            while (millis() < goal_time) {
                robot.Run(-30, 30);
            }
            robot.Stop();
            robot_state = TRACK;
            Serial.println("TRACK");
            break;

        case CENTI:
            //Serial.println("TEN_CENTIMETERS");
            robot.Run(50, 50);
            if (millis() >= goal_time) {
                robot_state = IDLE;
                Serial.println("IDLE");
            }
            // if(buttonA.getSingleDebouncedRelease())
            // {
            //  robot_state = IDLE;
            // Serial.println("IDLE");
            // }
            break;

        case TRACK:
            float speedRAW = tracker.Process(15);  //distance in [cm]
            float speed    = constrain(speedRAW, -50.0, 50.0);
            if (millis() > goal_time) {
                Serial.println(speedRAW);
                goal_time += 500;
            }
            robot.Run(50.0 - speed, 50.0 + speed);
            if (DetectDrop()) {
                Serial.println("Dropped");
                robot_state = CENTI;
                robot.Stop();
                goal_time = millis() + 2250;
            }
            break;
    };
}