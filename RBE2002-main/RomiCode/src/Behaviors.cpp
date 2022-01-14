#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "IMU.h"
#include "Median_filter.h"
#include "Wall_following_controller.h"
#include "Sonar_sensor.h"

//sensors
WallFollowingController Tracker;
Romi32U4ButtonA buttonA;
IMU_sensor LSM6;
SonarSensor sonar;
//motor-speed controller
SpeedController robot;
//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

void Behaviors::Init(void)
{
    sonar.Init();
    robot.Init();
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    Tracker.Init();
}

void Behaviors::Stop(void)
{
    robot.Stop();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

boolean Behaviors::DetectDrop()
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if(data[2] < dropThresh) {
        return 1;
    }
    else{
        return 0;
    }
}

boolean Behaviors::DetectRot(){
    auto data_gyr = LSM6.RGyro();
    data[0] = med_x.Filter(data_gyr.xg);
    data[1] = med_y.Filter(data_gyr.yg);
    data[2] = med_z.Filter(data_gyr.xg);
    if ((abs(data[1]) > gthreshold))
    {
        return 1;
    }
    else
    {
        return 0;
    }
    
    

}


void Behaviors::Run(void)
{
    switch (robot_state)
    {
    case IDLE:
        if(buttonA.getSingleDebouncedRelease()){ 
            robot.Stop();
            robot_state = DRIVE; 
            delay(500);
            goalTime = millis() + 1000;
        } 
        else { 
            robot_state = IDLE;
            //LSM6.PrintAcceleration();
            robot.Stop(); 
        }   
        break;
    
    case DRIVE:
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = IDLE; 
            robot.Stop();             
        } 
        else {
            robot.Run(70,70);
            if (DetectCollision() && millis()>goalTime)
            {
            robot.Stop();
            robot_state = WAITING;
            }
            
            
        }
        break;
    
    case WAITING:
        if(buttonA.getSingleDebouncedRelease())
        {
        goalTime = millis()+2000;
        robot_state = REVERSE;     
        // Serial.println(goalTime);
        // Serial.println(millis());

        }
        break;

    case REVERSE:
        //Serial.println("RESVERSE");
        while (millis()<goalTime)
        {
            robot.Run(-50,-50);
        }
        Serial.println(millis());
        Serial.println("TURN");
        goalTime = millis()+4200;
        robot_state = TURN;
        break;

    case TURN:
        while (millis()<goalTime)
        {
        robot.Run(-30,30);
        }
        robot.Stop();
        robot_state = TRACK;
        Serial.println("TRACK");
        break;

    case CENTI:
        //Serial.println("TEN_CENTIMETERS");
        robot.Run(50,50);
        if (millis()>=goalTime)
        {
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
        float speedRAW = Tracker.Process(20); //distance in [cm]
        float speed = constrain(speedRAW,-50.0,50.0);
        if (millis()>goalTime)
        {
         LSM6.PrintGyro();
        // Serial.println(speedRAW);
         goalTime += 100;
        }
        robot.Run(50.0-speed,50.0+speed);
        if(DetectRot()){
            Serial.println("Dropped");
            robot_state = CENTI;
            goalTime = millis() + 8000;
            while (millis()<goalTime)
            {
            Serial.println("Stalling");
            float speedRAW = Tracker.Process(12); //distance in [cm]
            float speed = constrain(speedRAW,-50.0,50.0);
            robot.Run(50.0-speed,50.0+speed);
            }
            Serial.println("Ending");
            robot.Stop();
            goalTime = millis() + 2250;
         }
        break;

    };}