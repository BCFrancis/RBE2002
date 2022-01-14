#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"
#include "Position_estimation.h"

Romi32U4Motors motors;
Encoder MagneticEncoder; 
Position odometry;

float time_track = 0;

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
    odometry.Init();
}

boolean SpeedController::AccelerationLimit(float target_velocity_left, float target_velocity_right){
float time_interval = 1;
float acc_gain = .05; // doesn't tip
float lefterr_true = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
float righterr_true = target_velocity_right - MagneticEncoder.ReadVelocityRight();
float lefterr = constrain(lefterr_true,-acc_gain,acc_gain);
float righterr = constrain(righterr_true,-acc_gain,acc_gain);
if (millis()>=acceleration_timer)
{
acceleration_timer+=time_interval;
int_left_speed += lefterr;
int_right_speed += righterr;
Run(int_left_speed,int_right_speed);
current_velocity = (int_left_speed+int_right_speed)/2.0;
// acceleration = acc_gain*1000.0;
 Serial.println(current_velocity); //Serial.print(" ");Serial.println(acceleration);
}
return abs(((target_velocity_right+target_velocity_left)/2.0)-current_velocity)<=1.0;
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        time_track = time_track + 50/1000.0;
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();
        
        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;
        // constrain(u_left,0.0,100.0);
        // constrain(u_right,0.0,100.0);
        motors.setEfforts(u_left,u_right);
        odometry.UpdatePose(target_velocity_left,target_velocity_right);
    }
}

boolean SpeedController::MoveToPosition(float target_x, float target_y)
{
    sum_of_error_distance=0;
    sum_of_error_theta=0;
    Serial.println("Moving to new waypoint");
    do
    {   
    error_distance = (sqrt(pow(target_x-odometry.GetX(),2)+pow(target_y-odometry.GetY(),2)));
    error_theta = odometry.GetTheta() - atan2((target_y-odometry.GetY()),(target_x-odometry.GetX()));
    //error_theta = odometry.GetTheta() - atan2(target_y,target_x);
    if (abs(error_theta) >= 3.14159/2)
    {
    error_distance= error_distance*-1;
    }   
    if (error_theta==0)
    {
    Serial.println("Correct Angle");
    sum_of_error_theta=0;
    }
    if (millis()>=goaltime)
    {
    sum_of_error_distance += error_distance;
    sum_of_error_theta += error_theta;
    goaltime+=50;
    }    
    left_velo = (Kpdistance*error_distance)+(Kpangle*error_theta)+(Kidistance*sum_of_error_distance)+(Kiangle*sum_of_error_theta);
    right_velo = (Kpdistance*error_distance)-(Kpangle*error_theta)+(Kidistance*sum_of_error_distance)-(Kiangle*sum_of_error_theta);
    float maxAllowed = 100;
    left_velo = left_velo*gain;
    right_velo = right_velo*gain;
    //constrain(left_velo,-maxAllowed,maxAllowed);
    //constrain(right_velo,-maxAllowed,maxAllowed);
    //Serial.print(odometry.GetTheta()); Serial.print(" "); Serial.println(atan2((target_y-odometry.GetY()),(target_x-odometry.GetX())));
    //Serial.println(error_distance);
    Run(left_velo,right_velo);        
    
    } while (abs(error_distance) >= .01); //define a distance criteria that lets the robot know that it reached the waypoint.
    return 1;
}

boolean SpeedController::Turn(int degree, int direction)
{
    motors.setEfforts(0, 0);
    int turns = counts*(degree/180.0);
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    {
        if(!direction) Run(50,-50);
        else Run(-50,50);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Straight(int target_velocity, int time)
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity,target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop()
{
    motors.setEfforts(0,0);
    odometry.Stop();
    time_track = 0;
}