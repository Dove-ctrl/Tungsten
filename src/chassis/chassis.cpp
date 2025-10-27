#include "chassis.h"

int chassis::GetMaxTemperature(){return max_temperature;}
int chassis::GetAverageTemperature(){return average_temperature;}
double chassis::GetAngularVelocity(){return angular_velocity;}
double chassis::GetYaw(){return yaw;}
Pose chassis::GetPose(){return Odometry.odom_pose;}

double chassis::GetLFVelocity(){return LF_velocity;}
double chassis::GetRFVelocity(){return RF_velocity;}
double chassis::GetLBVelocity(){return LB_velocity;}
double chassis::GetRBVelocity(){return RB_velocity;}
double chassis::GetLFVoltage(){return LF_voltage;}
double chassis::GetRFVoltage(){return RF_voltage;}
double chassis::GetLBVoltage(){return LB_voltage;}
double chassis::GetRBVoltage(){return RB_voltage;}