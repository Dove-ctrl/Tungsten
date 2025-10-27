#pragma once
#include "elementOS_lite.h"
#include "path_planner.h"
#include "structure_control.h"

class odometry{
private:
    //Configuration
    const double MOUNTING_ANGLE = 45.0; //deg
    const double TRACK_WHEEL_R = 2.75 * 25.4 / 2.0; //mm
    const double TRACK_WHEEL_BASE = 203.5; //mm
    const double ODOM_X_OFFSET = 0; //mm
    const double ODOM_Y_OFFSET = -89.45; //mm
    const int UPDATE_TIME = 5; //ms

public:
    Pose odom_pose;
    Vec2d odom_liner_velocity; // mm/s , deg

    odometry(){};

    void Update();
};

class chassis{
private:
    chassis():
        angular_velocity(0.0),
        yaw(0.0)
    {
        
    }
    chassis(const chassis &_chassis) = delete;
    const chassis &operator=(const chassis &_chassis) = delete;

    //Physical configuration
    static const uint32_t CHASSIS_MOTOR_NUMBER = 8;
    const double WHEEL_BASE = 330.2; //mm
    const double DRIVE_WHEEL_R = 2.75 * 25.4 / 2.0; //mm
    const double GEAR_RATIO = 36.0 / 48.0; // 驱动/从动
    const double MAX_LINER_VELOCITY = 2200; // mm/s
    const double MAX_ANGULAR_VELOCITY = 12; // rad/s

    //Controller configuration
    const double MAX_VOLTAGE = 11000; // mV
    const double K_OP_Y = 120;
    const double K_OP_X = 120;
    const double K_OP_ANGULAR = 90;
    int (*YAxis)(void) = eos::A3;
    int (*XAxis)(void) = eos::A4;
    int (*AngularAxis)(void) = eos::A1;
    const double MAX_CURVATURE = 0.7; //曲率达到该值时降速效果最大
    const double MIN_SPEED_FACTOR = 0.5; //最低速度占比，曲率极大时速度不低于基础速度的百分比
    const double LOOKAHEAD_DISTANCE = 300.0; //mm，基础前视距离
    const double LOOKAHEAD_DISTANCE_FACTOR = 0.05; //动态前视距离系数
    const double POSITION_THRESHOLD = 100.0; //mm，终点阈值
    const double SEGMENTS_SLOW_FACTOR = 0.7; //大于插值点数的该百分比就开始减速
    const double SLOW_FACTOR = 0.7; //减速系数
    const double TIMEOUT = 1250;

    //Parameters
    int temperature_list[CHASSIS_MOTOR_NUMBER];
    int max_temperature , average_temperature;

    double LF_velocity , RF_velocity , LB_velocity , RB_velocity; // mm/s

    Vec2d liner_velocity; // mm/s
    double angular_velocity; // rad/s

    double yaw; // degree

    //Controller
    double LF_voltage , RF_voltage , LB_voltage , RB_voltage; // mV

public:
    static chassis& GetInstance(){
        static chassis Chassis;
        return Chassis;
    }

    void ChassisRoute();

    //Odometry
    odometry Odometry;
    
    //Drive
    void MovingParametersDrive();
    void TemperatureDrive();
    void MovementDrive();
    static void ChassisDrive(){
        thread odometry_drive(
            [](){
                chassis::GetInstance().Odometry.Update();
            }
        );
        while(true){
            chassis::GetInstance().TemperatureDrive();
            chassis::GetInstance().MovementDrive();
            chassis::GetInstance().MovingParametersDrive();
            wait(10,msec);
        }
    }

    //Control
    void SetBrake(brakeType btype);
    void SetBrakeType(brakeType btype);

    double ComputeTargetSpeed(double baseSpeed, double curvature);
    void PurePursuit(const std::vector<Point>& controlPoints, int segments , double base_speed);
    void Turn(double direction);

    //Output
    int GetMaxTemperature();
    int GetAverageTemperature();
    double GetYaw();
    Pose GetPose();
    Vec2d GetLinerVelocity();
    double GetAngularVelocity();
    double GetLFVelocity();
    double GetRFVelocity();
    double GetLBVelocity();
    double GetRBVelocity();
    double GetLFVoltage();
    double GetRFVoltage();
    double GetLBVoltage();
    double GetRBVoltage();
    
};