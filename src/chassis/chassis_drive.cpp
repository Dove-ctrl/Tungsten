#include "chassis.h"

void chassis::TemperatureDrive(){ //温度监测驱动
    average_temperature = 0;
    temperature_list[0] = LFU.temperature(celsius);
    temperature_list[1] = LFD.temperature(celsius);
    temperature_list[2] = LBU.temperature(celsius);
    temperature_list[3] = LBD.temperature(celsius);
    temperature_list[4] = RFU.temperature(celsius);
    temperature_list[5] = RFD.temperature(celsius);
    temperature_list[6] = RBU.temperature(celsius);
    temperature_list[7] = RBD.temperature(celsius);
    for(int i=0 ; i<CHASSIS_MOTOR_NUMBER ; i++){
        average_temperature += temperature_list[i];
        max_temperature = (
            temperature_list[i] > max_temperature ? 
            temperature_list[i] :
            max_temperature
        );
    }
    average_temperature = average_temperature / CHASSIS_MOTOR_NUMBER;
}

//底盘移动驱动，自动时段由程序计算速度向量，决定轮上电压，手动时段由遥控器决定轮上电压
void chassis::MovementDrive(){
    if(eos::COMPETITION->isAutonomous() || eos::AUTODEBUG){

        //nothing to do

    }else if(eos::COMPETITION->isDriverControl() && !eos::AUTODEBUG){

        // 人头操控
        double vx_field = XAxis() * K_OP_X;
        double vy_field = YAxis() * K_OP_Y;
        double omega = AngularAxis() * K_OP_ANGULAR;

        double theta = Odometry.odom_pose.theta;
        double vx = vx_field * cos(theta) - vy_field * sin(theta);
        double vy = vx_field * sin(theta) + vy_field * cos(theta);

        LF_voltage = vx + vy + omega;
        RF_voltage = -vx + vy - omega;
        LB_voltage = -vx + vy + omega;
        RB_voltage = vx + vy - omega;

    }
    
    LFU.spin(fwd , LF_voltage , voltageUnits::mV);
    LFD.spin(fwd , LF_voltage , voltageUnits::mV);

    LBU.spin(fwd , LB_voltage , voltageUnits::mV);
    LBD.spin(fwd , LB_voltage , voltageUnits::mV);

    RFU.spin(fwd , RF_voltage , voltageUnits::mV);
    RFD.spin(fwd , RF_voltage , voltageUnits::mV);

    RBU.spin(fwd , RB_voltage , voltageUnits::mV);
    RBD.spin(fwd , RB_voltage , voltageUnits::mV);
}

void chassis::MovingParametersDrive(){ //运动参数更新驱动

    // 偏航角更新
    yaw = Odometry.odom_pose.theta * RAD_TO_DEG;
    if(yaw > 0){
        yaw = fmod(yaw, 360.0);
    }else if(yaw < 0){
        yaw = 360 - fmod(fabs(yaw), 360.0);
    }else{
        yaw = 0;
    }

    // 速度更新
    LF_velocity = rpm_to_mm_s(
        (
            LFU.velocity(velocityUnits::rpm) +  
            LFD.velocity(velocityUnits::rpm)
        )/(CHASSIS_MOTOR_NUMBER/2) * GEAR_RATIO,
        DRIVE_WHEEL_R
    );
    RF_velocity = rpm_to_mm_s(
        (
            RFU.velocity(velocityUnits::rpm) +  
            RFD.velocity(velocityUnits::rpm)
        )/(CHASSIS_MOTOR_NUMBER/2) * GEAR_RATIO,
        DRIVE_WHEEL_R
    );
    LB_velocity = rpm_to_mm_s(
        (
            LBU.velocity(velocityUnits::rpm) +  
            LBD.velocity(velocityUnits::rpm)
        )/(CHASSIS_MOTOR_NUMBER/2) * GEAR_RATIO,
        DRIVE_WHEEL_R
    );
    RB_velocity = rpm_to_mm_s(
        (
            RBU.velocity(velocityUnits::rpm) +  
            RBD.velocity(velocityUnits::rpm)
        )/(CHASSIS_MOTOR_NUMBER/2) * GEAR_RATIO,
        DRIVE_WHEEL_R
    );
    

    liner_velocity = Odometry.odom_liner_velocity;
}

void odometry::Update(){
    const double dt = UPDATE_TIME * 0.001;   

    double delta_encoderL = 0.0;
    double delta_encoderR = 0.0;
    double prev_encoderL = 0.0;
    double prev_encoderR = 0.0;

    while(true){
        // 传感器数据捕获
        double gyro_angular_velocity = Inertial.gyroRate(axisType::zaxis , velocityUnits::dps) * DEG_TO_RAD;

        delta_encoderL = EncoderL.rotation(rotationUnits::deg) - prev_encoderL;
        delta_encoderR = EncoderR.rotation(rotationUnits::deg) - prev_encoderR;
        prev_encoderL = EncoderL.rotation(rotationUnits::deg);
        prev_encoderR = EncoderR.rotation(rotationUnits::deg);

        double encoderL_velocity = dps_to_mm_s(
            delta_encoderL / dt,
            TRACK_WHEEL_R
        );
        double encoderR_velocity = - dps_to_mm_s(
            delta_encoderR / dt,
            TRACK_WHEEL_R
        );

        // 角度增量计算
        double delta_theta = gyro_angular_velocity * dt;
        if(fabs(delta_theta) <= 0.02 * DEG_TO_RAD){ delta_theta = 0.0; }

        // 本地速度计算
        double vx_local = encoderL_velocity * Sin(MOUNTING_ANGLE) + encoderR_velocity * Cos(MOUNTING_ANGLE);
        double vy_local = encoderL_velocity * Cos(MOUNTING_ANGLE) - encoderR_velocity * Sin(MOUNTING_ANGLE);

        // 正交中心偏移速度补偿
        vx_local = vx_local + gyro_angular_velocity * ODOM_Y_OFFSET;
        vy_local = vy_local - gyro_angular_velocity * ODOM_X_OFFSET;

        odom_liner_velocity = Vec2d{vx_local, vy_local};

        // 速度转换
        double vx_global = vx_local * cos(odom_pose.theta + delta_theta * 0.5) + vy_local * sin(odom_pose.theta + delta_theta * 0.5);
        double vy_global = - vx_local * sin(odom_pose.theta + delta_theta * 0.5) + vy_local * cos(odom_pose.theta + delta_theta * 0.5);

        //位姿更新
        odom_pose.x += vx_global * dt;
        odom_pose.y += vy_global * dt;
        odom_pose.theta += delta_theta;

        wait(UPDATE_TIME,msec);
    }
}