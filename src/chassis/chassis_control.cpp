#include "chassis.h"

void chassis::SetBrake(brakeType btype){
    LF_velocity = RF_velocity = LB_velocity = RB_velocity = 0.0;
    LFU.stop(btype);
    LFD.stop(btype);
    LBU.stop(btype);
    LBD.stop(btype);
    RFU.stop(btype);
    RFD.stop(btype);
    RBU.stop(btype);
    RBD.stop(btype);
}

void chassis::SetBrakeType(brakeType btype) {
    LFU.setStopping(btype);
    LFD.setStopping(btype);
    LBU.setStopping(btype);
    LBD.setStopping(btype);
    RFU.setStopping(btype);
    RFD.setStopping(btype);
    RBU.setStopping(btype);
    RBD.setStopping(btype);
}

double chassis::ComputeTargetSpeed(double baseSpeed, double curvature) {
    double factor = 1.0 - std::min(std::fabs(curvature) / MAX_CURVATURE, 1.0) * (1.0 - MIN_SPEED_FACTOR);
    return baseSpeed * factor;
}

void chassis::Turn(double direction){
    /* double total_time = eos::SystemTime(msec);
    double delta_direction = 0.0;
    eos::TerminalPrint("# Turn:" , WHITE);
    eos::TerminalPrintWithData("   target_yaw(deg) = " , fabs(direction) , BLUE);
    eos::TerminalPrintWithData("   start_yaw(deg) = " , yaw , YELLOW);

    if(direction >= 0){
        delta_direction = (fabs(direction) > yaw ? (fabs(direction) - yaw) : (360 + fabs(direction) - yaw));
    }else{
        delta_direction = (fabs(direction) < yaw ? (yaw - fabs(direction)) : (360 + yaw - fabs(direction)));
    }

    double target_theta = robot_pose.theta * RAD_TO_DEG + sign(direction) * delta_direction;
    
    turn_pid.set_target(target_theta);
    while(!turn_pid.is_arrived() && (eos::SystemTime(msec) - total_time) <= TIMEOUT){
        turn_pid.update(robot_pose.theta * RAD_TO_DEG);
        left_voltage = max_value_limit(MAX_VOLTAGE , turn_pid.get_output());
        right_voltage = -max_value_limit(MAX_VOLTAGE , turn_pid.get_output());
        wait(50,msec);
    }
    SetBrake(brake);
    turn_pid.reset();
    total_time = eos::SystemTime(msec) - total_time;
    eos::TerminalPrintWithData("   end_yaw(deg) = " , yaw , YELLOW);
    eos::TerminalPrintWithData("   time(msec) = " , total_time , MAGENTA);
    eos::TerminalEndl();
    bb7_status = true; */
}

void chassis::PurePursuit(const std::vector<Point>& controlPoints, int segments , double base_speed) {
    /* std::vector<Point> path = PathPlan(controlPoints, segments);
    size_t target_index = 0;
    double total_time = eos::SystemTime(msec);
    double max_speed = 0.0;
    eos::TerminalPrint("# PurePursuit:" , WHITE);
    eos::TerminalPrintWithData("   target_speed(mm/s) = " , base_speed , GREEN);
    eos::TerminalPrintWithData("   start_yaw(deg) = " , yaw , YELLOW);
    eos::TerminalPrintWithData("   start_point_x = " , robot_pose.x , BLUE);
    eos::TerminalPrintWithData("   start_point_y = " , robot_pose.y , BLUE);

    while(true) {
        max_speed = sign(liner_velocity) * std::max(max_speed, fabs(liner_velocity));

        double robot_x = robot_pose.x;
        double robot_y = robot_pose.y;
        double robot_theta = robot_pose.theta;

        double dynamic_lookahead = LOOKAHEAD_DISTANCE + LOOKAHEAD_DISTANCE_FACTOR * liner_velocity;
        
        //检查是否到达终点
        double dist_to_end = point_distance(path.back(), {robot_x, robot_y});
        if(dist_to_end < POSITION_THRESHOLD) {
            chassis::GetInstance().SetBrake(brake);
            break;
        }

        //保证目标点向前更新
        while (target_index < path.size()-1) {
            double dist_to_target = point_distance(path[target_index], {robot_x, robot_y});
            if (dist_to_target <= dynamic_lookahead) {
                target_index++;
            } else {
                break;
            }
        }
        Point target_point = path[target_index];

        //计算全局差值
        double dx = target_point.x - robot_x;
        double dy = target_point.y - robot_y;

        //计算曲率
        double local_x = dx * cos(robot_theta) - dy * sin(robot_theta);
        double local_y = dx * sin(robot_theta) + dy * cos(robot_theta);
        double L = point_distance({0, 0}, {local_x, local_y});
        double curvature = (L >= 1e-5 ? 2.0 * local_x / (L * L) : 0.0);

        //生成左右速度目标值
        double target_speed = ComputeTargetSpeed(base_speed, curvature);
        if(target_index + 1 >= segments * SEGMENTS_SLOW_FACTOR){
            target_speed = target_speed * SLOW_FACTOR;
        }
        double omega = target_speed * curvature;
        double leftTargetSpeed = target_speed + (omega * WHEEL_BASE / 2.0);
        double rightTargetSpeed = target_speed - (omega * WHEEL_BASE / 2.0);

        //计算输出
        left_v_pid.set_target(leftTargetSpeed);
        right_v_pid.set_target(rightTargetSpeed);
        left_v_pid.update(left_velocity);
        right_v_pid.update(right_velocity);
        left_voltage = left_v_pid.get_output();
        right_voltage = right_v_pid.get_output();

        wait(10,msec);
    }

    chassis::GetInstance().SetBrake(brake);
    total_time = eos::SystemTime(msec) - total_time;
    eos::TerminalPrintWithData("   max_speed(mm/s) = " , max_speed , GREEN);
    eos::TerminalPrintWithData("   end_yaw(deg) = " , yaw , YELLOW);
    eos::TerminalPrintWithData("   end_point_x = " , robot_pose.x , BLUE);
    eos::TerminalPrintWithData("   end_point_y = " , robot_pose.y , BLUE);
    eos::TerminalPrintWithData("   time(msec) = " , total_time , MAGENTA);
    eos::TerminalEndl();
    bb7_status = true; */
}