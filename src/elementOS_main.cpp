#include "elementOS_lite.h"
#include "chassis.h"
#include "structure_control.h"

void AutoRoute(){
    chassis::GetInstance().ChassisRoute();
    if(eos::AUTODEBUG){eos::AUTODEBUG = false;}
}

void Initialize(){
    wait(500,msec);
    Inertial.calibrate();
    waitUntil(!Inertial.isCalibrating());
    EncoderL.resetRotation();
    EncoderR.resetRotation();
    LFU.resetPosition();
    LFD.resetPosition();
    RFU.resetPosition();
    RFD.resetPosition();
    LBU.resetPosition();
    LBD.resetPosition();
    RBU.resetPosition();
    RBD.resetPosition();
}

void BrainInfoDisplay(){
    //pose
    eos::LCD->printAt(1 , 1*SINGLE_LINE_HIGHT , "yaw = %.2f   " , chassis::GetInstance().GetYaw());
    eos::LCD->printAt(1 , 2*SINGLE_LINE_HIGHT , "x = %.2f   " , chassis::GetInstance().GetPose().x);
    eos::LCD->printAt(1 , 3*SINGLE_LINE_HIGHT , "y = %.2f   " , chassis::GetInstance().GetPose().y);

    //chassis temperature
    eos::LCD->printAt(1 , 5*SINGLE_LINE_HIGHT , "average_temperature = %d   " , chassis::GetInstance().GetAverageTemperature());
    eos::LCD->printAt(1 , 6*SINGLE_LINE_HIGHT , "max_temperature = %d   " , chassis::GetInstance().GetMaxTemperature());

    /* //velocity
    eos::LCD->printAt(1 , 8*SINGLE_LINE_HIGHT , "liner_velocity = %.2f   " , chassis::GetInstance().GetLinerVelocity());
    eos::LCD->printAt(1 , 10*SINGLE_LINE_HIGHT , "angular_velocity = %.2f   " , chassis::GetInstance().GetAngularVelocity()); */

    //odometry
    /* eos::LCD->printAt(1 , 8*SINGLE_LINE_HIGHT , "odom_x = %.2f   " , chassis::GetInstance().Odometry.odom_pose.x);
    eos::LCD->printAt(1 , 9*SINGLE_LINE_HIGHT , "odom_y = %.2f   " , chassis::GetInstance().Odometry.odom_pose.y);
    eos::LCD->printAt(1 , 10*SINGLE_LINE_HIGHT , "odom_theta = %.2f   " , chassis::GetInstance().Odometry.odom_pose.theta * RAD_TO_DEG);
    eos::LCD->printAt(1 , 11*SINGLE_LINE_HIGHT , "odom_liner = %.2f   " , chassis::GetInstance().Odometry.odom_liner_velocity);
    eos::LCD->printAt(1 , 12*SINGLE_LINE_HIGHT , "odom_angular = %.2f   " , chassis::GetInstance().Odometry.odom_angular_velocity); */

    //arm
    //eos::LCD->printAt(1 , 13*SINGLE_LINE_HIGHT , "arm_angle = %.2f   " , arm::GetInstance().GetAngle());
    
}

void ControllerInfoDisplay(){
    eos::ControllerPrint("电池:" , 1 , 1);
    if(eos::SystemBattery() >= 10){
        eos::ControllerPrint(eos::SystemBattery() , 1 , 8);
    }else{
        eos::ControllerPrint(eos::SystemBattery() , 1 , 8);
        eos::ControllerPrint(" " , 1 , 9);
    }

    eos::ControllerPrint("联队:" , 1 , 15);
    if(eos::ALLIANCE == 0){
        eos::ControllerPrint("Red" , 1 , 22);
    }else{
        eos::ControllerPrint("Blue" , 1 , 22);
    }

    eos::ControllerPrint("Powered by ElemntOS" , 2 , 1);
}

void EosMain(){
    eos::ClearControllerScreen();
    eos::ClearBrainScreen();
    eos::TerminalClear();

    thread buzzer_thread(eos::BuzzerDrive);
    
    //遥控器显示
    eos::ControllerPrint("==================" , 1 , 1);
    eos::ControllerPrint("校准中" , 2 , 10);
    eos::ControllerPrint("==================" , 3 , 1);
    //主控显示
    eos::LCD->printAt(1 , 1*SINGLE_LINE_HIGHT , "Initializing... Do not move robot!");
    eos::LCD->printAt(1 , 2*SINGLE_LINE_HIGHT , "Powered by ElemntOS");
    //终端显示
    eos::TerminalPrint("Powered by ElemntOS." , RESET);

    //开始初始化
    Initialize();
    eos::ClearControllerScreen();
    eos::ClearBrainScreen();
    eos::INTTIALIZE_READY = true;

    if(!(eos::COMPETITION->isFieldControl() || eos::COMPETITION->isCompetitionSwitch())){
        //未接入场控

        eos::brain_button op_debug = eos::brain_button(50 , 50 , 100 , 160 , "Op Debug" , HEX_GREEN , HEX_WHITE);
        eos::brain_button auto_debug = eos::brain_button(280 , 50 , 100 , 160 , "Auto Debug" , HEX_YELLOW , HEX_BLACK);

        //选择调试模式
        while(true){
            eos::ControllerPrint("手动调试: B" , 1 , 7);
            eos::ControllerPrint("自动调试: A" , 2 , 7);
            eos::ControllerPrint("==================" , 3 , 1);

            eos::LCD->printAt(1 , SINGLE_LINE_HIGHT , "Choose your debugging type...");
            op_debug.DisplayButton();
            auto_debug.DisplayButton();

            if(eos::A() || auto_debug.IsPressed()){
                eos::AUTODEBUG = true; eos::OPDEBUG = false;
                eos::TerminalClear();
                eos::TerminalPrint("Autonomous Debug is running......" , GREEN);
                eos::TerminalEndl();
                break;
            }else if(eos::B() || op_debug.IsPressed()){
                eos::AUTODEBUG = false; eos::OPDEBUG = true;
                eos::TerminalClear();
                eos::TerminalPrint("Operator Debug is running......" , GREEN);
                if(eos::SystemBattery() <= 30){
                    eos::TerminalPrintWithData("Battery: " , eos::SystemBattery() , RED);
                }else if(eos::SystemBattery() > 30 && eos::SystemBattery() <= 70){
                    eos::TerminalPrintWithData("Battery: " , eos::SystemBattery() , YELLOW);
                }else{
                    eos::TerminalPrintWithData("Battery: " , eos::SystemBattery() , GREEN);
                }
                break;
            }
            
            eos::SystemWait(); 
        }
        eos::ClearControllerScreen();
        eos::ClearBrainScreen();
        eos::READY = true;
        double t = eos::SystemTime(sec);

        //遥控器显示
        ControllerInfoDisplay();

        while(true){
            
            if(!eos::AUTODEBUG && !eos::OPDEBUG){
                eos::TerminalPrintWithData("Debugging is over , total time = " , eos::SystemTime(sec) - t , RED);
                eos::TerminalPrintWithData("x = " , chassis::GetInstance().GetPose().x , BLUE);
                eos::TerminalPrintWithData("y = " , chassis::GetInstance().GetPose().y , BLUE);
                eos::TerminalPrintWithData("yaw = " , chassis::GetInstance().GetYaw() , BLUE);
                eos::TerminalPrintWithData("temperature = " , chassis::GetInstance().GetAverageTemperature() , BLUE);
                if(eos::SystemBattery() <= 30){
                    eos::TerminalPrintWithData("Battery = " , eos::SystemBattery() , RED);
                }else if(eos::SystemBattery() > 30 && eos::SystemBattery() <= 70){
                    eos::TerminalPrintWithData("Battery = " , eos::SystemBattery() , YELLOW);
                }else{
                    eos::TerminalPrintWithData("Battery = " , eos::SystemBattery() , GREEN);
                }
                break;
            }

            BrainInfoDisplay();

            eos::SystemWait();
        }
    }else{
        //接入场控
        eos::READY = true;

        eos::ControllerPrint("电池:" , 1 , 1);
        if(eos::SystemBattery() >= 10){
            eos::ControllerPrint(eos::SystemBattery() , 1 , 8);
        }else{
            eos::ControllerPrint(eos::SystemBattery() , 1 , 8);
            eos::ControllerPrint(" " , 1 , 9);
        }

        eos::ControllerPrint("联队:" , 1 , 15);
        if(eos::ALLIANCE == 0){
            eos::ControllerPrint("Red" , 1 , 22);
        }else{
            eos::ControllerPrint("Blue" , 1 , 22);
        }

        eos::ControllerPrint("Powered by ElemntOS" , 2 , 1);

        while(true){
            eos::BrainDisplayImage(0 , 0 , "usst_vex.png");
            eos::SystemWait(3000);
            eos::ClearBrainScreen();
            eos::BrainDisplayImage(0 , 0 , "periodic_table.png");
            eos::SystemWait(3000);
            eos::ClearBrainScreen();
        }
    }

    eos::SystemWait(1000);
    eos::SystemExit();
}