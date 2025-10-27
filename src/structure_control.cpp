#include "structure_control.h"

void intake::SetDirection(bool a){
    channel_direction = a;
}

void intake::SetActivate(bool a){
    channel_activate = a;
}

void intake::SpinDrive(){
    static bool brake_set = false;
    if (!brake_set) {
        IntakeD1.setBrake(brake);
        IntakeD2.setBrake(brake);
        IntakeU1.setBrake(brake);
        IntakeU2.setBrake(brake);
        brake_set = true;
    }

    if(eos::COMPETITION->isAutonomous() || eos::AUTODEBUG){

        channel_voltage = MAX_VOLTAGE * (channel_direction ? 1 : -1) * channel_activate;


    }else if(eos::COMPETITION->isDriverControl() && !eos::AUTODEBUG){

        channel_voltage = MAX_VOLTAGE * (intake::In() - intake::Out());

    }

    IntakeD1.spin(fwd , channel_voltage , voltageUnits::mV);
    IntakeD2.spin(fwd , channel_voltage , voltageUnits::mV);
    IntakeU1.spin(fwd , channel_voltage , voltageUnits::mV);
    IntakeU2.spin(fwd , channel_voltage , voltageUnits::mV);
}

void pistons::SetCannonStatus(bool s){
    cannon = s;
}

void pistons::SetCoverStatus(bool s){
    cover = s;
}

void pistons::ActivateDrive(){
    if(eos::COMPETITION->isAutonomous() || eos::AUTODEBUG){

        //nothing to do

    }else if(eos::COMPETITION->isDriverControl() && !eos::AUTODEBUG){

        eos::SingleButtonControl(cannon , pistons::CannonUp);
        eos::DoubleButtonControl(cover , pistons::CoverUp , pistons::CoverDown);

    }

    CannonPiston.set(cannon);
    CoverPiston.set(cover);
}