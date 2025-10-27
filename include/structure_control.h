#pragma once
#include "elementOS_lite.h"
#include "chassis.h"

class intake{
private:
    intake(){}
    intake(const intake &_intake) = delete;
    const intake &operator=(const intake &_intake) = delete;

    //Configuration
    const double MAX_VOLTAGE = 10000; // mV
    bool (*In)(void) = eos::R1;
    bool (*Out)(void) = eos::R2;

    //Parameters
    bool channel_direction; // true = in , false = out
    bool channel_activate; // true = activate , false = stop

    //Controller
    double channel_voltage;

public:
    static intake& GetInstance(){
        static intake Intake;
        return Intake;
    }

    //Control
    void SetDirection(bool a);
    void SetActivate(bool a);

    //Drive
    void SpinDrive();
    static void IntakeDrive(){
        while(true){
            intake::GetInstance().SpinDrive();
            wait(10,msec);
        }
    }

};

class pistons{
private:
    pistons(){
        cannon = false;
        cover = false;
    };
    pistons(const pistons &_pistons) = delete;
    const pistons &operator=(const pistons &_pistons) = delete;

    //Configuration
    bool (*CoverUp)(void) = eos::Y;
    bool (*CoverDown)(void) = eos::B;
    bool (*CannonUp)(void) = eos::A;

    //Controller
    bool cannon;
    bool cover;

public:
    static pistons& GetInstance(){
        static pistons Pistons;
        return Pistons;
    }

    //Control
    void SetCannonStatus(bool s);
    void SetCoverStatus(bool s);

    //Drive
    void ActivateDrive();
    static void PistonsDrive(){
        while(true){
            pistons::GetInstance().ActivateDrive();
            wait(10,msec);
        }
    }

    //Output
    bool GetCannonStatus(){return cannon;};
    bool GetCoverStatus(){return cover;};
};