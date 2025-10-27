#include "elementOS_lite.h"
#include "chassis.h"
#include "structure_control.h"

using namespace vex;

competition Competition;

void autonomous(void) {
  waitUntil(eos::READY);
  AutoRoute();
}

void usercontrol(void) {
  waitUntil(eos::READY);
  if(eos::AUTODEBUG){
    AutoRoute();
  }else{
    while(true){
      eos::SystemWait();
    }
  }
}

int main() {

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  eos::SystemInitialize(&Brain , &Controller , &Competition , &Buzzer);
  thread eos_main(EosMain);
  waitUntil(eos::READY);
  chassis::GetInstance();
  thread chassis_drive(chassis::ChassisDrive);
  thread intake_drive(intake::IntakeDrive);
  thread pistons_drive(pistons::PistonsDrive);

  while (true) {
    wait(100, msec);
  }
  
}
