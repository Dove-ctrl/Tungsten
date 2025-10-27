#include "elementOS_lite.h"

using namespace vex;

brain Brain;
controller Controller = controller(primary);

motor LFD = motor(PORT13 , ratio6_1 , true);
motor LFU = motor(PORT14 , ratio6_1 , false);

motor RFD = motor(PORT19, ratio6_1 , false);
motor RFU = motor(PORT18, ratio6_1 , true);

motor LBD = motor(PORT11 , ratio6_1 , true);
motor LBU = motor(PORT12 , ratio6_1 , false);

motor RBD = motor(PORT17 , ratio6_1 , false);
motor RBU = motor(PORT16 , ratio6_1 , true);

motor IntakeD1 = motor(PORT1 , ratio6_1 , false);
motor IntakeD2 = motor(PORT2 , ratio6_1 , true);

motor IntakeU1 = motor(PORT3 , ratio6_1 , true);
motor IntakeU2 = motor(PORT4 , ratio6_1 , false);

encoder EncoderL = encoder(Brain.ThreeWirePort.G);
encoder EncoderR = encoder(Brain.ThreeWirePort.E);

inertial Inertial = inertial(PORT15);

vex::distance ChannelDistance = vex::distance(PORT10);

digital_out CannonPiston = digital_out(Brain.ThreeWirePort.B);
digital_out CoverPiston = digital_out(Brain.ThreeWirePort.A);
digital_out Buzzer = digital_out(Brain.ThreeWirePort.D);