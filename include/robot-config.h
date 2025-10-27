#pragma once
using namespace vex;

extern brain Brain;
extern controller Controller;

extern motor LFD;
extern motor LFU;
extern motor RFD;
extern motor RFU;
extern motor LBD;
extern motor LBU;
extern motor RBD;
extern motor RBU;

extern motor IntakeD1;
extern motor IntakeD2;
extern motor IntakeU1;
extern motor IntakeU2;

extern encoder EncoderL;
extern encoder EncoderR;

extern inertial Inertial;

extern vex::distance ChannelDistance;

extern digital_out CannonPiston;
extern digital_out CoverPiston;
extern digital_out Buzzer;