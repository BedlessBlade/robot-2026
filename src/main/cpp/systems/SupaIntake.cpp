//  -- SupaIntake.cpp by patib --
// defines the functions for the intake system and helps Robot.cpp interact w/ the motors and pistons
// last update: 2/19/2026

#include "systems/SupaIntake.h"
#include "systems/System.h"

#include "Constants.h"
#include "Controllers.h"

//Motors are under spinCur (spin current)
//Pistons are under pistLoc (piston location)

SupaIntake::spinCur SupaIntake::GetMotorCase() {
    return motorState; // send motorState to Robot.cpp
}

bool isOn = false;

//  --in honor of patibs 2 hours wasted--

//general single-command functions

//Intake Motor Controls
void SupaIntake::SpinIn () {
    m_sparkMotor.Set(Constants::kIntakeForward); //set 0.7 as test value to make sure it intakes properly
    motorState = spinCur::IN; //changes enum value so other files know spin property
};
void SupaIntake::SpinOut () {
    m_sparkMotor.Set(Constants::kIntakeReverse); // negitives set speed as same, but oppisite direction
    motorState = spinCur::OUT;
};

void SupaIntake::SpinStop () {
    m_sparkMotor.Set(0.0); //stops motor
    motorState = spinCur::IDLE; //sets enums for comparisons in Robot.cpp
};                                                                                  

//Intake Pistons Control
void SupaIntake::Extend () {
    m_solenoid.Set(frc::DoubleSolenoid::kForward);
    pnuemState = pistLoc::DOWN;
};
void SupaIntake::Retract () {
    m_solenoid.Set(frc::DoubleSolenoid::kReverse); //Reverse Direction of piston
    pnuemState = pistLoc::UP;
};

//Properly exit system easily
void SupaIntake::exitSystem() {
    SupaIntake::SpinStop();
    SupaIntake::Retract();
};

void SupaIntake::SetState(bool state) {
    intakeState = state;
};

//set intake properly
void SupaIntake::setIntake(bool setMode){
    if (setMode) {
        SpinIn();
        Extend();
    }
    else if (!setMode) {
        SpinOut();
        Retract();
        SpinStop();
    }
}


bool SupaIntake::ArmDown() {
    return pnuemState == pistLoc::UP;
};

//Misc IDK functions  NEED FIX
void SupaIntake::Update(Robot::Mode mode, double t) {
    if (mode == Robot::Mode::kTeleop) {
        if (Controllers::GetInstance().GetOperatorController().GetLeftTriggerAxis() >= 0.5 && !isOn) {
            setIntake(true);
            isOn = true;
        };
        if (Controllers::GetInstance().GetOperatorController().GetLeftTriggerAxis() < 0.5 && isOn) {
            setIntake(false);
            isOn = false;
        };
    }
    else if (mode == Robot::Mode::kAuto) {
        //Auto code goes here 
    }
};


SupaIntake::SupaIntake() {

}