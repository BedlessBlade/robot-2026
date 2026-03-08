#include "systems/Intake.h"

#include <iostream>

Intake::Intake() {
}


void Intake::SetIntakeSpeed(double Speed) {
    m_speed = Speed;
};

void Intake::SetExtensionState(bool State) {
    m_state = State;
};


void Intake::Update(Robot::Mode mode, double t) {
    m_intakeMotor.Set(m_speed); // Set speed
    //std::cout << "called" << std::endl;

    // Set pistons
    if (m_state) { 
        m_solenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
        m_solenoid.Set(frc::DoubleSolenoid::kReverse);
    };
};