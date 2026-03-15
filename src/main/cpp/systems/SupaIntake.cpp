// SupaIntake.cpp - remake edition
// By Patib

#include "systems/SupaIntake.h"
#include "systems/System.h"

SupaIntake::SupaIntake() {}

// Single-use functions
void SupaIntake::SetMotors(double Speed) {
    if (m_intakeDown){
    m_motorSpeed = Speed;
    } else {
        m_motorSpeed = Speed;
    }
}

void SupaIntake::ToggleIntake() {
    m_intakeDown = !m_intakeDown;
}

// deploy updates to the hardware
void SupaIntake::Update(Robot::Mode mode, double t) {
    m_intakeSupaMotor.Set(m_motorSpeed);

    // cases so we can use bool to control pneumatics
    if (m_intakeDown) {
        // m_intakeSolenoid.Set(frc::DoubleSolenoid::kOff);
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
        // m_intakeSolenoid.Set(frc::DoubleSolenoid::kOff);
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kReverse);
    }
}