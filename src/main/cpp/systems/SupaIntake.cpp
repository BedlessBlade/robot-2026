// SupaIntake.cpp - remake edition
// By Patib

#include "systems/SupaIntake.h"
#include "systems/System.h"

SupaIntake::SupaIntake() {}

// Single-use functions
void SupaIntake::SetMotors(double Speed) {
    if (m_pneumState){
        m_motorSpeed = Speed;
    } else {
        m_motorSpeed = 0.0;
    }
}

void SupaIntake::SetPneums() {
    if (m_pneumState) {

          m_pneumState = false;
        } else {

          m_pneumState = true;
        }
    
}

// deploy updates to the hardware
void SupaIntake::Update(Robot::Mode mode, double t) {
    m_intakeSupaMotor.Set(m_motorSpeed);

    // cases so we can use bool to control pneumatics
    if (m_pneumState) {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kOff);
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kOff);
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kReverse);
    }

    // Assign if teleop is on to a local bool
    if (mode == Robot::Mode::kTeleop) {
        m_easilyUse = false;
    } else {
        m_easilyUse = true;
    }
}

// general logic functions (can be used in auto modes for ease of use)
// void SupaIntake::StartAll() {
//     if (m_easilyUse) {
//         m_motorSpeed = Constants::kIntakeForce;
//         m_pneumState = true;
//     }
// }
// void SupaIntake::EndAll() {
//     if (s_easilyUse) {
//         s_motorSpeed = 0.0;
//         s_pneumState = false;
//     }
// }
