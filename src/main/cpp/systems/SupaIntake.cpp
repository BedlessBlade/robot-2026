// SupaIntake.cpp - remake edition
// By Patib

#include "systems/SupaIntake.h"
#include "systems/System.h"

SupaIntake::SupaIntake() {}

// Single-use functions
void SupaIntake::SetMotors(double Speed) {
    // Only update if new speed is not 0 or motor speed is not 0
    // Prevents useless updates from happening over and over
    // But not stopping the fast-paced updating
    // if (spd != 0.0 or s_motorSpeed != 0.0) {
    //     s_motorSpeed = spd;     
    // }
    m_motorSpeed = Speed;
}

void SupaIntake::SetPneums() {
    // Stops overwriting the bool
    // unless the overwrite changes
    // if (set != s_pneumState) {
    //     s_pneumState = set;
    // }
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
