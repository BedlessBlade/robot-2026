#include "systems/NewSupaIntake.h"
#include "systems/System.h"

#include "Constants.h"
#include "Controllers.h"

// Intake Fuel
void NewSupaIntake::startMotor() {
    m_sparkMotor.Set(0.7);
};

// Don't intake Fuel
void NewSupaIntake::stopMotor() {
    m_sparkMotor.Set(0.0);
}

// Get intake ready for Fuel
void NewSupaIntake::pullForw() {
    m_solenoid.Set(frc::DoubleSolenoid::kForward);
}

// Return intake post-Fuel
void NewSupaIntake::pullBack() {
    m_solenoid.Set(frc::DoubleSolenoid::kReverse);
}

// Update Robot
void NewSupaIntake::Update(Robot::Mode mode, double t) {
    if (mode == Robot::Mode::kTeleop) {
        if (Controllers::GetInstance().GetOperatorController().GetLeftTriggerAxis() > 0.25) {
            startMotor();
        }
        else {
            stopMotor();
        };
    } else if (mode == Robot::Mode::kAuto) {
        //Idk
    }
}