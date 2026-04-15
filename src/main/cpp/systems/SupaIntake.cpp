// SupaIntake.cpp - remake edition
// By Patib

#include "systems/SupaIntake.h"
#include "systems/System.h"



SupaIntake::SupaIntake() {
    //m_intakeDown = (m_intakeSolenoid.Get() == frc::DoubleSolenoid::kForward);
    // Create a current limit config to apply to the drive motors
    //auto currentLimitConfig = configs::CurrentLimitsConfigs{}.WithSupplyCurrentLimitEnable(true).WithSupplyCurrentLimit(units::ampere_t{30});
    //m_intakeMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{}.WithCurrentLimits(currentLimitConfig));
}

// Single-use functions
void SupaIntake::SetMotors(double speed) {
    m_motorSpeed = speed;
}

void SupaIntake::SetIntake(bool state) {
    m_intakeDown = state;

    if (state) {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kReverse);
    } else {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kForward);
    }
}

bool SupaIntake::GetIntakeDown() {
    return m_intakeDown;
}

// deploy updates to the hardware
void SupaIntake::Update(Robot::Mode mode, double t) {

    if (m_intakeDown) {
        m_intakeMotor.Set(m_motorSpeed);
    } else {
        m_intakeMotor.Set(0.0);
    }
}