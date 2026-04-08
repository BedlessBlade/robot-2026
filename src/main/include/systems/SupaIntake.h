#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DoubleSolenoid.h>

#include "Constants.h"
#include "System.h"

class SupaIntake : public System {

public:

    static SupaIntake &GetInstance() {
        static SupaIntake instance;
        return instance;
    }

    void SetMotors(double speed);   // Motors Function
    void SetIntake(bool state);
    bool GetIntakeDown();

    void Update(Robot::Mode mode, double t) override;   // Set Values to hardware

private:
    ctre::phoenix6::hardware::TalonFX m_intakeMotor{Constants::kIntakeMotorId};

    frc::DoubleSolenoid m_intakeSolenoid{Constants::kIntakePneumCanId,
        frc::PneumaticsModuleType::REVPH,
        Constants::kIntakePneumId1,
        Constants::kIntakePneumId2};

    bool m_intakeDown = false;    // False is up, True is down
    double m_motorSpeed;    // Set motor to equal speed (-1.0 to 1.0)

    SupaIntake();
};