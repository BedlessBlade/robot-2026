#pragma once

#include <rev/SparkMax.h>
#include <frc/DoubleSolenoid.h>

#include "Constants.h"
#include "System.h"

class Intake : public System {

public:
    static Intake &GetInstance() {
        static Intake instance;
        return instance;
    }


    void SetIntakeSpeed(double Speed);
    
    void SetExtensionState(bool State);

    void Update(Robot::Mode mode, double t) override;


private:

    rev::spark::SparkMax m_intakeMotor{Constants::kIntakeMotorId, rev::spark::SparkMax::MotorType::kBrushless};

    bool m_state;

    double m_speed;

    frc::DoubleSolenoid m_solenoid{Constants::kIntakePneumCanId,
         frc::PneumaticsModuleType::REVPH,
         Constants::kIntakePneumId1,
         Constants::kIntakePneumId2};

    Intake();
};
