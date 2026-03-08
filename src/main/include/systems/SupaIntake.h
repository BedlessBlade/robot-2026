#pragma once

#include <rev/SparkMax.h>
#include <frc/DoubleSolenoid.h>

#include "Constants.h"
#include "System.h"

class SupaIntake : public System {

public:

    static SupaIntake &GetInstance() {
        static SupaIntake instance;
        return instance;
    }

    void SetMotors(double spd);   // Motors Function
    void SetPneums(bool set);   // Pnuematics Function

    void StartAll();    // Enable and Disable all motors associated with the intake system 
    void EndAll();      // only allow use during auto & is not usable during teleop

    void Update(Robot::Mode mode, double t) override;   // Set Values to hardware

private:
    rev::spark::SparkMax m_intakeSupaMotor{Constants::kIntakeMotorId, rev::spark::SparkMax::MotorType::kBrushless};

    frc::DoubleSolenoid m_intakeSolenoid{Constants::kIntakePneumCanId,
        frc::PneumaticsModuleType::REVPH,
        Constants::kIntakePneumId1,
        Constants::kIntakePneumId2};

    bool s_pneumState = false;    // False is up, True is down
    bool s_easilyUse = false;   // Set so that ease of use functions cant be activated during teleop

    double s_motorSpeed;    // Set motor to equal speed (-1.0 to 1.0)

    SupaIntake();
};