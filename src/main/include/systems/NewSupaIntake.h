#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DoubleSolenoid.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <rev/SparkMax.h>

#include "Robot.h"
#include "Constants.h"
#include "systems/System.h"

using namespace ctre::phoenix6;

class NewSupaIntake : public System {
public:
    // Static Instance
    static NewSupaIntake &GetInstance() {
        static NewSupaIntake instance;
        return instance; 
    }

    void startMotor();
    void stopMotor();

    void pullForw();
    void pullBack();

    void Update(Robot::Mode mode, double t);

private:
    rev::spark::SparkMax m_sparkMotor{Constants::kIntakeMotorId, rev::spark::SparkMax::MotorType::kBrushless};

    frc::DoubleSolenoid m_solenoid{Constants::kIntakePneumCanId,
         frc::PneumaticsModuleType::REVPH,
         Constants::kIntakePneumId1,
         Constants::kIntakePneumId2};

    NewSupaIntake();
};