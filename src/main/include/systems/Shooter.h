#pragma once

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "System.h"

class Shooter : public System {

public:
    static Shooter &GetInstance() {
        static Shooter instance;
        return instance;
    }

    void SetAngle(double angle);

    bool IsAtSetpoint();

    void ReverseMotors();

    void ShootingMotor();

    auto SetMotorSpeed(double speed);

    void ExtendMotor();

    void Update(Robot::Mode mode, double t) override;

private:
    // change back to Turret motor for once done
    rev::spark::SparkMax m_azimuthTurretMotor{Constants::kAzimuthMotorId, rev::spark::SparkMax::MotorType::kBrushless}; // get motor for turret rotation
    rev::spark::SparkClosedLoopController m_azimuthController = m_azimuthTurretMotor.GetClosedLoopController(); // get controller for turret rotation

    ctre::phoenix6::hardware::TalonFX m_ShooterMotorTop{Constants::kShooterMotorTopId};
    ctre::phoenix6::hardware::TalonFX m_ShooterMotorLow{Constants::kShooterMotorLowId};

    rev::spark::SparkMaxConfig m_azimuthConfig; //configured in the cpp file

    Shooter();
};