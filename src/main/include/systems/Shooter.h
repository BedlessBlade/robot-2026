#pragma once

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <frc/Servo.h>

#include "Constants.h"
#include "System.h"

class Shooter : public System {

public:
    static Shooter &GetInstance() {
        static Shooter instance;
        return instance;
    }

    void SetAngle(double angle);

    void SetShooterSpeed(units::angular_velocity::turns_per_second_t Speed);

    bool IsAtSetpoint();

    void Update(Robot::Mode mode, double t) override;

    void SetHoodState(double state);

private:
    // change back to Turret motor for once done
    rev::spark::SparkMax m_azimuthTurretMotor{Constants::kAzimuthMotorId, rev::spark::SparkMax::MotorType::kBrushless}; // get motor for turret rotation
    rev::spark::SparkClosedLoopController m_azimuthController = m_azimuthTurretMotor.GetClosedLoopController(); // get controller for turret rotation

    ctre::phoenix6::hardware::TalonFX m_shooterMotorRight{Constants::kShooterMotorRightId};
    ctre::phoenix6::hardware::TalonFX m_shooterMotorLeft{Constants::kShooterMotorLeftId};

    // control templates for velocity control on shooter motors
    ctre::phoenix6::controls::VelocityVoltage m_rightVelocity;
    ctre::phoenix6::controls::VelocityVoltage m_leftVelocity;

    frc::Servo m_hoodServo{Constants::kHoodServoPort};

    rev::spark::SparkMaxConfig m_azimuthConfig; //configured in the cpp file

    units::angular_velocity::turns_per_second_t m_shooterSpeed;

    double m_azimuthSetpoint;

    double m_hoodState;

    Shooter();
};