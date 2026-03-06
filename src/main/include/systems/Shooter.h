#pragma once

// Venderdep includes
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <frc/Servo.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

// subsystem includes
#include "Constants.h"
#include "System.h"

// shooter class definition
class Shooter : public System {
public:
    // singleton getinstance
    static Shooter &GetInstance() {
        static Shooter instance;
        return instance;
    }

    // Shooter State
    enum shooterStates {IDLE, PREFIRE, FIRE};

    // Setters
    void SetTurretAngle(units::degree_t angle);
    void SetShooterSpeed(units::turns_per_second_t speed);
    void SetHoodPosition(double position);
    void StartShooting();
    void StopShooting();

    // Getters
    double GetTurretAngle();
    double GetShooterSpeed();
    double GetHoodPosition();
    shooterStates GetShooterState();

    // other functions
    void Update(Robot::Mode mode, double t) override;
    bool ReadyToFire();

private:
    // turret hardware
    rev::spark::SparkMax m_azimuthTurretMotor{Constants::kAzimuthMotorId, rev::spark::SparkMax::MotorType::kBrushless}; // get motor for turret rotation
    rev::spark::SparkClosedLoopController m_azimuthController = m_azimuthTurretMotor.GetClosedLoopController(); // get controller for turret rotation
    rev::spark::SparkMaxConfig m_azimuthConfig; //configured in the cpp file

    // shooter hardware
    ctre::phoenix6::hardware::TalonFX m_shooterMotorRight{Constants::kShooterMotorRightId}; // Kraken X60 #1
    ctre::phoenix6::hardware::TalonFX m_shooterMotorLeft{Constants::kShooterMotorLeftId}; // Kraken X60 #2

    ctre::phoenix6::controls::VelocityVoltage m_rightVelocity = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0); // right motor velocity voltage controller
    ctre::phoenix6::controls::VelocityVoltage m_leftVelocity = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0); // left motor velocity voltage controller
    ctre::phoenix6::controls::NeutralOut m_brake{};

    frc::Servo m_leftHoodServo{Constants::kLeftHoodServoPort}; // Hood linear servo #1
    frc::Servo m_rightHoodServo{Constants::kRightHoodServoPort}; // Hood linear servo #2

    // member variables
    shooterStates m_shooterState;
    units::degree_t m_azimuthSetpoint; // turret angle setpoint (degrees)
    units::turns_per_second_t m_velocitySetpoint; // shooter flywheel speed setpoint (RPS)
    double m_hoodSetpoint; // hood angle setpoint (% output, 0-1)

    // Singleton private constructor
    Shooter();
};