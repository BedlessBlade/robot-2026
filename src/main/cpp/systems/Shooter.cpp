// Venderdep includes
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <cmath>
#include <frc/Servo.h>
#include <algorithm>
#include <iostream>

// subsystem includes
#include "Constants.h"
#include "systems/Shooter.h"
#include "systems/ShotCalculator.h"

// Define constructor
Shooter::Shooter() : m_shooterState{shooterStates::IDLE} {
    // Configure turret motor
    m_azimuthConfig.closedLoop
        .P(Constants::kShooterAzimuthP)
        .I(Constants::kShooterAzimuthI)
        .D(Constants::kShooterAzimuthD)
        .OutputRange(Constants::kMinShooterAzimuth, Constants::kMaxShooterAzimuth)
    
    .feedForward
        .kS(Constants::kShooterAzimuthS)
        .kV(Constants::kShooterAzimuthV)
        .kA(Constants::kShooterAzimuthA)
        .kG(0)
        .kCos(0)
        .kCosRatio(0);

    // Config for shooter motors
    ctre::phoenix6::configs::TalonFXConfiguration m_shootConfig;
    m_shootConfig.Slot0.kS = 0;
    m_shootConfig.Slot0.kV = 0.12;//Constants::kShooterMotorV; // Velocity feedforward
    m_shootConfig.Slot0.kP = 0.11;//Constants::kShooterMotorP; // Proportional gain
    m_shootConfig.Slot0.kI = 0;//Constants::kShooterMotorI; // Integral gain
    m_shootConfig.Slot0.kD = 0;//Constants::kShooterMotorD; // Derivative gain
    
    m_shooterMotorRight.GetConfigurator().Apply(m_shootConfig);
    m_shooterMotorLeft.GetConfigurator().Apply(m_shootConfig);
}

// Define setters
void Shooter::StartShooting() {
    m_shooterState = shooterStates::PREFIRE;
};

void Shooter::StopShooting() {
    m_shooterState = shooterStates::IDLE;
};

void Shooter::SetTurretAngle(units::degree_t angle) {
    m_azimuthSetpoint = angle;
};

void Shooter::SetShooterSpeed(units::turns_per_second_t speed) {
    m_velocitySetpoint = speed;
    //std::cout << "setting velocity setpoint";
};
 
void Shooter::SetHoodPosition(double position) {
    m_hoodSetpoint = position;
};

// Define getters

// Define readytofire
bool Shooter::ReadyToFire() {
    //double velocityError = std::abs(m_velocitySetpoint.value() - GetShooterSpeed());
    //double turretError = std::abs(m_azimuthSetpoint.value() - GetTurretAngle());
    return true;
};

// Define update function
void Shooter::Update(Robot::Mode mode, double t) {
    // Shooter statemachine
    if (m_shooterState == shooterStates::IDLE) {
        // Shooter State is IDLE
        SetShooterSpeed(0.0_tps);
        //SetTurretAngle(0.0_deg);

    } else if (m_shooterState == shooterStates::PREFIRE) {
        // Shooter State is PREFIRE
        //SetShooterSpeed(ShotCalculator::GetInstance().GetShooterVelocity());
        //SetTurretAngle(ShotCalculator::GetInstance().GetTurretAngle());
        SetShooterSpeed(50.0_tps);

        // If at setpoint, transition to fire
        if (ReadyToFire()) {
            m_shooterState = shooterStates::FIRE;
        }
    
    } else if (m_shooterState == shooterStates::FIRE) {
        // Shooter State is FIRE
        //SetShooterSpeed(ShotCalculator::GetInstance().GetShooterVelocity());
        //SetTurretAngle(ShotCalculator::GetInstance().GetTurretAngle());
        SetShooterSpeed(50.0_tps);
    }
    
    // Update shooter
    m_shooterMotorRight.SetControl(m_rightVelocity.WithVelocity(m_velocitySetpoint)); // Set shooter motor 1
    m_shooterMotorLeft.SetControl(m_leftVelocity.WithVelocity(-m_velocitySetpoint)); // Set shooter motor 2

    m_leftHoodServo.Set(m_hoodSetpoint);
    m_rightHoodServo.Set(m_hoodSetpoint);
    
    // Update turret
    double turretRevs = std::clamp(m_azimuthSetpoint.value() + 0.0, 0.0, 270.0) / 360; // Convert, clamp, and add offset to setpoint
    double motorRevs = Constants::kAzimuthMotorRevsToRevs * turretRevs; // Convert turret rotations to motor rotations (214.5:1 reduction)
    m_azimuthController.SetSetpoint(motorRevs, rev::spark::SparkBase::ControlType::kPosition); // Set turret motor
 };