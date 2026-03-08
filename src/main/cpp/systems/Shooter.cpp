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
#include "systems/Indexer.h"

using namespace rev::spark;
using namespace ctre::phoenix6;

// Define constructor
Shooter::Shooter() : 
    // Inital turret, shooter, and hood setpoints
    m_shooterState{shooterStates::IDLE},
    m_azimuthSetpoint{0.0_deg},
    m_velocitySetpoint{0.0_tps},
    m_hoodSetpoint{0.0} {

    // Configure turret motor
    m_azimuthConfig.closedLoop
        .P(Constants::kShooterAzimuthP)
        .I(Constants::kShooterAzimuthI)
        .D(Constants::kShooterAzimuthD)
        .OutputRange(Constants::kMinAzimuthOutput, Constants::kMaxAzimuthOutput)
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder);
    

    m_azimuthConfig.Inverted(true);
    //.feedForward
    //    .kS(Constants::kShooterAzimuthS)
    //    .kV(Constants::kShooterAzimuthV)
    //    .kA(Constants::kShooterAzimuthA)
    //    .kG(0)
    //    .kCos(0)
    //    .kCosRatio(0);

    //m_azimuthConfig.closedLoop.maxMotion
    //    .CruiseVelocity(Constants::kShooterAzimuthCV) // Trapz max velocity
    //    .MaxAcceleration(Constants::kShooterAzimtuhAcc) // Trapz acceleration
    //    .AllowedProfileError(Constants::kShooterAzimuthTol); // Trapz path following tolerance
    
    m_azimuthTurretMotor.Configure(m_azimuthConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
    m_azimuthTurretMotor.GetEncoder().SetPosition(0);

    // Config for shooter motors
    ctre::phoenix6::configs::TalonFXConfiguration m_shootConfig;
    m_shootConfig.Slot0.kS = Constants::kShooterMotorS; // Friction feedforward
    m_shootConfig.Slot0.kV = Constants::kShooterMotorV; // Velocity feedforward
    m_shootConfig.Slot0.kP = Constants::kShooterMotorP; // Proportional gain
    m_shootConfig.Slot0.kI = Constants::kShooterMotorI; // Integral gain
    m_shootConfig.Slot0.kD = Constants::kShooterMotorD; // Derivative gain
    
    m_shooterMotorLeft.GetConfigurator().Apply(m_shootConfig);
    m_shooterMotorRight.SetControl(controls::Follower{m_shooterMotorLeft.GetDeviceID(), true});
}

// Define setters
void Shooter::StartShooting() {
    m_shooterState = shooterStates::PREFIRE;
};

void Shooter::StopShooting() {
    m_shooterState = shooterStates::IDLE;
    Indexer::GetInstance().StopIndexing();
};

void Shooter::SetTurretAngle(units::degree_t angle) {
    m_azimuthSetpoint = angle + 180.0_deg;
};

void Shooter::SetShooterSpeed(units::turns_per_second_t speed) {
    m_velocitySetpoint = speed;
};
 
void Shooter::SetHoodPosition(double position) {
    m_hoodSetpoint = position;
};

// Define getters
double Shooter::GetShooterSpeed() {
    return m_shooterMotorLeft.GetVelocity().GetValue().value();
};

double Shooter::GetTurretAngle() {
    return 360 * m_azimuthTurretMotor.GetEncoder().GetPosition() / Constants::kAzimuthMotorRevsToRevs; // gets current turret angle (deg)
};

// Define readytofire
bool Shooter::ReadyToFire() {
    double velocityError = std::abs(m_velocitySetpoint.value() - GetShooterSpeed());
    double azimuthError = std::abs(m_azimuthSetpoint.value() - GetTurretAngle());
    if (m_shooterState == shooterStates::PREFIRE && velocityError < Constants::velErrorTol && azimuthError < Constants::azimuthErrorTol) {
        // If the velocity and azimuth error is less than the tolerance, ok to transition to shoot
        return true;

    } else if (m_shooterState == shooterStates::FIRE && azimuthError < Constants::azimuthErrorTol) {
        // If we're already shooting, disregard velocity and only look at azimuth error
        return true;

    } else {
        // Default state if we're not shooting or preparing to shoot
        return false;
    } 
};

// Define update function
void Shooter::Update(Robot::Mode mode, double t) {
    if (mode == Robot::kAuto || mode == Robot::kTeleop) {
        // Shooter statemachine
        if (m_shooterState == shooterStates::IDLE) {
            // Shooter State is IDLE
            SetShooterSpeed(0.0_tps);
            SetTurretAngle(0.0_deg);
            // SetHoodPosition(0.2);

        } else if (m_shooterState == shooterStates::PREFIRE) {
            // Shooter State is PREFIRE
            SetShooterSpeed(ShotCalculator::GetInstance().GetShooterVelocity());
            SetTurretAngle(ShotCalculator::GetInstance().GetTurretAngle());
            // SetHoodPosition(0.2);

            // If at setpoint, transition to fire
            if (ReadyToFire()) {
                m_shooterState = shooterStates::FIRE;
            }
        
        } else if (m_shooterState == shooterStates::FIRE) {
            // Shooter State is FIRE
            SetShooterSpeed(ShotCalculator::GetInstance().GetShooterVelocity());
            SetTurretAngle(ShotCalculator::GetInstance().GetTurretAngle());
            // SetHoodPosition(0.2);

            if (ReadyToFire()) {
                Indexer::GetInstance().StartIndexing();
            } else {
                Indexer::GetInstance().StopIndexing();
            }
        }
        
        // Update shooter
        m_shooterMotorLeft.SetControl(m_leftVelocity.WithVelocity(m_velocitySetpoint)); // Set shooter motor 2

        // m_leftHoodServo.Set(m_hoodSetpoint);
        // m_rightHoodServo.Set(m_hoodSetpoint);
        
        // Update turret
        double turretRevs = std::clamp(m_azimuthSetpoint.value(), Constants::kMinShooterAzimuth, Constants::kMaxShooterAzimuth) / 360; // Convert and clamp setpoint
        double motorRevs = Constants::kAzimuthMotorRevsToRevs * turretRevs; // Convert turret rotations to motor rotations (214.5:1 reduction)
        m_azimuthController.SetSetpoint(motorRevs, SparkMax::ControlType::kPosition, ClosedLoopSlot::kSlot0); // Set turret motor
    }
};