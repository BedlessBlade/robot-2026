#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <cmath>

#include "Constants.h"
#include "systems/Shooter.h"

Shooter::Shooter() {
    m_azimuthConfig.closedLoop
    .P(Constants::kShooterAzimuthP)
    .I(Constants::kShooterAzimuthI)
    .D(Constants::kShooterAzimuthD)
    .OutputRange(Constants::kMinShooterAzimuth,
        Constants::kMaxShooterAzimuth)

    .feedForward  // config the const. from header
    .kS(Constants::kShooterAzimuthS)
    .kV(Constants::kShooterAzimuthV)
    .kA(Constants::kShooterAzimuthA)
    .kG(Constants::kShooterAzimuthG)
    .kCos(Constants::kShooterAzimuthCos)
    .kCosRatio(Constants::kShooterAzimuthCosRatio);

    ctre::phoenix6::controls::VelocityVoltage m_rightVelocity(0_tps);
    ctre::phoenix6::controls::VelocityVoltage m_leftVelocity(0_tps);

    ctre::phoenix6::configs::Slot0Configs m_shootConfig;
    m_shootConfig.kV = Constants::kShooterMotorV;
    m_shootConfig.kP = Constants::kShooterMotorP;
    m_shootConfig.kI = Constants::kShooterMotorI;
    m_shootConfig.kD = Constants::kShooterMotorD;

    m_shooterMotorRight.GetConfigurator().Apply(m_shootConfig, 50_ms);
    m_shooterMotorLeft.GetConfigurator().Apply(m_shootConfig, 50_ms);

    m_rightVelocity.Slot = 0;
    m_leftVelocity.Slot = 0;
};

void Shooter::SetAngle(double angle) {
    m_azimuthSetpoint = angle;
};

bool Shooter::IsAtSetpoint() {
    return m_azimuthController.IsAtSetpoint();
};

void Shooter::SetShooterSpeed(units::angular_velocity::turns_per_second_t Speed) {
    m_shooterSpeed = Speed;
};

void Shooter::Update(Robot::Mode mode, double t) {
    m_shooterMotorRight.SetControl(m_rightVelocity.WithVelocity(m_shooterSpeed));
    m_shooterMotorLeft.SetControl(m_leftVelocity.WithVelocity(m_shooterSpeed));
    
    double revs = Constants::kAzimuthMotorRevsToRevs * (m_azimuthSetpoint / (2 * M_PI));
    m_azimuthController.SetSetpoint(revs, rev::spark::SparkBase::ControlType::kPosition);


};