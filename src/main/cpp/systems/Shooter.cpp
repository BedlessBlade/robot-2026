#include "Constants.h"
#include "Systems/Shooter.h"

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
};

void Shooter::SetAngle(double angle) {
    double revs = Constants::kAzimuthMotorRevsToRevs * (angle / 2);
    m_azimuthController.SetSetpoint(revs, rev::spark::SparkBase::ControlType::kPosition);
};

bool Shooter::IsAtSetpoint() {
    return m_azimuthController.IsAtSetpoint();
};

auto Shooter::adjustMotorSpeed(double topSpeed, double lowSpeed) {
    m_ShooterMotorLow.Set(lowSpeed);
    m_ShooterMotorTop.Set(topSpeed);
}

// i aint doing this
void adjustMotors() {};

void Shooter::ToggleMotors(bool Toggle, double TopSpeed, double LowSpeed) {
    //Figure this out
    if (Toggle) {
        m_ShooterMotorTop.Set(TopSpeed);
        m_ShooterMotorLow.Set(LowSpeed);
    }
    else if (!Toggle) {
        m_ShooterMotorTop.Set(0);
        m_ShooterMotorLow.Set(0);
    }
}

void Align(frc::Pose2d pose) {
    units::length::meter_t x = pose.X();
    units::length::meter_t y = pose.Y();
    frc::Translation2d theta = pose.Translation(); 

    
}