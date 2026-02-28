#include "Constants.h"
#include "Systems/Shooter.h"

Shooter::Shooter() {
    m_azimuthConfig.closedLoop
    .P(Constants::kShooterAzimuthP)
    .I(Constants::kShooterAzimuthI)
    .D(Constants::kShooterAzimuthD)
    .OutputRange(Constants::kMinAzimuthOutput,
        Constants::kMaxAzimuthOutput)

    .feedForward  // config the const. from header
    .kS(Constants::kShooterAzimuthS)
    .kV(Constants::kShooterAzimuthV)
    .kA(Constants::kShooterAzimuthA)
    .kG(Constants::kShooterAzimuthG)
    .kCos(Constants::kShooterAzimuthCos)
    .kCosRatio(Constants::kShooterAzimuthCosRatio);
};

void Shooter::SetAngle(double angle) {
    double revs = Constants::kAzimuthMotorRevsToRevs * (angle / (2 * M_PI));
    m_azimuthController.SetSetpoint(revs, rev::spark::SparkBase::ControlType::kPosition);
};

bool Shooter::IsAtSetpoint() {
    return m_azimuthController.IsAtSetpoint();
};

auto Shooter::SetMotorSpeed(double speed) {
    m_ShooterMotorLow.Set(speed);
    m_ShooterMotorTop.Set(speed);
}