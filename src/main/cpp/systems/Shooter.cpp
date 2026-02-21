#include "Constants.h"
#include "systems/Shooter.h"

Shooter::Shooter() {
    m_azimuthConfig.closedLoop
    .P(Constants::kShooterAzimuthP)
    .I(Constants::kShooterAzimuthI)
    .D(Constants::kShooterAzimuthD)
    .OutputRange(Constants::kMinAzimuthOutput, Constants::kMaxAzimuthOutput);
}

void Shooter::SetAngle(double angle) {
    double revs = Constants::kAzimuthMotorRevsToRevs * (angle / 2);
    m_azimuthController.SetSetpoint(revs, rev::spark::SparkBase::ControlType::kPosition);
};