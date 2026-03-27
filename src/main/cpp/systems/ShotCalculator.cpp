#include "systems/ShotCalculator.h"
#include "Constants.h"
#include "systems/SwerveDrive.h"

#include <frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <vector>
#include <wpi/interpolating_map.h>
#include <iostream>
#include <math.h>

// Call to calculate shot parameters
void ShotCalculator::CalculateShotParams(frc::Translation2d goalPosition, units::second_t latency) {
    // Calculate estimated pose while accounting for phase delay
    frc::Pose2d estimatedPose = SwerveDrive::GetInstance().GetPose2d();
    frc::ChassisSpeeds robotRelVelocity = SwerveDrive::GetInstance().GetVelocity2d();
    estimatedPose = estimatedPose.Exp(frc::Twist2d(robotRelVelocity.vx * latency, robotRelVelocity.vy * latency, robotRelVelocity.omega * latency));

    // Calculate distance from turret to target
    frc::Pose2d turretPosition = estimatedPose.TransformBy(Constants::kTurretOffset);
    units::meter_t distanceToGoal = goalPosition.Distance(turretPosition.Translation());

    // Calculate field relative turret velocity
    frc::ChassisSpeeds robotVelocity = frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotRelVelocity, estimatedPose.Rotation());
    double robotAngle = estimatedPose.Rotation().Radians().value();
    units::meters_per_second_t turretVelX = robotVelocity.vx; // + robotVelocity.omega * (Constants::kTurretOffset.Y() * cos(robotAngle) - Constants::kTurretOffset.X() * sin(robotAngle));
    units::meters_per_second_t turretVelY = robotVelocity.vy; // + robotVelocity.omega * (Constants::kTurretOffset.X() * cos(robotAngle) - Constants::kTurretOffset.Y() * sin(robotAngle));

    // Time of flight
    units::second_t timeOfFlight;
    frc::Pose2d lookAheadPose = turretPosition;
    units::meter_t lookAheadDistanceToGoal = distanceToGoal;
    for (int i = 0; i < 20; i++) {
        timeOfFlight = units::second_t{Constants::kDist2ToFA * pow(lookAheadDistanceToGoal.value(), Constants::kDist2ToFB)}; // Fudge added at comp, need to retune
        units::meter_t offSetX = turretVelX * timeOfFlight;
        units::meter_t offSetY = turretVelY * timeOfFlight;
        lookAheadPose = {turretPosition.Translation() + frc::Translation2d(offSetX, offSetY), turretPosition.Rotation()};
        lookAheadDistanceToGoal = goalPosition.Distance(lookAheadPose.Translation());
    }
    
    // Calculate parameters accounting for imparted velocity
    units::degree_t goalAngle = ((goalPosition - lookAheadPose.Translation()).Angle().Degrees() - estimatedPose.Rotation().Degrees()) + 180_deg;
    if (goalAngle < 0_deg) {
        // All commands are positive
        goalAngle += 360_deg;
    }

    m_turretAngle = angleFilter.Calculate(units::degree_t{std::fmod(goalAngle.value(), 360.0)});
    m_shooterVelocity = velocityFilter.Calculate(units::turns_per_second_t{Constants::kDist2TPSA * 0.98 * pow(lookAheadDistanceToGoal.value(), Constants::kDist2TPSB)}); // Fudge added at comp, need to retune
}

// Shooter wheel velocity getter (RPS or Rev/s)
units::turns_per_second_t ShotCalculator::GetShooterVelocity() {
    return m_shooterVelocity;
}

// Turret angel getter (deg)
units::degree_t ShotCalculator::GetTurretAngle() {
    return m_turretAngle;
}

// Turret angle setter
void ShotCalculator::SetTurretAngle(units::degree_t angle) {
    m_turretAngle = angle;
}

bool ShotCalculator::ShotValid() {
    return m_turretAngle.value() < Constants::kMaxShooterAzimuth && 
    m_shooterVelocity.value() > Constants::kMinShooterCal && 
    m_shooterVelocity.value() < Constants::kMaxShooterCal;
}

// Shooter wheel velocity setter
void ShotCalculator::SetShooterVelocity(units::turns_per_second_t velocity) {
    m_shooterVelocity = velocity;
}

// Constructor
ShotCalculator::ShotCalculator() {}