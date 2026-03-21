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
void ShotCalculator::CalculateShotParams(frc::Translation2d goalPosition,
                                         units::second_t latency) {

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
    units::meters_per_second_t turretVelX = robotVelocity.vx; // + robotVelocity.omega * (Constants::kTurretOffset.Y() * std::cos );
    units::meters_per_second_t turretVelY = robotVelocity.vy; // + robotVelcoity.omega

    // Time of flight
    units::second_t timeOfFlight;
    frc::Pose2d lookAheadPose = turretPosition;
    units::meter_t lookAheadDistanceToGoal = distanceToGoal;
    for (int i = 0; i < 20; i++) {
        timeOfFlight = units::second_t{Constants::kDist2ToFA * pow(lookAheadDistanceToGoal.value(), Constants::kDist2ToFB)};
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

    m_turretAngle = units::degree_t{std::fmod(goalAngle.value(), 360.0)};
    m_shooterVelocity = units::turns_per_second_t{Constants::kDist2TPSA * 0.98 * pow(lookAheadDistanceToGoal.value(), Constants::kDist2TPSB)};

    // latency compensation
    //frc::Translation2d futurePosition = robotPosition.operator+(robotVelocity.operator*(latency.value()));
    //frc::Rotation2d futureHeading = {robotHeading.operator+(robotAngVelocity * latency)};
    
    // Turret Position
    //frc::Translation2d turretPosition = futurePosition.operator+(Constants::kTurretOffset.RotateBy(futureHeading));
    
    // Turret Velocity
    //frc::Translation2d turretVelocityAng = {-robotAngVelocity.value() * Constants::kTurretOffset.X(), robotAngVelocity.value() * Constants::kTurretOffset.Y()}; // Cross product Omega X turretOffset
    //frc::Translation2d turretVelocity = robotVelocity.operator+(turretVelocityAng);

    // Get target vector
    //frc::Translation2d toGoal = goalPosition.operator-(turretPosition);
    //units::length::meter_t distanceToGoal = toGoal.Norm();
    //frc::Translation2d toGoalNormalized = toGoal.operator/(distanceToGoal.value());

    // TEMP non-SOM implementation
    // units::degree_t goalAngle = toGoalNormalized.Angle().Degrees();
    // if (robotHeading.Degrees() < 0_deg) {
    //     if (goalAngle > 0_deg) {
    //         goalAngle -= 360_deg;
    //     }
    //     m_turretAngle = 180_deg - (robotHeading.Degrees() - goalAngle);

    // } else {
    //     if (goalAngle < 0_deg) {
    //         goalAngle += 360_deg;
    //     }
    //     m_turretAngle = (goalAngle - robotHeading.Degrees()) + 180_deg;
    // }

    // m_shooterVelocity = units::turns_per_second_t{Constants::kDist2TPSA * pow(distanceToGoal.value(), Constants::kDist2TPSB)}; // Convert dist to TPS

    // Use LUT to find baseline (i.e., 0 robot velocity) shot params
    //units::second_t timeOfFlight = units::second_t{Constants::kDist2ToFA * pow(distanceToGoal.value(), Constants::kDist2ToFB)}; // Convert dist to ToF
    //units::meters_per_second_t baselineShotVelocity = distanceToGoal / timeOfFlight; // Shot velocity

    // Get target velocity vector
    //frc::Translation2d targetVelocity = toGoalNormalized.operator*(baselineShotVelocity.value()); // target velocity vector (m/s)

    // Get shot velocity vector
    //frc::Translation2d shotVelocity = turretVelocity.operator-(targetVelocity);

    // Extract shooter params
    //units::degree_t goalAngle = (shotVelocity.Angle().Degrees() - robotHeading.Degrees()) + 0_deg;
    //if (goalAngle < 0_deg) {
        // All commands are positive
    //    goalAngle += 360_deg;
    //}

    //m_turretAngle = units::degree_t{std::fmod(goalAngle.value(), 360.0)};

    // use LUT to find the shooter RPS
    //units::meters_per_second_t requiredVelocity = units::meters_per_second_t{shotVelocity.Norm().value()};
    //units::meter_t effectiveDistance = units::meter_t{Constants::kVel2DistA * pow(requiredVelocity.value(), Constants::kVel2DistB)};// Convert vel to dist
    //m_shooterVelocity = units::turns_per_second_t{Constants::kDist2TPSA * pow(effectiveDistance.value(), Constants::kDist2TPSB)}; // Convert dist to TPS
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
    return m_turretAngle.value() < Constants::kMaxShooterAzimuth && m_shooterVelocity.value() > Constants::kMinShooterCal && m_shooterVelocity.value() < Constants::kMaxShooterCal;
}

// Shooter wheel velocity setter
void ShotCalculator::SetShooterVelocity(units::turns_per_second_t velocity) {
    m_shooterVelocity = velocity;
}

// Constructor
ShotCalculator::ShotCalculator() {}