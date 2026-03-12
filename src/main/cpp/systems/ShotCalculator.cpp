#include "systems/ShotCalculator.h"
#include "Constants.h"

#include <frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <vector>
#include <wpi/interpolating_map.h>
#include <iostream>
#include <math.h>


// Call to calculate shot parameters
void ShotCalculator::CalculateShotParams(frc::Translation2d robotPosition, 
                                        frc::Translation2d robotVelocity, 
                                        frc::Translation2d goalPosition,
                                        frc::Rotation2d robotHeading,
                                        units::degrees_per_second_t robotAngVelocity,
                                        units::second_t latency) {

    // latency compensation
    frc::Translation2d futurePosition = robotPosition.operator+(robotVelocity.operator*(latency.value()));
    frc::Rotation2d futureHeading = {robotHeading.operator+(robotAngVelocity * latency)};
    
    // Turret Position
    frc::Translation2d turretPosition = futurePosition.operator+(Constants::kTurretOffset.RotateBy(futureHeading));
    
    // Turret Velocity
    frc::Translation2d turretVelocityAng = {-robotAngVelocity.value() * Constants::kTurretOffset.X(), robotAngVelocity.value() * Constants::kTurretOffset.Y()}; // Cross product Omega X turretOffset
    frc::Translation2d turretVelocity = robotVelocity.operator+(turretVelocityAng);

    // Get target vector
    frc::Translation2d toGoal = goalPosition.operator-(turretPosition);
    units::length::meter_t distanceToGoal = toGoal.Norm();
    frc::Translation2d toGoalNormalized = toGoal.operator/(distanceToGoal.value());

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
    units::second_t timeOfFlight = units::second_t{Constants::kDist2ToFA * pow(distanceToGoal.value(), Constants::kDist2ToFB)}; // Convert dist to ToF
    units::meters_per_second_t baselineShotVelocity = distanceToGoal / timeOfFlight; // Shot velocity

    // Get target velocity vector
    frc::Translation2d targetVelocity = toGoalNormalized.operator*(baselineShotVelocity.value()); // target velocity vector (m/s)

    // Get shot velocity vector
    frc::Translation2d shotVelocity = targetVelocity.operator-(turretVelocity);

    // Extract shooter params
    units::degree_t goalAngle = shotVelocity.Angle().Degrees();
    if (robotHeading.Degrees() < 0_deg) {
        if (goalAngle > 0_deg) {
            goalAngle -= 360_deg;
        }
        m_turretAngle = 180_deg - (robotHeading.Degrees() - goalAngle);

    } else {
        if (goalAngle < 0_deg) {
            goalAngle += 360_deg;
        }
        m_turretAngle = (goalAngle - robotHeading.Degrees()) + 180_deg;
    }
    units::meters_per_second_t requiredVelocity = units::meters_per_second_t{shotVelocity.Norm().value()};

    // use LUT to find the shooter RPS
    units::meter_t effectiveDistance = units::meter_t{Constants::kVel2DistA * pow(requiredVelocity.value(), Constants::kVel2DistB)};// Convert vel to dist
    m_shooterVelocity = units::turns_per_second_t{Constants::kDist2TPSA * pow(effectiveDistance.value(), Constants::kDist2TPSB)}; // Convert dist to TPS
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
    m_turretAngle = -angle;
}

// Shooter wheel velocity setter
void ShotCalculator::SetShooterVelocity(units::turns_per_second_t velocity) {
    m_shooterVelocity = velocity;
}

// Constructor
ShotCalculator::ShotCalculator() {}