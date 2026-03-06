#include "systems/ShotCalculator.h"
#include "Constants.h"
#include <frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <vector>
#include <wpi/interpolating_map.h>


// Call to calculate shot parameters
void ShotCalculator::CalculateShotParams(frc::Pose2d robotPosition, frc::Translation2d robotVelocity, frc::Translation2d goalPosition, double latency) {
    // latency compensation
    frc::Pose2d futurePosition = {robotPosition.Translation().operator+(robotVelocity.operator*(latency)), robotPosition.Rotation()};                  //project future position
    
    // Get target vector
    frc::Translation2d toGoal = goalPosition.operator-(futurePosition.Translation());                                             // Vector from future robot position to the goal position
    units::length::meter_t distanceToGoal = toGoal.Norm();     
    frc::Pose2d turretOffset = {9.5_m, 4_m, 0_deg};                                                     // Distance to goal
    frc::Pose2d turretPosition = turretOffset.RotateAround(futurePosition.Translation(), futurePosition.Rotation());
    futurePosition.RotateBy(futurePosition.Rotation());                                      // Distance to goal
    
    

    frc::Translation2d toGoalNormalized = toGoal.operator/(distanceToGoal.value());                                 // Normalize toGoal vector

    // Use LUT to find baseline (i.e., 0 robot velocity) shot params
    // double timeOfFlight = dist2tofTable.get(distanceToGoal);                                                        // Use lookup table to find time of flight
    // double baselineShotVelocity = distanceToGoal.value() / timeOfFlight;                                            // Shot velocity (m/s)

    // Get target velocity vector
    // frc::Translation2d targetVelocity = toGoalNormalized.operator*(baselineShotVelocity);                           // target velocity vector (m/s)

    // Get shot velocity vector
    // frc::Translation2d shotVelocity = targetVelocity.operator-(robotVelocity);

    // Extract shooter params
    // m_turretAngle = shotVelocity.Angle().Degrees().value();
    // double requiredVelocity = shotVelocity.Norm().value();

    // use LUT to find the shooter RPS
    // double effectiveDistance = vel2distTable.get(requiredVelocity);
    // m_shooterVelocity = dist2RpsTable.get(effectiveDistance);
}

// Shooter wheel velocity getter (RPS or Rev/s)
double ShotCalculator::GetShooterVelocity() {
    return m_shooterVelocity;
}

// Turret angel getter (deg)
double ShotCalculator::GetTurretAngle() {
    return m_turretAngle;
}

// Constructor
ShotCalculator::ShotCalculator() {}