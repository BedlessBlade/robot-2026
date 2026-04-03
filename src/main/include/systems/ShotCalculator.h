#pragma once

#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/translation2d.h>
#include <wpi/interpolating_map.h>
#include <vector>
#include <frc/filter/LinearFilter.h>

// Shot calculator class
class ShotCalculator {
public:
    // Singleton format getinstance function
    static ShotCalculator &GetInstance() {
        static ShotCalculator instance;
        return instance;
    }

    //shooter getters for turret Azimuth angle (degrees) and shooter angular velocity (Rev/s)
    void CalculateShotParams(frc::Translation2d goalPosition, units::second_t latency);
    
    // Getters
    units::degree_t GetTurretAngle();
    units::degrees_per_second_t GetTurretVelocity();
    units::turns_per_second_t GetShooterVelocity();

    // Setters, used to override shooter
    void SetTurretAngle(units::degree_t angle);
    void SetShooterVelocity(units::turns_per_second_t velocity);

    bool ShotValid();

private:
    // Private constructor
    ShotCalculator();

    // Filters
    frc::LinearFilter<units::degree_t> angleFilter = frc::LinearFilter<units::degree_t>::SinglePoleIIR(0.05, 0.005_s);
    frc::LinearFilter<units::degrees_per_second_t> velocityFilter = frc::LinearFilter<units::degrees_per_second_t>::SinglePoleIIR(0.05, 0.005_s);

    //Shot Parameters
    units::degree_t m_turretAngle;
    units::degree_t m_lastTurretAngle;
    units::degrees_per_second_t m_turretVelocity; 
    units::turns_per_second_t m_shooterVelocity;
};