#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/translation2d.h>
#include <wpi/interpolating_map.h>
#include <vector>
#include "Robot.h"
#include <wpi/interpolating_map.h>

// Shot calculator class
class ShotCalculator {
public:
    // Singleton format getinstance function
    static ShotCalculator &GetInstance() {
        static ShotCalculator instance;
        return instance;
    }

    //shooter getters for turret Azimuth angle (degrees) and shooter angular velocity (Rev/s)
    void CalculateShotParams(frc::Translation2d goalPosition,
                            units::second_t latency);
    
    // Getters
    units::degree_t GetTurretAngle();
    units::turns_per_second_t GetShooterVelocity();

    // Setters, used to override shooter
    void SetTurretAngle(units::degree_t angle);
    void SetShooterVelocity(units::turns_per_second_t velocity);

    bool ShotValid();

private:
    // Private constructor
    ShotCalculator();

    //Shot Parameters
    units::degree_t m_turretAngle;
    units::turns_per_second_t m_shooterVelocity;
};