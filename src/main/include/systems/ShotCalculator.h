#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/translation2d.h>
#include <wpi/interpolating_map.h>
#include <vector>
#include "Robot.h"

// Shot calculator class
class ShotCalculator {
public:
    // Singleton format getinstance function
    static ShotCalculator &GetInstance() {
        static ShotCalculator instance;
        return instance;
    }

    //shooter getters for turret Azimuth angle (degrees) and shooter angular velocity (Rev/s)
    void CalculateShotParams(frc::Translation2d robotPosition, frc::Translation2d robotVelocity, frc::Translation2d goalPosition, double latency);
    double GetTurretAngle();
    double GetShooterVelocity();
  

private:
    // Private constructor
    ShotCalculator();

    //Shot Parameters
    double m_turretAngle, m_shooterVelocity;

};
