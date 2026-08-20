#include "auto/FollowPath2.h"

#include "Constants.h"
#include "systems/SwerveDrive.h"
#include <iostream>

FollowPath2::FollowPath2(std::vector<std::tuple<frc::Pose2d, double, double, double, double>> points, bool resetPose, bool persist) : 
    m_points{points}, 
    m_resetPose{resetPose}, 
    m_persist{persist} {}

void FollowPath2::Start(double t) {
    // Reset pose and drivetrain controllers
    if (m_resetPose && m_points.size() > 0) {
        SwerveDrive::GetInstance().ResetPose(std::get<0>(m_points[0]));
        SwerveDrive::GetInstance().RestControllers();
    }

    // Advance point index
    if (m_points.size() > 1) {
        m_pointIndex++;
    }

    // Start path
    m_started = true;
}

void FollowPath2::Update(double t) {
    // Advance point index when at point
    if (SwerveDrive::GetInstance().AtPositionSetpoint() && m_pointIndex < m_points.size() - 1) {
        m_pointIndex++;
    }

    // Command drivetrain
    SwerveDrive::GetInstance().DriveToPose(std::get<0>(m_points[m_pointIndex]), 
                                           std::get<0>(m_points[m_pointIndex + 1]), 
                                           std::get<0>(m_points[m_pointIndex + 2]), 
                                           std::get<1>(m_points[m_pointIndex + 1]), 
                                           std::get<2>(m_points[m_pointIndex + 1]));


}

void FollowPath2::Stop() {
    if (!m_persist) {
        SwerveDrive::GetInstance().DriveVelocity(0, 0, 0);
    }
}

bool FollowPath2::IsDone() const {
    // return m_started && !m_persist && AtPoint() && SwerveDrive::GetInstance().VelocityMagnitude() < Constants::kPathFollowingVelocityTolerance;
    return true;
}