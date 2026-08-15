#include "auto/FollowPath2.h"

#include "Constants.h"
#include "systems/SwerveDrive.h"
#include <iostream>

FollowPath2::FollowPath2(std::vector<std::tuple<frc::Pose2d, double, double, double, double>> points, bool resetPose, bool persist) : 
    m_points{points}, 
    m_resetPose{resetPose}, 
    m_persist{persist} {}

void FollowPath2::Start(double t) {
    if (m_resetPose && m_points.size() > 0) {
        SwerveDrive::GetInstance().ResetPose(std::get<0>(m_points[0]));
    }

    if (m_points.size() > 1) {
        m_pointIndex++;
    }

    m_started = true;
}

void FollowPath2::Update(double t) {
    // if (AtPoint() && m_pointIndex < m_points.size() - 1) {
    //     m_pointIndex++;
    // }
}

void FollowPath2::Stop() {
    // SwerveDrive::GetInstance().DriveVelocity(0, 0, 0);
    // //SwerveDrive::GetInstance().EnableRamp();
}

bool FollowPath2::IsDone() const {
    // return m_started && !m_persist && AtPoint() && SwerveDrive::GetInstance().VelocityMagnitude() < Constants::kPathFollowingVelocityTolerance;
    return true;
}