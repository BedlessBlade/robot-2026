#pragma once

#include <frc/geometry/Pose2d.h>
#include <vector>

#include "Constants.h"
#include "Task.h"

// Updated follow path task with support for per point velocity (m/s) and acceleration (m/s^2) limits
class FollowPath2 : public Task {
public:
    // Path, reset robot pose to first pose in vector, persist at the end of the path
    FollowPath2(std::vector<std::tuple<frc::Pose2d, double, double, double, double>> points, bool resetPose = false, bool persist = false);

    // Standard task functions
    void Start(double t) override;
    void Update(double t) override;
    void Stop() override;
    bool IsDone() const override;

private:
    // Path point (Pose2d), maxVel (m/s), maxAccel (M/s^2), position threshold (m), heading threshold (rad)
    std::vector<std::tuple<frc::Pose2d, double, double, double, double>> m_points;
    bool m_resetPose, m_persist;
    int m_pointIndex = 0;
    bool m_started = false;
};
