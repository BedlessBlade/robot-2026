#pragma once

#include <frc/geometry/Pose2d.h>
#include <vector>

#include "Constants.h"
#include "Task.h"

// Updated follow path task with support for per point velocity (m/s) and acceleration (m/s^2) limits
class FollowPath2 : public Task {
public:
    FollowPath2(std::vector<std::tuple<frc::Pose2d, double, double, double>> points, bool resetPose = false, bool persist = false);

    void Start(double t) override;
    void Update(double t) override;
    void Stop() override;
    bool IsDone() const override;

private:
    std::vector<std::tuple<frc::Pose2d, double, double, double>> m_points;
    bool m_resetPose, m_persist;
    int m_pointIndex = 0;
    bool m_started = false;
};
