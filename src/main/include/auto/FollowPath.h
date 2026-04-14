#pragma once

#include <frc/geometry/Pose2d.h>
#include <vector>

#include "Constants.h"
#include "Task.h"
#include "control/PIDController.h"

class FollowPath : public Task {
public:
  // todo: reorder NEXT YEAR
  FollowPath(std::vector<frc::Pose2d> points, bool resetPose = false,
             bool persist = false);

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;
  bool IsDone() const override;

private:
  std::vector<frc::Pose2d> m_points;
  bool m_resetPose, m_persist, m_slow;
  int m_pointIndex = 0;
  bool m_started = false;

  PIDController m_controllers[3]{
      {Constants::kPathFollowingKp, Constants::kPathFollowingKi,
       Constants::kPathFollowingKd, Constants::kPathFollowingTau},
      {Constants::kPathFollowingKp, Constants::kPathFollowingKi,
       Constants::kPathFollowingKd, Constants::kPathFollowingTau},
      {Constants::kPathFollowingAngleKp, Constants::kPathFollowingAngleKi,
       Constants::kPathFollowingAngleKd, Constants::kPathFollowingTau}};

  bool AtPoint() const;
};
