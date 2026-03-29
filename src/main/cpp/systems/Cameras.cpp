#include "systems/Cameras.h"

#include <algorithm>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>

#include "systems/SwerveDrive.h"

void Cameras::Update(Robot::Mode mode, double t) {
  // Left camera
  auto results = m_leftCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    auto pose = m_leftPoseEstimator.Update(results[0]);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }

  // Right camera
  auto results = m_rightCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    auto pose = m_rightPoseEstimator.Update(results[0]);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }

  // Back left camera
  auto results = m_backLeftCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    auto pose = m_backLeftPoseEstimator.Update(results[0]);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }

  // Back right camera
  auto results = m_backRightCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    auto pose = m_backRightPoseEstimator.Update(results[0]);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }
}

Cameras::Cameras() {}
