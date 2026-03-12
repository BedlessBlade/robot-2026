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
      SwerveDrive::GetInstance().VisionUpdate(
          pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }

  // Right camera
  results = m_rightCamera.GetAllUnreadResults();
  for (auto result : results) {
    bool blockedTag = false;
    for (auto target : result.GetTargets()) {
      if (std::find(Constants::kBlockedTags.begin(),
                    Constants::kBlockedTags.end(),
                    target.GetFiducialId()) != Constants::kBlockedTags.end()) {
        blockedTag = true;
        break;
      }
    }
    if (blockedTag) {
      continue;
    }

    auto pose = m_rightPoseEstimator.Update(result);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(
          pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
      break;
    }
  }
}

Cameras::Cameras() {}
