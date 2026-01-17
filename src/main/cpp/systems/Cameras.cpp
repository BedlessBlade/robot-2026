#include "systems/Cameras.h"

#include <algorithm>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>

#include "systems/SwerveDrive.h"

void Cameras::Update(Robot::Mode mode, double t) {
  // Front camera
  auto results = m_frontCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    auto pose = m_frontPoseEstimator.Update(results[0]);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(
          pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }

  // Back camera
  results = m_backCamera.GetAllUnreadResults();
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

    auto pose = m_backPoseEstimator.Update(result);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(
          pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
      break;
    }
  }
}

Cameras::AlgaeState Cameras::GetAlgaeState() const { return m_algaeState; }

Cameras::Cameras() {}
