#include "systems/Cameras.h"

#include <algorithm>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>

#include "systems/SwerveDrive.h"

void Cameras::Update(Robot::Mode mode, double t) {
  // // Left camera
  // auto results = m_leftCamera.GetAllUnreadResults();
  // for (auto result : results) {
  //   // skip results with no targets
  //   if (!result.HasTargets()) {
  //     continue;
  //   }

  //   // skip results with high ambiguity
  //   if (result.GetBestTarget().GetPoseAmbiguity() > 0.2) {
  //     continue;
  //   }

  //   if (result.multitagResult.has_value()) {
  //     // calculate robot pose
  //     auto pose = m_leftPoseEstimator.EstimateCoprocMultiTagPose(result);

  //     // tags used
  //     auto tagsUsed = pose -> targetsUsed;

  //     // # of tags used
  //     double tagCount = tagsUsed.size();

  //     // calculate average tag distance
  //     units::length::meter_t averageDist {0_m};
  //     units::length::meter_t totalDist {0_m};
  //     for (auto tag : tagsUsed) {
  //       totalDist += tag.bestCameraToTarget.Translation().Norm();
  //     }

  //     averageDist = totalDist / (tagCount);
  //   }
  // }

  auto results = m_leftCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    for (auto result : results) {
      // Prefer multitag results
      auto pose = m_leftPoseEstimator.EstimateCoprocMultiTagPose(result);

      // Fall back to single tag estimate
      if (!pose.has_value()) {
        pose = m_leftPoseEstimator.EstimateLowestAmbiguityPose(result);
      }

      // Skip if no tags are visable
      if (!pose.has_value()) {
        continue;
      }

      // Skip single tag estimates with high ambiguity
      if (pose -> targetsUsed.size() == 1 && pose -> targetsUsed[0].GetPoseAmbiguity() > 0.2) {
        continue;
      }

      // Add pose
      SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    };
  };

  // Right camera
  results = m_rightCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    for (auto result : results) {
      // Prefer multitag results
      auto pose = m_rightPoseEstimator.EstimateCoprocMultiTagPose(result);

      // Fall back to single tag estimate
      if (!pose.has_value()) {
        pose = m_rightPoseEstimator.EstimateLowestAmbiguityPose(result);
      }

      // Skip if no tags are visable
      if (!pose.has_value()) {
        continue;
      }

      // Skip single tag estimates with high ambiguity
      if (pose -> targetsUsed.size() == 1 && pose -> targetsUsed[0].GetPoseAmbiguity() > 0.2) {
        continue;
      }

      // Add pose
      SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    };
  };

  // Back Left camera
  // results = m_backLeftCamera.GetAllUnreadResults();
  // if (results.size() > 0) {
  //   for (auto result : results) {
  //     // Prefer multitag results
  //     auto pose = m_backLeftPoseEstimator.EstimateCoprocMultiTagPose(result);

  //     // Fall back to single tag estimate
  //     if (!pose.has_value()) {
  //       pose = m_backLeftPoseEstimator.EstimateLowestAmbiguityPose(result);
  //     }

  //     // Skip if no tags are visable
  //     if (!pose.has_value()) {
  //       continue;
  //     }

  //     // Skip single tag estimates with high ambiguity
  //     if (pose -> targetsUsed.size() == 1 && pose -> targetsUsed[0].GetPoseAmbiguity() > 0.2) {
  //       continue;
  //     }

  //     // Add pose
  //     SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
  //   };
  // };

  // Back Right camera
  results = m_backRightCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    for (auto result : results) {
      // Prefer multitag results
      auto pose = m_backRightPoseEstimator.EstimateCoprocMultiTagPose(result);

      // Fall back to single tag estimate
      if (!pose.has_value()) {
        pose = m_backRightPoseEstimator.EstimateLowestAmbiguityPose(result);
      }

      // Skip if no tags are visable
      if (!pose.has_value()) {
        continue;
      }

      // Skip single tag estimates with high ambiguity
      if (pose -> targetsUsed.size() == 1 && pose -> targetsUsed[0].GetPoseAmbiguity() > 0.2) {
        continue;
      }

      // Add pose
      SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    };
  };
}

Cameras::Cameras() {}