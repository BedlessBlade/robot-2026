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

  // // Left camera
  // auto results = m_leftCamera.GetAllUnreadResults();
  // if (results.size() > 0) {
  //   auto pose = m_leftPoseEstimator.Update(results[0]);

  //   if (pose.has_value()) {
  //     SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
  //   }
  // }

  // // Right camera
  // results = m_rightCamera.GetAllUnreadResults();
  // if (results.size() > 0) {
  //   auto pose = m_rightPoseEstimator.Update(results[0]);

  //   if (pose.has_value()) {
  //     SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
  //   }
  // }

  // // Back left camera
  // results = m_backLeftCamera.GetAllUnreadResults();
  // if (results.size() > 0) {
  //   auto pose = m_backLeftPoseEstimator.Update(results[0]);

  //   if (pose.has_value()) {
  //     SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
  //   }
  // }

  // // Back right camera
  // results = m_backRightCamera.GetAllUnreadResults();
  // if (results.size() > 0) {
  //   auto pose = m_backRightPoseEstimator.Update(results[0]);

  //   if (pose.has_value()) {
  //     SwerveDrive::GetInstance().VisionUpdate(pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
  //   }
  // }
}

Cameras::Cameras() {}
