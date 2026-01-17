#pragma once

#include <memory>
#include <photon/PhotonPoseEstimator.h>

#include "Constants.h"
#include "System.h"

class Cameras : public System {
public:
  static Cameras &GetInstance() {
    static Cameras instance;
    return instance;
  }

  enum AlgaeState { kNone, kBottom, kTop, kBoth };

  AlgaeState GetAlgaeState() const;
  void Update(Robot::Mode mode, double t) override;

private:
  photon::PhotonCamera m_frontCamera{"Tag_Camera_02"};
  photon::PhotonCamera m_backCamera{"Tag_Camera_01"};

  photon::PhotonPoseEstimator m_frontPoseEstimator{
      frc::AprilTagFieldLayout::LoadField(
          frc::AprilTagField::k2025ReefscapeWelded),
      photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kFrontCameraTransform};
  photon::PhotonPoseEstimator m_backPoseEstimator{
      frc::AprilTagFieldLayout::LoadField(
          frc::AprilTagField::k2025ReefscapeWelded),
      photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kBackCameraTransform};

  AlgaeState m_algaeState;

  Cameras();
};
