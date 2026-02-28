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
  photon::PhotonCamera m_leftCamera{"Tag_Camera_02"};
  photon::PhotonCamera m_rightCamera{"Tag_Camera_01"};

  photon::PhotonPoseEstimator m_leftPoseEstimator{
      frc::AprilTagFieldLayout::LoadField(
          frc::AprilTagField::k2025ReefscapeWelded),
      photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kLeftCameraTransform};
  photon::PhotonPoseEstimator m_rightPoseEstimator{
      frc::AprilTagFieldLayout::LoadField(
          frc::AprilTagField::k2025ReefscapeWelded),
      photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kRightCameraTransform};

  AlgaeState m_algaeState;

  Cameras();
};
