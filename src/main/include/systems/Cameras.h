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

  void Update(Robot::Mode mode, double t) override;

private:
  photon::PhotonCamera m_leftCamera{"Arducam_OV9281_USB_Camera"};
  photon::PhotonCamera m_rightCamera{"Tag_Camera_01"};
  photon::PhotonCamera m_backLeftCamera{"Tag_Camera_04"}; // Back left camera
  photon::PhotonCamera m_backRightCamera{"Tag_Camera_03"}; // Back right camera

  photon::PhotonPoseEstimator m_leftPoseEstimator{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark), photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kLeftCameraTransform};
  photon::PhotonPoseEstimator m_rightPoseEstimator{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark), photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kRightCameraTransform};
  photon::PhotonPoseEstimator m_backLeftPoseEstimator{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark), photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kBackLeftCameraTransform};
  photon::PhotonPoseEstimator m_backRightPoseEstimator{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark), photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kBackRightCameraTransform};

  Cameras();
};
