#pragma once

#include <frc/Compressor.h>
#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <memory>

#include "Constants.h"
#include "Looper.h"
#include "auto/Task.h"
#include "control/PIDController.h"

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>

#include <units/time.h>
#include <units/velocity.h>

class Robot : public frc::TimedRobot {
public:
  // Enum to specify the robot's mode to the sub-systems
  enum Mode { kDisabled, kAuto, kTeleop };

  enum AutoAlignMode { kNone, kRamp };

  Robot();
  ~Robot();

  void DisabledInit() override;
  void DisabledExit() override;
  void TeleopInit() override;

  bool HubActive();

  bool isXButtonPressed = false;

private:

  frc::Compressor m_compressor;

  Looper m_looper;

  frc::SendableChooser<int> m_startChooser;
  frc::SendableChooser<std::string> m_autoChooser;
  frc::SendableChooser<int> m_autoEndChooser;

  std::shared_ptr<Task> m_auto;

  frc::Field2d m_field;
  std::string m_positionString;
  char m_alliance = 'N';

  AutoAlignMode m_autoAlignMode = kNone;
  frc::Pose2d m_autoAlignSetpoint;
  int m_autoAlignSetpointIndex = 0;
  PIDController m_alignControllers[3];

  frc::Translation2d m_goalPosition;
  frc::Pose2d m_currentPose;
  frc::Translation2d m_currentVelocity;
  units::degrees_per_second_t m_currentAngularVelocity;

  bool m_braking = false;
  bool m_intaking = false;

  inline void ResetAlignControllers() {
    for (int i = 0; i < 3; i++) {
      m_alignControllers[i].Reset();
    }
  }
};