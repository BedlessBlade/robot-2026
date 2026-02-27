#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Rotation2d.h>

#include "Constants.h"
#include "Controllers.h"
#include "Locations.h"
#include "Util.h"
#include "auto/AutoDoNothing.h"
#include "systems/Cameras.h"
#include "systems/SwerveDrive.h"
#include "systems/QuestNav.h"
#include "systems/LEDs.h"
#include "systems/Indexer.h"
#include "systems/Climber.h"
#include "systems/Shooter.h"

// This gets called first. So, initialize everything here.
Robot::Robot()
  : m_alignControllers{
      {Constants::kPathFollowingKp, Constants::kPathFollowingKi,
       Constants::kPathFollowingKd},
      {Constants::kPathFollowingKp, Constants::kPathFollowingKi,
       Constants::kPathFollowingKd},
      {Constants::kPathFollowingAngleKp, Constants::kPathFollowingAngleKi,
       Constants::kPathFollowingAngleKd}} {
  m_startChooser.SetDefaultOption("1", 1);
  m_startChooser.AddOption("2", 2);
  m_startChooser.AddOption("3", 3);
  frc::SmartDashboard::PutData("Start Location", &m_startChooser);

  frc::SmartDashboard::PutBoolean("Calibrate Pose", false);

  double rs_shooterTopSpeed = 0.0;
  double rs_shooterLowSpeed = 0.0;
        
  m_autoChooser.SetDefaultOption("Do Nothing", "DoNothing");
  //  m_autoChooser.AddOption("Cross the Line", "CrossLine");
  //  m_autoChooser.AddOption("One Coral", "OneCoral");
  //  m_autoChooser.AddOption("Two Coral", "TwoCoral");
  frc::SmartDashboard::PutData("Auto", &m_autoChooser);

  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("QuestNav Field", &m_QuestNavField);

  // Call GetInstance() so the constructors get called
  Cameras::GetInstance();
  SwerveDrive::GetInstance();
  Indexer::GetInstance();
  Climber::GetInstance();
  LEDs::GetInstance();
  LEDs::GetInstance().LEDsInit();
  Shooter::GetInstance();

  

  // This initializes the main looper. What you put here will run @200 Hz while
  // the robot is on.
  m_looper = Looper{[this] {
    Mode mode = kDisabled;
    if (IsEnabled()) {
      if (IsAutonomous()) {
        mode = kAuto;
      } else if (IsTeleop()) {
        mode = kTeleop;
      }
    }

    double t = frc::Timer::GetFPGATimestamp().value();
  

    if (mode == kAuto) {
      if (m_auto) {
        m_auto->Update(t);

        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
          globalAlliance = "Red";
        } else if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue) {
          globalAlliance = "Blue";
        } else {
          globalAlliance = "None";
        }
      }
    } else if (mode == kTeleop) {
      if (m_braking) {
        SwerveDrive::GetInstance().Coast();
        m_braking = false;
      }

      m_field.SetRobotPose(SwerveDrive::GetInstance().GetPose2d());
      
      isXButtonPressed = Controllers::GetInstance().GetDriverController().GetXButtonPressed();
      // Check this before sending drive velocities
      if (Controllers::GetInstance().GetDriverController().GetXButtonPressed()) {
        // Left coral scoring location
        ResetAlignControllers();
        m_autoAlignSetpoint = NearestLeftCoral(
            SwerveDrive::GetInstance().GetPose2d(), &m_autoAlignSetpointIndex);
        m_autoAlignMode = kPosition;
      } else if (Controllers::GetInstance()
                     .GetDriverController()
                     .GetBButtonPressed()) {
        // Right coral scoring location
        ResetAlignControllers();
        m_autoAlignSetpoint = NearestRightCoral(
            SwerveDrive::GetInstance().GetPose2d(), &m_autoAlignSetpointIndex);
        m_autoAlignMode = kPosition;
        SwerveDrive::GetInstance().DisableRamp();
      } else if (Controllers::GetInstance()
                     .GetDriverController()
                     .GetYButtonPressed()) {
        // Algae scoring location
        ResetAlignControllers();
        m_autoAlignSetpoint = NearestAlgae(
            SwerveDrive::GetInstance().GetPose2d(), &m_autoAlignSetpointIndex);
        m_autoAlignMode = kPosition;
        SwerveDrive::GetInstance().DisableRamp();
      } else if (Controllers::GetInstance()
                     .GetDriverController()
                     .GetAButtonPressed()) {
        // Feeder station location
        ResetAlignControllers();
        m_autoAlignSetpoint = NearestFeeder(
            SwerveDrive::GetInstance().GetPose2d(), &m_autoAlignSetpointIndex);
        m_autoAlignMode = kPosition;
        SwerveDrive::GetInstance().DisableRamp();
      } else if (Controllers::GetInstance()
                     .GetDriverController()
                     .GetRightBumperButtonPressed()) {
        // Feeder station location with only angle
        m_autoAlignSetpoint = NearestFeeder(
            SwerveDrive::GetInstance().GetPose2d(), &m_autoAlignSetpointIndex);
        m_autoAlignMode = kNoPosition;
        SwerveDrive::GetInstance().DisableRamp();
      } else if (Controllers::GetInstance()
                     .GetDriverController()
                     .GetAButtonReleased() ||
                 Controllers::GetInstance()
                     .GetDriverController()
                     .GetBButtonReleased() ||
                 Controllers::GetInstance()
                     .GetDriverController()
                     .GetYButtonReleased() ||
                 Controllers::GetInstance()
                     .GetDriverController()
                     .GetXButtonReleased() ||
                 Controllers::GetInstance()
                     .GetDriverController()
                     .GetRightBumperButtonReleased()) {
        m_autoAlignMode = kNone;
        SwerveDrive::GetInstance().EnableRamp();
      }

      // Get the inputs from the controller during teleop mode. Note this uses
      // the split setup where the left joystick controls velocity, and the
      // right joystick controls the rotation. The Util::exp() function squares
      // the input while keeping the sign.
      double leftY =
          Controllers::GetInstance().GetDriverController().GetLeftY();
      double vx = Util::Exp(-leftY) * Constants::kDriveControlMultipler;

      double leftX =
          Controllers::GetInstance().GetDriverController().GetLeftX();
      double vy = Util::Exp(-leftX) * Constants::kDriveControlMultipler;

      // GetRightX() doesn't work in the Linux simulation for some reason
      double rightX =
          Controllers::GetInstance().GetDriverController().GetRightX();
      double w = Util::Exp(-rightX) * Constants::kDriveAngularControlMultiplier;

      // The auto will reset the pose to be facing towards the driver on the red
      // alliance so it needs to be corrected
      auto alliance = frc::DriverStation::GetAlliance();

      if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        globalAlliance = "Red";
      } else if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue) {
        globalAlliance = "Blue";
      } else {
        globalAlliance = "None";
      }

      if (alliance.has_value() &&
          alliance.value() == frc::DriverStation::Alliance::kRed) {
        vx *= -1;
        vy *= -1;
      }

      if (Controllers::GetInstance().GetDriverController().GetRawButton(9) &&
          Controllers::GetInstance().GetDriverController().GetRawButton(10)) {
        if (leftX >= 0.5 && rightX <= -0.5) {
          if (alliance.has_value() &&
              alliance.value() == frc::DriverStation::Alliance::kRed) {
            SwerveDrive::GetInstance().ResetPose(frc::Pose2d{
                frc::Translation2d{}, frc::Rotation2d{units::radian_t{M_PI}}});
          } else {
            SwerveDrive::GetInstance().ResetPose(frc::Pose2d{});
          }
        }

        SwerveDrive::GetInstance().DriveVelocity(0, 0, 0);
      } else {
        if (m_autoAlignMode != kNone) {
          auto robotPose = SwerveDrive::GetInstance().GetPose2d();

          if (m_autoAlignMode != kNoPosition) {
            vx = m_alignControllers[0].Update(
                robotPose.Translation().X().value(),
                m_autoAlignSetpoint.Translation().X().value());
            if (vx < -Constants::kPathFollowingMaxV) {
              vx = -Constants::kPathFollowingMaxV;
            }
            if (vx > Constants::kPathFollowingMaxV) {
              vx = Constants::kPathFollowingMaxV;
            }

            vy = m_alignControllers[1].Update(
                robotPose.Translation().Y().value(),
                m_autoAlignSetpoint.Translation().Y().value());
            if (vy < -Constants::kPathFollowingMaxV) {
              vy = -Constants::kPathFollowingMaxV;
            }
            if (vy > Constants::kPathFollowingMaxV) {
              vy = Constants::kPathFollowingMaxV;
            }

            if (robotPose
             .Translation()
             .Distance(m_autoAlignSetpoint.Translation())
             .value() < Constants::kPathFollowingTolerance && SwerveDrive::GetInstance().VelocityMagnitude() <
             Constants::kPathFollowingVelocityTolerance) {
                Controllers::GetInstance().GetDriverController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
                Controllers::GetInstance().GetOperatorController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
             }
          }

          double angleSetpoint =
              m_autoAlignSetpoint.Rotation().Radians().value();
          if (angleSetpoint < -Constants::kPathFollowingMaxV) {
            angleSetpoint = -Constants::kPathFollowingMaxW;
          }
          if (vx > Constants::kPathFollowingMaxV) {
            angleSetpoint = Constants::kPathFollowingMaxW;
          }
          double currentAngle = robotPose.Rotation().Radians().value();
          double angleError = angleSetpoint - currentAngle;
          if (angleError > M_PI) {
            angleSetpoint -= 2 * M_PI;
          }

          w = m_alignControllers[2].Update(currentAngle, angleSetpoint);
        } else {
          Controllers::GetInstance().GetDriverController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
          Controllers::GetInstance().GetOperatorController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
        }

        SwerveDrive::GetInstance().DriveVelocity(vx, vy, w);
      }

      if (Controllers::GetInstance()
              .GetOperatorController()
              .GetAButtonPressed()) {
        
      } else if (Controllers::GetInstance()
                     .GetOperatorController()
                     .GetBButtonPressed()) {
        
      } else if (Controllers::GetInstance()
                     .GetOperatorController()
                     .GetXButtonPressed()) {
        
      } else if (Controllers::GetInstance()
                     .GetOperatorController()
                     .GetYButtonPressed()) {
        
      } else if (Controllers::GetInstance().GetOperatorController().GetPOV() ==
                 180) {
        
      } else if (Controllers::GetInstance().GetOperatorController().GetPOV() ==
                 0) {
        
      } else if (Controllers::GetInstance().GetOperatorController().GetPOV() ==
                 270) {
                  
      } else if (Controllers::GetInstance().GetOperatorController().GetPOV() ==
                 90) {
        
      } else if (Controllers::GetInstance()
                     .GetOperatorController()
                     .GetLeftTriggerAxis() > 0.5) {
        
      } else if (Controllers::GetInstance()
                     .GetOperatorController()
                     .GetRightTriggerAxis() > 0.5) {
                      //add Align shoot and turret

                      //remove this line later, add pose to top of code or whatever.
                      auto robotPose = SwerveDrive::GetInstance().GetPose2d();
                      int distToHub = sqrt(pow((Constants::kHubX - (robotPose.X() + (robotPose.Rotation().Cos() / Constants::kShooterOffsetDist))), 2) + 
                                   pow((Constants::kHubY - (robotPose.Y() + (robotPose.Rotation().Sin() / Constants::kShooterOffsetDist))), 2));
                      Shooter::GetInstance().SetMotorSpeed();

      } else if (Controllers::GetInstance()
                     .GetOperatorController()
                     .GetRightX() > 0.5) {

      } else if (Controllers::GetInstance()
                     .GetOperatorController()
                     .GetRightY() > 0.5) {
        
      } else if (Controllers::GetInstance()
                     .GetOperatorController()
                     .GetRightY() < -0.5) {
        
      } else {
        
      }
    } else if (mode == kDisabled) {
      if (std::abs(
              SwerveDrive::GetInstance().GetPose2d().Translation().X().value() -
              Constants::kFieldLength / 2) <= Constants::kBrakeDistance &&
          !m_braking) {
        SwerveDrive::GetInstance().Brake();
        m_braking = true;
      } else if (m_braking) {
        SwerveDrive::GetInstance().Coast();
        m_braking = false;
      }
    }

    QuestNav nav;
    frc::Pose2d QuestPose = nav.GetQuestPose();
    if (frc::SmartDashboard::GetBoolean("Calibrate Pose", false)) {
      nav.Calibrate();
      frc::SmartDashboard::PutBoolean("Calibrate Pose", false);
      std::cout << "Recalibrated" << std::endl;
    };
    
    m_QuestNavField.SetRobotPose(QuestPose);
    std::cout << "X:Y:Pitch : " << QuestPose.X().value() << ", "
                                << QuestPose.Y().value() << ", "
                                << QuestPose.Rotation().Degrees().value()
                                << std::endl;



    Cameras::GetInstance().Update(mode, t);
    SwerveDrive::GetInstance().Update(mode, t);

    LEDs::GetInstance().Update(mode, globalAlliance);
    Indexer::GetInstance().Update(mode);
  }};
}

// This destructor gets called when the robot program shuts down.
// Cleanup any resources (especially files) before the robot code gets
// restarted.
Robot::~Robot() {}

// Ensure this matches the declaration in Robot.h (typically: void DisabledInit();)
void Robot::DisabledInit() {
}

void Robot::DisabledExit() {
  SwerveDrive::GetInstance().Coast();
  m_braking = false;

  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value()) {
    Locations::GetInstance().Generate(alliance.value());

    frc::Pose2d start = Locations::GetInstance().GetStartPosition(
        alliance.value(), m_startChooser.GetSelected());
    SwerveDrive::GetInstance().ResetPose(start);

    std::string autoName = m_autoChooser.GetSelected();
        double t = frc::Timer::GetFPGATimestamp().value();
        m_auto = std::make_shared<AutoDoNothing>();
        m_auto->Start(t);
  }
}

void Robot::TeleopInit() {
  // Make sure that ramping is enabled for the driver motors even if auto is
  // incomplete
  SwerveDrive::GetInstance().EnableRamp();
}

frc::Pose2d Robot::NearestLeftCoral(frc::Pose2d robotPose, int *i) {
  frc::Pose2d nearest = Locations::GetInstance().GetCoralPositions()[0];
  auto minDistance = robotPose.Translation().Distance(nearest.Translation());
  if (i) {
    *i = 0;
  }

  for (int j = 2; j < 12; j += 2) {
    auto distance = robotPose.Translation().Distance(
        Locations::GetInstance().GetCoralPositions()[j].Translation());
    if (distance < minDistance) {
      nearest = Locations::GetInstance().GetCoralPositions()[j];
      minDistance = distance;

      if (i) {
        *i = j;
      }
    }
  }

  return nearest;
}

frc::Pose2d Robot::NearestRightCoral(frc::Pose2d robotPose, int *i) {
  frc::Pose2d nearest = Locations::GetInstance().GetCoralPositions()[1];
  auto minDistance = robotPose.Translation().Distance(nearest.Translation());
  if (i) {
    *i = 1;
  }

  for (int j = 3; j < 12; j += 2) {
    auto distance = robotPose.Translation().Distance(
        Locations::GetInstance().GetCoralPositions()[j].Translation());
    if (distance < minDistance) {
      nearest = Locations::GetInstance().GetCoralPositions()[j];
      minDistance = distance;

      if (i) {
        *i = j;
      }
    }
  }

  return nearest;
}

frc::Pose2d Robot::NearestAlgae(frc::Pose2d robotPose, int *i) {
  frc::Pose2d nearest = Locations::GetInstance().GetAlgaePositions()[0];
  auto minDistance = robotPose.Translation().Distance(nearest.Translation());
  if (i) {
    *i = 0;
  }

  for (int j = 1; j < 6; j++) {
    auto distance = robotPose.Translation().Distance(
        Locations::GetInstance().GetAlgaePositions()[j].Translation());
    if (distance < minDistance) {
      nearest = Locations::GetInstance().GetAlgaePositions()[j];
      minDistance = distance;

      if (i) {
        *i = j;
      }
    }
  }

  return nearest;
}

frc::Pose2d Robot::NearestFeeder(frc::Pose2d robotPose, int *i) {
  frc::Pose2d nearest = Locations::GetInstance().GetFeederPositions()[0];
  auto minDistance = robotPose.Translation().Distance(nearest.Translation());
  if (i) {
    *i = 0;
  }

  for (int j = 1; j < 6; j++) {
    auto distance = robotPose.Translation().Distance(
        Locations::GetInstance().GetFeederPositions()[j].Translation());
    if (distance < minDistance) {
      nearest = Locations::GetInstance().GetFeederPositions()[j];
      minDistance = distance;

      if (i) {
        *i = j;
      }
    }
  }

  return nearest;
}




#ifndef RUNNING_FRC_TESTS
int main(int argc, char **argv) { frc::StartRobot<Robot>(); }
#endif
