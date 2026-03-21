#include "Robot.h"
#include <cmath> 

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory>
#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Rotation2d.h>

#include "Constants.h"
#include "Controllers.h"
#include "Locations.h"
#include "Util.h"

//#include "systems/Intake.h" -- Not removing any just commenting for SupaIntake testing
#include "systems/SupaIntake.h"
#include "systems/Cameras.h"
#include "systems/SwerveDrive.h"
#include "systems/QuestNav.h"
#include "systems/LEDs.h"
#include "systems/Indexer.h"
#include "systems/Climber.h"
#include "systems/Shooter.h"
#include "systems/ShotCalculator.h"

#include "auto/AutoDoNothing.h"
#include "auto/AutoDepot.h"
#include "auto/AutoOutpost.h"
#include "auto/AutoCenter.h"
#include "auto/AutoShoot.h"
#include "auto/AutoShootBackUp.h"


// This gets called first. So, initialize everything here.
Robot::Robot()
  : m_compressor{frc::PneumaticsModuleType::REVPH},
    m_alignControllers{
      {Constants::kPathFollowingKp, Constants::kPathFollowingKi,
       Constants::kPathFollowingKd},
      {Constants::kPathFollowingKp, Constants::kPathFollowingKi,
       Constants::kPathFollowingKd},
      {Constants::kPathFollowingAngleKp, Constants::kPathFollowingAngleKi,
       Constants::kPathFollowingAngleKd}} {
  m_startChooser.SetDefaultOption("1", 1);
  m_startChooser.AddOption("2", 2);
  m_startChooser.AddOption("3", 3);
  m_startChooser.AddOption("4", 4);
  m_startChooser.AddOption("5", 5);

  m_autoChooser.SetDefaultOption("Do Nothing", "DoNothing");
  m_autoChooser.AddOption("To Depot", "Outpost");
  m_autoChooser.AddOption("To Outpost", "Outpost");
  m_autoChooser.AddOption("To Neutral Zone", "Center");
  m_autoChooser.AddOption("Shoot", "Shoot");
  m_autoChooser.AddOption("Shoot & Back Up", "ShootBackUp");
  // m_autoChooser.AddOption("To Neutral & Depot", "CenterDepot");
  

  frc::SmartDashboard::PutData("Start Location", &m_startChooser);
  frc::SmartDashboard::PutBoolean("Calibrate Pose", false);
  frc::SmartDashboard::PutData("Auto", &m_autoChooser);
  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("QuestNav Field", &m_QuestNavField);

  // Call GetInstance() so the constructors get called
  //Intake::GetInstance();
  SupaIntake::GetInstance();
  Cameras::GetInstance();
  SwerveDrive::GetInstance();
  Indexer::GetInstance();
  Climber::GetInstance();
  LEDs::GetInstance();
  LEDs::GetInstance().LEDsInit();
  Shooter::GetInstance();


  m_compressor.EnableDigital(
    // units::pounds_per_square_inch_t{Constants::kMinPressure},
    // units::pounds_per_square_inch_t{Constants::kMaxPressure}
  );

  
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
    auto alliance = frc::DriverStation::GetAlliance();

  
    if (mode != kDisabled) {
      if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        globalAlliance = "Red";

        // Pose selection logic
        if (m_currentPose.X() >= units::meter_t{Constants::kRedHubX}) {
          // in red alliance zone
          m_goalPosition = frc::Translation2d{units::meter_t{Constants::kRedHubX}, units::meter_t{Constants::kRedHubY}};

        } else {
          // in neutral zone
          if (m_currentPose.Y() >= units::meter_t{Constants::kRedHubY}) {
            m_goalPosition = frc::Translation2d{560.165_in, 226.635_in};

          } else {
            m_goalPosition = frc::Translation2d{560.165_in, 91.055_in};
          }
        }

      } else if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue) {
        globalAlliance = "Blue";
        
        // Pose selection logic
        if (m_currentPose.X() <= units::meter_t{Constants::kBlueHubX}) {
          // in blue alliance zone
          m_goalPosition = frc::Translation2d{units::meter_t{Constants::kBlueHubX}, units::meter_t{Constants::kBlueHubY}};

        } else {
          // in neutral zone
          if (m_currentPose.Y() >= units::meter_t{Constants::kRedHubY}) {
            m_goalPosition = frc::Translation2d{91.055_in, 226.635_in};

          } else {
            m_goalPosition = frc::Translation2d{91.055_in, 91.055_in};

          }
        }
      } else {
        globalAlliance = "None";
      }
      
      // std::cout << SwerveDrive::GetInstance().GetPose2d().X().value() * Constants::kInchesPerMeter << ", " << 
      //             SwerveDrive::GetInstance().GetPose2d().Y().value()  * Constants::kInchesPerMeter << ", " <<
      //             SwerveDrive::GetInstance().GetPose2d().Rotation().Degrees().value() << "\n";
    }

    // Get Current State
    m_currentPose = SwerveDrive::GetInstance().GetPose2d();

    //Calculate shot at current state
    ShotCalculator::GetInstance().CalculateShotParams(m_goalPosition, Constants::kPhaseDelay);

    if (mode == kAuto) {
      if (m_auto) {
        m_auto->Update(t);
      }

    } else if (mode == kTeleop) {
      if (m_braking) {
        SwerveDrive::GetInstance().Coast();
        m_braking = false;
      }

      // The auto will reset the pose to be facing towards the driver on the red
      // alliance so it needs to be corrected
      auto alliance = frc::DriverStation::GetAlliance();

      m_field.SetRobotPose(SwerveDrive::GetInstance().GetPose2d());
      
      m_autoAlignMode = kNone;

      // Ramp auto align logic
      if (Controllers::GetInstance().GetDriverController().GetXButtonPressed()) {
        ResetAlignControllers();
        units::meter_t ySetpoint;
        units::degree_t thetaSetpoint;

        // Find closest ramp center
        if (m_currentPose.Y() >= units::meter_t{Constants::kRedHubY}) {
          ySetpoint = units::meter_t{Constants::kRedHubY} + 60_in;

        } else {
          ySetpoint = units::meter_t{Constants::kRedHubY} - 60_in;
        }

        // Find closest heading
        if (std::abs(m_currentPose.Rotation().Degrees().value()) > 90) {
          thetaSetpoint = 180_deg;

        } else {
          thetaSetpoint = 0_deg;
        }

        m_autoAlignSetpoint = {frc::Translation2d{0_m, ySetpoint}, frc::Rotation2d{thetaSetpoint}};

        m_autoAlignMode = kRamp;
        SwerveDrive::GetInstance().DisableRamp();

      } else if (Controllers::GetInstance().GetDriverController().GetXButtonReleased()) {
        m_autoAlignMode = kNone;
        SwerveDrive::GetInstance().EnableRamp();

      }

      std::cout << m_autoAlignMode << std::endl;

      bool LastPosPiston = false;

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

      // Slow/ Medium Mode
      if (Controllers::GetInstance().GetDriverController().GetRightTriggerAxis() > 0.5) {
        vy *= Constants::kSlowMode;
        vx *= Constants::kSlowMode;

      } else if (Controllers::GetInstance().GetDriverController().GetLeftTriggerAxis() > 0.5) {
        vx *= Constants::kMediumMode;
        vy *= Constants::kMediumMode;
      }

      // Brake Mode
      if (Controllers::GetInstance().GetDriverController().GetLeftBumperButton() || Controllers::GetInstance().GetDriverController().GetRightBumperButton()) {
        // SwerveDrive::GetInstance().DriveVelocity(0, 0, 0);
        vx = 0;
        vy = 0;
      }

      // Invert driver controls when on red
      if (alliance.has_value() &&
          alliance.value() == frc::DriverStation::Alliance::kRed) {
        vx *= -1;
        vy *= -1;
      }

      // Reset robot pose
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
        if (m_autoAlignMode == kRamp) {
          auto robotPose = SwerveDrive::GetInstance().GetPose2d();

          // Update y controller only
          vy = m_alignControllers[1].Update(
              robotPose.Translation().Y().value(),
              m_autoAlignSetpoint.Translation().Y().value());
          if (vy < -Constants::kPathFollowingMaxV) {
            vy = -Constants::kPathFollowingMaxV;
          }
          if (vy > Constants::kPathFollowingMaxV) {
            vy = Constants::kPathFollowingMaxV;
          }

          // Update theta controller
          double angleSetpoint =
              m_autoAlignSetpoint.Rotation().Radians().value();
          if (angleSetpoint < -Constants::kPathFollowingMaxW) {
              angleSetpoint = -Constants::kPathFollowingMaxW;
          }
          if (angleSetpoint > Constants::kPathFollowingMaxW) {
             angleSetpoint = Constants::kPathFollowingMaxW;
           }
          double currentAngle = robotPose.Rotation().Radians().value();
          double angleError = angleSetpoint - currentAngle;
          if (angleError > M_PI) {
            angleSetpoint -= 2 * M_PI;
          }

          w = m_alignControllers[2].Update(currentAngle, angleSetpoint);
        }

        //makes swerve work
        SwerveDrive::GetInstance().DriveVelocity(vx, vy, w);
      }

      // Auto Aim
      // operator Shooter/ Turret overrides
      if (Controllers::GetInstance().GetOperatorController().GetAButton()) {
        // Override 1
        ShotCalculator::GetInstance().SetShooterVelocity(45_tps);
        ShotCalculator::GetInstance().SetTurretAngle(0_deg);
      
      } else if (Controllers::GetInstance().GetOperatorController().GetBButton()) {
        // Override 2
        ShotCalculator::GetInstance().SetShooterVelocity(40_tps);
        ShotCalculator::GetInstance().SetTurretAngle(180_deg);
        
      } else if (Controllers::GetInstance().GetOperatorController().GetYButton()) {
        // Override 3
        ShotCalculator::GetInstance().SetShooterVelocity(45_tps);
        ShotCalculator::GetInstance().SetTurretAngle(180_deg);
      }

      if (Controllers::GetInstance().GetOperatorController().GetRightTriggerAxis() > 0.5) {
        if (Shooter::GetInstance().GetShooterState() == Shooter::shooterStates::IDLE) {
          if (ShotCalculator::GetInstance().ShotValid()) {
            Shooter::GetInstance().StartShooting();
            Controllers::GetInstance().GetDriverController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
            Controllers::GetInstance().GetOperatorController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
          } else {
            Controllers::GetInstance().GetDriverController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, .2);
            Controllers::GetInstance().GetOperatorController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, .2);
          }
        }

      } else {
        if (Shooter::GetInstance().GetShooterState() != Shooter::shooterStates::IDLE) {
          // only call if not at idle
          Shooter::GetInstance().StopShooting();
        }
      }


      // operator controls elseif statement hell
      if (Controllers::GetInstance().GetOperatorController().GetPOV() == 0) {
        SupaIntake::GetInstance().SetIntake(0);

      //Dpad down - Stow climb
      } else if (Controllers::GetInstance().GetOperatorController().GetPOV() == 180) {
        SupaIntake::GetInstance().SetIntake(1);

      //Dpad left - Climb
      } else if (Controllers::GetInstance().GetOperatorController().GetPOV() == 270) {
        //Climber::GetInstance().SetClimber(1);

      } else if (Controllers::GetInstance().GetOperatorController().GetLeftTriggerAxis() > 0.5) {
        SupaIntake::GetInstance().SetMotors(0.75);

      } else if (Controllers::GetInstance().GetOperatorController().GetLeftTriggerAxis() < 0.5) {
        SupaIntake::GetInstance().SetMotors(0.0);

      } else { std::cout << "hello\n"; }

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

    // QuestNav nav;
    // frc::Pose2d QuestPose = nav.GetQuestPose();
    // if (frc::SmartDashboard::GetBoolean("Calibrate Pose", false)) {
    //   nav.Calibrate();
    //   frc::SmartDashboard::PutBoolean("Calibrate Pose", false);
    //   std::cout << "Recalibrated" << std::endl;
    // };
    
    // m_QuestNavField.SetRobotPose(QuestPose);
    // std::cout << "X:Y:Pitch : " << QuestPose.X().value() << ", "
    //                             << QuestPose.Y().value() << ", "
    //                             << QuestPose.Rotation().Degrees().value()
    //                             << std::endl;

    
    // Call update functions for subsystems instances
    Cameras::GetInstance().Update(mode, t);
    SwerveDrive::GetInstance().Update(mode, t);
    Shooter::GetInstance().Update(mode, t);
    SupaIntake::GetInstance().Update(mode, t);
    LEDs::GetInstance().Update(mode, globalAlliance);
    Indexer::GetInstance().Update(mode);
  }};

};

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
    
    // //go to places
    if(autoName == "Depot") { 
      m_auto = std::make_shared<AutoDepot>(alliance.value(), m_startChooser.GetSelected()); 
    } else if(autoName == "Outpost") { 
      m_auto = std::make_shared<AutoOutpost>(alliance.value(), m_startChooser.GetSelected()); 
    } else if(autoName == "Center") { 
      m_auto = std::make_shared<AutoCenter>(alliance.value(), m_startChooser.GetSelected()); 

    //shoot
    } else if (autoName == "Shoot") { 
      m_auto = std::make_shared<AutoShoot>(alliance.value(), m_startChooser.GetSelected()); 
    } else if (autoName == "ShootBackUp") { 
      m_auto = std::make_shared<AutoShootBackUp>(alliance.value(), m_startChooser.GetSelected()); 

    //autodonothing!!!
    } else { 
      m_auto = std::make_shared<AutoDoNothing>(); 
    } 
    m_auto->Start(t);
  }
}

void Robot::TeleopInit() {
  // Make sure that ramping is enabled for the driver motors even if auto is incomplete
  SwerveDrive::GetInstance().EnableRamp();
}

#ifndef RUNNING_FRC_TESTS
int main(int argc, char **argv) { frc::StartRobot<Robot>(); }
#endif