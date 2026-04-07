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
#include "systems/Shooter.h"
#include "systems/ShotCalculator.h"

#include "auto/tasks/AutoDoNothing.h"
#include "auto/tasks/AutoShoot.h"
#include "auto/tasks/AutoShootBackUp.h"
#include "auto/tasks/AutoDepot.h"
#include "auto/tasks/AutoOutpost.h"
#include "auto/tasks/AutoCenterDepot.h"
#include "auto/tasks/AutoCenterOne.h"
#include "auto/tasks/AutoCenterTwo.h"


// This gets called first. So, initialize everything here.
Robot::Robot()
  : m_compressor{frc::PneumaticsModuleType::REVPH},
    m_alignControllers{
      {Constants::kPathFollowingKp, Constants::kPathFollowingKi,
       Constants::kPathFollowingKd, Constants::kPathFollowingTau},
      {Constants::kPathFollowingKp, Constants::kPathFollowingKi,
       Constants::kPathFollowingKd, Constants::kPathFollowingTau},
      {Constants::kPathFollowingAngleKp, Constants::kPathFollowingAngleKi,
       Constants::kPathFollowingAngleKd, Constants::kPathFollowingTau}} {
  m_startChooser.SetDefaultOption("1", 1);
  m_startChooser.AddOption("2", 2);
  m_startChooser.AddOption("3", 3);
  m_startChooser.AddOption("4", 4);
  m_startChooser.AddOption("5", 5);

  m_autoChooser.SetDefaultOption("Do Nothing", "DoNothing");
  m_autoChooser.AddOption("Shoot", "Shoot");
  m_autoChooser.AddOption("Shoot & Back Up", "ShootBackUp");
  m_autoChooser.AddOption("To Depot", "Depot");
  m_autoChooser.AddOption("To Outpost", "Outpost");
  m_autoChooser.AddOption("To Neutral Zone (1 Sweep)", "CenterOne");
  m_autoChooser.AddOption("To Neutral Zone (2 Sweeps)", "CenterTwo");
  m_autoChooser.AddOption("To Neutral & Depot", "CenterDepot");

  m_positionString = "(0, 0, 0)";
  frc::SmartDashboard::PutData("Start Location", &m_startChooser);
  frc::SmartDashboard::PutData("Auto", &m_autoChooser);
  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutString("Pose (Inches)", "(0,0,0)");

  // Call GetInstance() so the constructors get called
  //Intake::GetInstance();
  SupaIntake::GetInstance();
  Cameras::GetInstance();
  SwerveDrive::GetInstance();
  Indexer::GetInstance();
  LEDs::GetInstance();
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
    
    // update variables used throughout the loop
    double t = frc::Timer::GetFPGATimestamp().value();
    auto alliance = frc::DriverStation::GetAlliance();

    m_field.SetRobotPose(SwerveDrive::GetInstance().GetPose2d());
    m_currentPose = SwerveDrive::GetInstance().GetPose2d();
    frc::SmartDashboard::PutString("Pose (Inches)", 
      "(" + std::to_string(m_currentPose.X().value() * Constants::kInchesPerMeter) + ", " + 
            std::to_string(m_currentPose.Y().value() * Constants::kInchesPerMeter) + ", " +
            std::to_string(m_currentPose.Rotation().Degrees().value()) + ")"); 

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
      
    }

    //Calculate shot at current state
    ShotCalculator::GetInstance().CalculateShotParams(m_goalPosition, Constants::kPhaseDelay);
    Controllers::GetInstance().GetDriverController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
    Controllers::GetInstance().GetOperatorController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);

    if (mode == kAuto) {
      if (m_auto) {
        m_auto->Update(t);
      }      
      LEDs::GetInstance().SetPattern(alliance ? LEDs::LEDstates::BREATHBLUE : LEDs::LEDstates::BREATHERED);

    } else if (mode == kTeleop) {
      if (m_braking) {
        SwerveDrive::GetInstance().Coast();
        m_braking = false;

        LEDs::GetInstance().SetPattern(alliance ? LEDs::LEDstates::BLUE : LEDs::LEDstates::RED);
      }

      // The auto will reset the pose to be facing towards the driver on the red
      // alliance so it needs to be corrected

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
        SwerveDrive::GetInstance().SetMode(SwerveDrive::SpeedMode::Slow);
      } else if (Controllers::GetInstance().GetDriverController().GetLeftTriggerAxis() > 0.5) {
        SwerveDrive::GetInstance().SetMode(SwerveDrive::SpeedMode::Medium);
      } else {
        SwerveDrive::GetInstance().SetMode(SwerveDrive::SpeedMode::Fast);
      }

      // Brake Mode
      if (Controllers::GetInstance().GetDriverController().GetLeftBumperButton() || 
          Controllers::GetInstance().GetDriverController().GetRightBumperButton()) {
        // SwerveDrive::GetInstance().DriveVelocity(0, 0, 0);
        vx = 0;
        vy = 0;
      }

      // Invert driver controls when on red
      if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
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

          vy = std::clamp(m_alignControllers[1].Update(robotPose.Translation().Y().value(), m_autoAlignSetpoint.Translation().Y().value()), 
            -Constants::kPathFollowingMaxV, Constants::kPathFollowingMaxV);
            
          double angleSetpoint = m_autoAlignSetpoint.Rotation().Radians().value();
          
          double currentAngle = robotPose.Rotation().Radians().value();
          double angleError = angleSetpoint - currentAngle;
          if (angleError > M_PI) {
            angleSetpoint -= 2 * M_PI;
          }

          w = std::clamp(m_alignControllers[2].Update(currentAngle, angleSetpoint), 
            -Constants::kPathFollowingMaxW, Constants::kPathFollowingMaxW);
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

      // intake up/down if/else
      //Dpad up - Intake up
      if (Controllers::GetInstance().GetOperatorController().GetPOV() == 0) {
        SupaIntake::GetInstance().SetIntake(0);
        LEDs::GetInstance().SetPattern(alliance ? LEDs::LEDstates::BLUE : LEDs::LEDstates::RED);

      //Dpad down - Intake down
      } else if (Controllers::GetInstance().GetOperatorController().GetPOV() == 180) {
        SupaIntake::GetInstance().SetIntake(1);
        LEDs::GetInstance().SetPattern(alliance ? LEDs::LEDstates::BLUEYELLOW : LEDs::LEDstates::REDYELLOW);

      } 
      
      //intake motor if/else
      if (Controllers::GetInstance().GetOperatorController().GetLeftTriggerAxis() > 0.5) {
        SupaIntake::GetInstance().SetMotors(0.75);
        LEDs::GetInstance().SetPattern(alliance ? LEDs::LEDstates::BLUEYELLOWSCROLL : LEDs::LEDstates::REDYELLOWSCROLL);

      } else if (Controllers::GetInstance().GetOperatorController().GetLeftBumperButton()) {
        SupaIntake::GetInstance().SetMotors(-0.75);
        LEDs::GetInstance().SetPattern(alliance ? LEDs::LEDstates::BLUEYELLOWREVERSE : LEDs::LEDstates::REDYELLOWREVERSE);
      
      } else {
        SupaIntake::GetInstance().SetMotors(0.0);
        LEDs::GetInstance().SetPattern(alliance ? LEDs::LEDstates::BLUE : LEDs::LEDstates::RED);
      }

    } else if (mode == kDisabled) {
      LEDs::GetInstance().SetPattern(LEDs::LEDstates::OFF);
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
// Cleanup any resources (especially files) before the robot code gets restarted.
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

    frc::Pose2d start = Locations::GetInstance().GetStartPosition(alliance.value(), m_startChooser.GetSelected());
    SwerveDrive::GetInstance().ResetPose(start);

    std::string autoName = m_autoChooser.GetSelected();
    double t = frc::Timer::GetFPGATimestamp().value();
    
    // simple autos
    if (autoName == "Shoot") { 
      m_auto = std::make_shared<AutoShoot>(alliance.value(), m_startChooser.GetSelected()); 
    } else if (autoName == "ShootBackUp") { 
      m_auto = std::make_shared<AutoShootBackUp>(alliance.value(), m_startChooser.GetSelected()); 

    // location autos
    } else if (autoName == "Depot") { 
      m_auto = std::make_shared<AutoDepot>(alliance.value(), m_startChooser.GetSelected()); 
    } else if (autoName == "Outpost") { 
      m_auto = std::make_shared<AutoOutpost>(alliance.value(), m_startChooser.GetSelected()); 
    
    // center autos
    } else if (autoName == "CenterOne") { 
      m_auto = std::make_shared<AutoCenterOne>(alliance.value(), m_startChooser.GetSelected()); 
    } else if (autoName == "CenterTwo") { 
      m_auto = std::make_shared<AutoCenterTwo>(alliance.value(), m_startChooser.GetSelected());
    } else if (autoName == "CenterDepot") {
      m_auto = std::make_shared<AutoCenterDepot>(alliance.value(), m_startChooser.GetSelected());

    // autodonothing!!!
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