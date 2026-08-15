#include "systems/SwerveDrive.h"
#include "systems/Shooter.h"

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/velocity.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>

#include "Constants.h"
#include "Robot.h"
#include "frc/filter/SlewRateLimiter.h"
#include <iostream>

// We need to initialize the gyro and kinematics members. The kinematics
// constructor needs the positions of the four wheels. The coordinate system is
// +x is towards the front of the robot, and +y is to the robot's left.
SwerveDrive::SwerveDrive()
    : m_gyro{Constants::kPigeonCanId},
      m_kinematics{frc::Translation2d{Constants::kWheelBaseLength / 2,
                                      Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{Constants::kWheelBaseLength / 2,
                                      -Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{-Constants::kWheelBaseLength / 2,
                                      Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{-Constants::kWheelBaseLength / 2,
                                      -Constants::kWheelBaseWidth / 2}},
      m_steeringMotors{{Constants::kFlSteeringMotorId},
                       {Constants::kFrSteeringMotorId},
                       {Constants::kBlSteeringMotorId},
                       {Constants::kBrSteeringMotorId}},
      m_driveMotors{{Constants::kFlDriveMotorId},
                    {Constants::kFrDriveMotorId},
                    {Constants::kBlDriveMotorId},
                    {Constants::kBrDriveMotorId}},
      m_encoders{{Constants::kFlEncoderId},
                 {Constants::kFrEncoderId},
                 {Constants::kBlEncoderId},
                 {Constants::kBrEncoderId}},
      m_poseEstimator{
          m_kinematics,
          GetGyroRotation2d(),
          {frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[0].GetPosition().GetValue().value() * 2 *
                   M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio},
               frc::Rotation2d{m_encoders[0].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[1].GetPosition().GetValue().value() * 2 *
                   M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio},
               frc::Rotation2d{m_encoders[1].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[2].GetPosition().GetValue().value() * 2 *
                   M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio},
               frc::Rotation2d{m_encoders[2].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[3].GetPosition().GetValue().value() * 2 *
                   M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio},
               frc::Rotation2d{m_encoders[3].GetPosition().GetValue() * 2 *
                               M_PI}}},
          frc::Pose2d{}
        },
      m_maxVel{Constants::kDriveMaxVelocity},
      m_maxAccel{Constants::kDriveMaxAcceleration},
      m_maxAngVel{Constants::kDriveMaxAngularVelocity},
      m_maxAngAccel{Constants::kDriveMaxAngularAcceleration}
      {
  // Configure the PID values for the position mode on the steering motors
  auto [kS, kV, kP, kI, kD] = Constants::kSteeringMotorGains;
  configs::Slot0Configs config;
  config.kS = kS;
  config.kV = kV;
  config.kP = kP;
  config.kI = kI;
  config.kD = kD;

  // Create a current limit config to apply to the drive motors
  auto currentLimitConfig = configs::CurrentLimitsConfigs{}
                                .WithSupplyCurrentLimitEnable(true)
                                .WithSupplyCurrentLimit(units::ampere_t{
                                    Constants::kDriveCurrentLimit});

  for (int i = 0; i < 4; i++) {
    // At the same time, go ahead and configure the remote sensor to be the
    // CANCoder.

    bool inverted = true;

    if (i == 0 || i == 3) {
      inverted = false;
    };

    m_encoders[i].GetConfigurator().Apply(
        configs::MagnetSensorConfigs{}
            .WithSensorDirection(
                signals::SensorDirectionValue::CounterClockwise_Positive)
            .WithMagnetOffset(Constants::kEncoderOffsets[i]));
    m_steeringMotors[i].GetConfigurator().Apply(
        configs::TalonFXConfiguration{}
            .WithSlot0(config)
            .WithFeedback(
                configs::FeedbackConfigs{}.WithRemoteCANcoder(m_encoders[i]))
            .WithMotorOutput(configs::MotorOutputConfigs{}.WithInverted(inverted)));

    m_driveMotors[i].GetConfigurator().Apply(
        configs::TalonFXConfiguration{}
        .WithCurrentLimits(currentLimitConfig)
        .WithMotorOutput(configs::MotorOutputConfigs{}.WithInverted(inverted))
    );

    // Steering motors should always be in coast mode because we are using
    // closed-loop control
    m_steeringMotors[i].SetNeutralMode(signals::NeutralModeValue::Coast);
  }

  // Default to coast mode
  Coast();
}

// This function needs to be called by the looper to update the drive motors
void SwerveDrive::Update(Robot::Mode mode, double t) {
  // Update the estimation of where the robot thinks it is on the field
  m_poseEstimator.Update(
      GetGyroRotation2d(),
      {frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[0].GetPosition().GetValue().value() *
                          2 * M_PI * Constants::kWheelRadius *
                          Constants::kDriveGearRatio},
           m_encoders[0].GetPosition().GetValue()},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[1].GetPosition().GetValue().value() *
                          2 * M_PI * Constants::kWheelRadius *
                          Constants::kDriveGearRatio},
           m_encoders[1].GetPosition().GetValue()},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[2].GetPosition().GetValue().value() *
                          2 * M_PI * Constants::kWheelRadius *
                          Constants::kDriveGearRatio},
           m_encoders[2].GetPosition().GetValue()},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[3].GetPosition().GetValue().value() *
                          2 * M_PI * Constants::kWheelRadius *
                          Constants::kDriveGearRatio},
           m_encoders[3].GetPosition().GetValue()}});

  if (mode == Robot::kAuto || mode == Robot::kTeleop) {
    // Create desired speeds chassisSpeeds object
    frc::ChassisSpeeds desiredSpeeds = frc::ChassisSpeeds{units::meters_per_second_t{m_vx}, units::meters_per_second_t{m_vy}, units::radians_per_second_t{m_w}};

    // Position mode
    if (m_positionMode = XHold) {
      // Hold robot along specified X position

    } else if (m_positionMode = YHold) {
      // Hold robot along specified Y position

    } else if (m_positionMode = PoseHold) {
      // Hold robot at pose

    }

    // Heading mode
    if (m_headingMode = HeadingHold) {
      // Hold robot at specified heading

    }


    // Conditional velocity and acceleration limits, set last before calculating velocity commands m
    if (Shooter::GetInstance().GetShooterState() == Shooter::FIRE) {
      m_maxVel = Constants::kDriveMaxVelocityShooting;
      m_maxAccel = Constants::kDriveMaxAccelerationShooting;
      m_maxAngVel = Constants::kDriveMaxAngularVelocityShooting;
      m_maxAngAccel = Constants::kDriveMaxAngularAccelerationShooting;
    }

    // Limit velocity
    if (m_maxVel > 0_mps && m_maxAngVel > 0_rad_per_s) {
        units::meters_per_second_t desiredVel = units::meters_per_second_t{std::hypot(desiredSpeeds.vx.value(), desiredSpeeds.vy.value())};

        // Clamp linear velocity
        if (desiredVel > m_maxVel) {
            double scaleFactor = m_maxVel / desiredVel;
            desiredSpeeds = frc::ChassisSpeeds{desiredSpeeds.vx * scaleFactor, desiredSpeeds.vy * scaleFactor, desiredSpeeds.omega};
        }

        // Clamp angular velocity
        desiredSpeeds.omega = units::radians_per_second_t{std::clamp(desiredSpeeds.omega.value(), -m_maxAngVel.value(), m_maxAngVel.value())};
    }

    // Limit acceleration
    if (m_maxAccel > 0_mps_sq && m_maxAngAccel > 0_rad_per_s_sq) {
        units::meters_per_second_t xVelDiff = desiredSpeeds.vx - m_lastSpeeds.vx;
        units::meters_per_second_t yVelDiff = desiredSpeeds.vy - m_lastSpeeds.vy;

        // Find atainable acceleration
        units::meters_per_second_squared_t desiredAccel = units::meters_per_second_t{std::hypot(xVelDiff.value(), yVelDiff.value())} / Constants::kDt;
        units::meters_per_second_squared_t obtainableAccel = units::meters_per_second_squared_t{std::clamp(desiredAccel.value(), 0.0, m_maxAccel.value())};
        double accelAngle = std::atan2(yVelDiff.value(), xVelDiff.value());

        //std::cout << accelAngle << std::endl;

        // Find atainable angular acceleration
        units::radians_per_second_squared_t desiredAngAccel = (desiredSpeeds.omega - m_lastSpeeds.omega) / Constants::kDt;
        units::radians_per_second_squared_t obtainableAngAccel = units::radians_per_second_squared_t{std::clamp(desiredAngAccel.value(), -m_maxAngAccel.value(), m_maxAngAccel.value())};

        // calculate final desired speed
        xVelDiff = std::cos(accelAngle) * obtainableAccel * Constants::kDt;
        yVelDiff = std::sin(accelAngle) * obtainableAccel * Constants::kDt;
        units::radians_per_second_t omegaVelDiff = obtainableAngAccel * Constants::kDt;

        desiredSpeeds = frc::ChassisSpeeds{m_lastSpeeds.vx + xVelDiff, m_lastSpeeds.vy + yVelDiff, m_lastSpeeds.omega + omegaVelDiff};
    }

    // store last speed command
    m_lastSpeeds = desiredSpeeds;

    // reset max vel and accel
    m_maxVel = Constants::kDriveMaxVelocity;
    m_maxAccel = Constants::kDriveMaxAcceleration;
    m_maxAngVel = Constants::kDriveMaxAngularVelocity;
    m_maxAngAccel = Constants::kDriveMaxAngularAcceleration;

    // Use the WPILib kinematics class to determine the individual wheel angles and velocities.
    auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(desiredSpeeds, GetPose2d().Rotation()));

    // Prevent velocities from clipping
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
        &states, units::meters_per_second_t{Constants::kMaxV});
    auto [fl, fr, bl, br] = states;

    // Optimize the angle setpoints to make the wheels reach the correct angle
    // as fast as possible (not go the long way around).
    fl.Optimize(m_encoders[0].GetPosition().GetValue());
    fr.Optimize(m_encoders[1].GetPosition().GetValue());
    bl.Optimize(m_encoders[2].GetPosition().GetValue());
    br.Optimize(m_encoders[3].GetPosition().GetValue());

    // Decrease the speed of modules that aren't pointing in the correct
    // direction.
    fl.speed *= (fl.angle - frc::Rotation2d{units::radian_t{ m_encoders[0].GetPosition().GetValue() }}).Cos();
    fr.speed *= (fr.angle - frc::Rotation2d{units::radian_t{ m_encoders[1].GetPosition().GetValue() }}).Cos();
    bl.speed *= (bl.angle - frc::Rotation2d{units::radian_t{ m_encoders[2].GetPosition().GetValue() }}).Cos();
    br.speed *= (br.angle - frc::Rotation2d{units::radian_t{ m_encoders[3].GetPosition().GetValue() }}).Cos();

    // Set the positions for the wheel angles
    m_steeringMotors[0].SetControl(controls::PositionVoltage{
        units::turn_t{fl.angle.Radians().value() / 2 / M_PI}}.WithSlot(0));
    m_steeringMotors[1].SetControl(controls::PositionVoltage{
        units::turn_t{fr.angle.Radians().value() / 2 / M_PI}}.WithSlot(0));
    m_steeringMotors[2].SetControl(controls::PositionVoltage{
        units::turn_t{bl.angle.Radians().value() / 2 / M_PI}}.WithSlot(0));
    m_steeringMotors[3].SetControl(controls::PositionVoltage{
        units::turn_t{br.angle.Radians().value() / 2 / M_PI}}.WithSlot(0));

    // Use open loop control on the drive motors to get close enough
    // Using closed-loop velocity control with CTRE devices at lower speeds
    // can cause jitter.
    m_driveMotors[0].SetControl(controls::DutyCycleOut{
        fl.speed.value() * Constants::kDriveVelocityMultiplier});
    m_driveMotors[1].SetControl(controls::DutyCycleOut{
        fr.speed.value() * Constants::kDriveVelocityMultiplier});
    m_driveMotors[2].SetControl(controls::DutyCycleOut{
        bl.speed.value() * Constants::kDriveVelocityMultiplier});
    m_driveMotors[3].SetControl(controls::DutyCycleOut{
        br.speed.value() * Constants::kDriveVelocityMultiplier});
  }
}

frc::ChassisSpeeds SwerveDrive::GetStates() {
  auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{SwerveDrive::m_vx}, units::meters_per_second_t{SwerveDrive::m_vy},
        units::radians_per_second_t{SwerveDrive::m_w}, SwerveDrive::GetPose2d().Rotation());

  return speeds;
}

frc::Rotation2d SwerveDrive::GetGyroRotation2d() const {
  return m_gyro.GetRotation2d();
}

units::degrees_per_second_t SwerveDrive::GetGyroAngVelocity2d() {
  return units::degrees_per_second_t{m_gyro.GetAngularVelocityZWorld().GetValue().value()};
}

frc::ChassisSpeeds SwerveDrive::GetVelocity2d() {
  // Get module states
  frc::SwerveModuleState flCur{units::meters_per_second_t{m_driveMotors[0].GetVelocity().GetValue().value() * 2 * M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio}, m_encoders[0].GetPosition().GetValue()};
  frc::SwerveModuleState frCur{units::meters_per_second_t{m_driveMotors[1].GetVelocity().GetValue().value() * 2 * M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio}, m_encoders[1].GetPosition().GetValue()};
  frc::SwerveModuleState blCur{units::meters_per_second_t{m_driveMotors[2].GetVelocity().GetValue().value() * 2 * M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio}, m_encoders[2].GetPosition().GetValue()};
  frc::SwerveModuleState brCur{units::meters_per_second_t{m_driveMotors[3].GetVelocity().GetValue().value() * 2 * M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio}, m_encoders[3].GetPosition().GetValue()};

  // Convert module states to chassis speeds (robot relative)
  return m_kinematics.ToChassisSpeeds(flCur, frCur, blCur, brCur);
}

frc::Pose2d SwerveDrive::GetPose2d() const {
  return m_poseEstimator.GetEstimatedPosition();
}

void SwerveDrive::Coast() {
  for (int i = 0; i < 4; i++) {
    m_driveMotors[i].SetNeutralMode(signals::NeutralModeValue::Coast);
  }
}

void SwerveDrive::Brake() {
  for (int i = 0; i < 4; i++) {
    m_driveMotors[i].SetNeutralMode(signals::NeutralModeValue::Brake);
  }
}

// Sets the drive velocity in meters per second and radians per second
// Call this function with no arguments to stop the robot
void SwerveDrive::DriveVelocity(double vx, double vy, double w) {
  m_vx = vx;
  m_vy = vy;
  m_w = w;
}

void SwerveDrive::ResetPose(frc::Pose2d pose) {
  m_poseEstimator.ResetPose(pose);
}

void SwerveDrive::VisionUpdate(frc::Pose2d pose, units::second_t timestamp) {
  m_poseEstimator.AddVisionMeasurement(pose, timestamp);
}

double SwerveDrive::VelocityMagnitude() {
  return std::sqrt(m_vx * m_vx + m_vy * m_vy);
}

// Set max velocity
void SwerveDrive::SetMaxVelocity(units::meters_per_second_t maxVel) {
    m_maxVel = maxVel;
}

// Set max angular velocity
void SwerveDrive::SetMaxAngularVelocity(units::radians_per_second_t maxAngVel) {
    m_maxAngVel = maxAngVel;
}

// Set max acceleration
void SwerveDrive::SetMaxAcceleration(units::meters_per_second_squared_t maxAccel) {
    m_maxAccel = maxAccel;
}

// Set max angular acceleration
void SwerveDrive::SetMaxAngularAcceleration(units::radians_per_second_squared_t maxAngAccel) {
    m_maxAngAccel = maxAngAccel;
}

// Drive to pose
void SwerveDrive::DriveToPose(frc::Pose2d startPose, frc::Pose2d endPose, frc::Pose2d nextPose, units::meter_t distThreshold, units::radian_t angleThreshold, bool persistVelCommand) {
    // Disable position and heading hold
    m_positionMode = None;
    m_headingMode = None; 

    // Start to end (P21) and robot to end (P2R) vectors
    frc::Translation2d P21 = frc::Translation2d{endPose.X() - startPose.X(), endPose.Y() - startPose.Y()};
    frc::Translation2d P2R = frc::Translation2d{endPose.X() - GetPose2d().X(), endPose.Y() - GetPose2d().Y()};
    frc::Translation2d P32 = frc::Translation2d{nextPose.X() - endPose.X(), nextPose.Y() - endPose.Y()};

    // Normalize start and end angle to [-pi, pi)
    double startAngle = startPose.Rotation().Radians().value();
    startAngle = std::fmod(std::abs(startAngle), 2 * M_PI);
    startAngle = startAngle > M_PI ? startAngle - 2 * M_PI : startAngle;

    double endAngle = endPose.Rotation().Radians().value();
    endAngle = std::fmod(std::abs(endAngle), 2 * M_PI);
    endAngle = endAngle > M_PI ? endAngle - 2 * M_PI : endAngle;

    double currentHeading = GetPose2d().Rotation().Radians().value();
    currentHeading = std::fmod(std::abs(currentHeading), 2 * M_PI);
    currentHeading = currentHeading > M_PI ? currentHeading - 2 * M_PI : currentHeading;

    // Decelerate command
    bool decelCommand = false;

    units::meter_t decelThreshold = (m_maxVel * m_maxVel) / (2 * m_maxAccel);
    if (P21.Norm() < 2 * decelThreshold) {
      // Short path, max vel not reached
      decelThreshold = P21.Norm() / 2;
    }

    double lookAheadAngle = std::acos(P21.Dot(P32).value() / (P21.Norm().value() * P32.Norm().value()));
    if (lookAheadAngle >= M_PI / 2 && P32.Norm() > 0_m) {
      // If P21 and P32 are pointing in a similar direction (i.e., > 90 deg), apply angle based correction to reduce decelThreshold (e.g., if parallel set threshold to 0) 
      decelThreshold *= std::sin(lookAheadAngle);
    }

    if (P2R.Norm() <= decelThreshold) {
      decelCommand = true;
    } 

    // End conditions
    bool atGoalPosition = P2R.Norm() < distThreshold;
    bool atGoalHeading = std::abs(currentHeading - endAngle) < angleThreshold.value();

    if (persistVelCommand && atGoalPosition) {
      // End conditon if persisting Velocity commands between points
      
      // Reset PID controllers
      RestControllers();

      // Return path not finished
      m_atPositionSetpoint = true;

    } else if (!persistVelCommand && atGoalPosition && atGoalHeading) {
      // End condition if not persisting velocity commands between points
      m_vx = 0;
      m_vy = 0;
      m_w = 0;

      // Reset PID controllers
      RestControllers();

      // Return path not finished
      m_atPositionSetpoint = true;

    } else {
      // Drivetrain velocity commands
      double robotAngle = std::copysign(std::acos(P21.Dot(P2R).value() / (P21.Norm().value() * P2R.Norm().value())), P21.Cross(P2R).value());
      units::meter_t errorNorm = P2R.Norm() * std::sin(robotAngle);
      units::meter_t errorTan = P2R.Norm() * std::cos(robotAngle);

      // Normal and tangent velocity commands, clamp tangent to prioritize normal (cross path) error
      units::meters_per_second_t normVelCommand = units::meters_per_second_t{m_xController.Calculate(0.0, errorNorm.value())};
      units::meters_per_second_t tanVelCommand = units::meters_per_second_t{std::clamp(m_yController.Calculate(0.0, errorTan.value()), -m_maxVel.value(), m_maxVel.value())};

      // Set tangent velocity command to maxVel for non persistVelCommand paths
      if (!decelCommand) {
          tanVelCommand = units::meters_per_second_t{std::copysign(m_maxVel.value(), tanVelCommand.value())};
      }

      // Project path velocity to field coordinate system
      double pathAngle = P21.Angle().Radians().value();
      units::meters_per_second_t xVelCommand = tanVelCommand * std::cos(pathAngle) + normVelCommand * std::cos(pathAngle + M_PI/2);
      units::meters_per_second_t yVelCommand = tanVelCommand * std::sin(pathAngle) + normVelCommand * std::sin(pathAngle + M_PI/2);

      // Goal heading as a function of path progress
      double goalHeading = endAngle - (endAngle - startAngle) * errorTan / P21.Norm();
    
      // Heading velocity command
      units::radians_per_second_t wCommand = units::radians_per_second_t{m_thetaController.Calculate(currentHeading, goalHeading)};

      // Set goal
      m_vx = xVelCommand.value();
      m_vy = yVelCommand.value();
      m_w = wCommand.value();

      // Return path not finished
      m_atPositionSetpoint = false;
    }
}

void SwerveDrive::HoldAtPose(frc::Pose2d endPose) {
  m_vx = m_xController.Calculate(GetPose2d().X().value(), endPose.X().value());
  m_vy = m_yController.Calculate(GetPose2d().Y().value(), endPose.Y().value());

  // Normalize goal angle and current heading
  double endAngle = endPose.Rotation().Radians().value();
  endAngle = std::fmod(std::abs(endAngle), 2 * M_PI);
  endAngle = endAngle > M_PI ? endAngle - 2 * M_PI : endAngle;

  double currentHeading = GetPose2d().Rotation().Radians().value();
  currentHeading = std::fmod(std::abs(currentHeading), 2 * M_PI);
  currentHeading = currentHeading > M_PI ? currentHeading - 2 * M_PI : currentHeading;

  m_w = m_thetaController.Calculate(currentHeading, endAngle);
}

bool SwerveDrive::AtPositionSetpoint() {
  return m_atPositionSetpoint;
}

void SwerveDrive::RestControllers() {
  if (!m_controllersRest) {
    m_xController.Reset();
    m_yController.Reset();
    m_thetaController.Reset();

    m_controllersRest = true;
  }
}