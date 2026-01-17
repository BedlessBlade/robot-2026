#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Robot.h"
#include "System.h"

using namespace ctre::phoenix6;

class SwerveDrive : public System {
public:
  static SwerveDrive &GetInstance() {
    static SwerveDrive instance;
    return instance;
  }

  void Update(Robot::Mode mode, double t) override;
  frc::Rotation2d GetGyroRotation2d() const;
  frc::Pose2d GetPose2d() const;
  void Coast();
  void Brake();
  void DriveVelocity(double vx = 0, double vy = 0, double w = 0);
  void ResetPose(frc::Pose2d pose);
  void VisionUpdate(frc::Pose2d pose, units::second_t timestamp);
  void EnableRamp();
  void DisableRamp();
  double VelocityMagnitude();

private:
  // The gyroscope keeps track of which direction the robot is facing.
  hardware::Pigeon2 m_gyro;

  // The steering motors turn the direction of the wheels on the swerve modules.
  hardware::TalonFX m_steeringMotors[4];
  hardware::TalonFX m_driveMotors[4];

  // These encoders keep track of the direction the swerve modules are pointing.
  hardware::CANcoder m_encoders[4];

  // The kinematics class is what we use to convert between global velocities
  // and wheel velocities.
  frc::SwerveDriveKinematics<4> m_kinematics;

  // The pose estimator keeps track of the robot's position on the field.
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // These are the target velocities that are set using DriveVelocity()
  // Vx is forward, Vy is left, and W is the rotational velocity in radians per
  // second
  double m_vx, m_vy, m_w;

  // Slew rate limiters to prevent excessive acceleration
  frc::SlewRateLimiter<units::meters_per_second> m_filterXFast{
      Constants::kDriveMaxAccelerationFast};
  frc::SlewRateLimiter<units::meters_per_second> m_filterYFast{
      Constants::kDriveMaxAccelerationFast};
  frc::SlewRateLimiter<units::radians_per_second> m_filterWFast{
      Constants::kDriveMaxAngularAccelerationFast};
  frc::SlewRateLimiter<units::meters_per_second> m_filterXSlow{
      Constants::kDriveMaxAccelerationSlow};
  frc::SlewRateLimiter<units::meters_per_second> m_filterYSlow{
      Constants::kDriveMaxAccelerationSlow};
  frc::SlewRateLimiter<units::radians_per_second> m_filterWSlow{
      Constants::kDriveMaxAngularAccelerationSlow};

  bool m_rampEnabled = true;
  bool m_fastFilter = true;

  // Make the constructor private so that the GetInstance() function must be
  // used.
  SwerveDrive();
};
