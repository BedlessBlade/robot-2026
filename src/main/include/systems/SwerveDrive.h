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
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/PIDController.h>

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
  units::angular_velocity::degrees_per_second_t GetGyroAngVelocity2d();
  frc::ChassisSpeeds GetVelocity2d(); 
  frc::Pose2d GetPose2d() const;
  void Coast();
  void Brake();
  void DriveVelocity(double vx = 0, double vy = 0, double w = 0);
  void ResetPose(frc::Pose2d pose);
  void VisionUpdate(frc::Pose2d pose, units::second_t timestamp);
  double VelocityMagnitude();
  frc::ChassisSpeeds GetStates();

  // Velocity and acceleration limit setters
  void SetMaxVelocity(units::meters_per_second_t maxVel);
  void SetMaxAngularVelocity(units::radians_per_second_t maxAngVel);
  void SetMaxAcceleration(units::meters_per_second_squared_t maxAccel);
  void SetMaxAngularAcceleration(units::radians_per_second_squared_t maxAngAccel);

  // Drive to pose functions
  void DriveToPose(frc::Pose2d startPose, frc::Pose2d endPose, frc::Pose2d nextPose, units::meter_t distThreshold, units::radian_t angleThreshold, bool persistVelCommand);
  void HoldAtPose(frc::Pose2d endPose);
  void RestControllers();

  // Get atPositionSetpoint
  bool AtPositionSetpoint();

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

  // Drivetrain velocity limits (m/s and deg/s)
  units::meters_per_second_t m_maxVel;
  units::radians_per_second_t m_maxAngVel;

  // Drivetrain acceleration limits (m/s^2 and deg/s^2)
  units::meters_per_second_squared_t m_maxAccel;
  units::radians_per_second_squared_t m_maxAngAccel;

  // Last goal velocity
  frc::ChassisSpeeds m_lastSpeeds;

  // translation and heading PID controllers
  frc::PIDController m_xController{Constants::kPathFollowingKp, Constants::kPathFollowingKi, Constants::kPathFollowingKd, Constants::kDt};
  frc::PIDController m_yController{Constants::kPathFollowingKp, Constants::kPathFollowingKi, Constants::kPathFollowingKd, Constants::kDt};
  frc::PIDController m_thetaController{Constants::kPathFollowingAngleKp, Constants::kPathFollowingAngleKi, Constants::kPathFollowingAngleKd, Constants::kDt};

  // at position setpoint
  bool m_atPositionSetpoint;
  bool m_controllersRest;

  // Make the constructor private so that the GetInstance() function must be used
  SwerveDrive();
};