#pragma once

// Other incldues
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

// Subsystem includes
#include "Robot.h"
#include "System.h"
#include "Constants.h"

using namespace ctre::phoenix6;

class SwerveDrive2 : public System {
public:
    // Singleton swerve drive instance getter
    static SwerveDrive2 &GetInstance() {
        static SwerveDrive2 instance;
        return instance;
    }

    // Field or robot relative speed setters
    void SetVelocity(units::meters_per_second_t vx = 0_mps, units::meters_per_second_t vy = 0_mps, units::radians_per_second_t w = 0_deg_per_s);

    // Velocity and acceleration limit setters
    void SetMaxVelocity(units::meters_per_second_t maxVel);
    void SetMaxAngularVelocity(units::radians_per_second_t maxAngVel);
    void SetMaxAcceleration(units::meters_per_second_squared_t maxAccel);
    void SetMaxAngularAcceleration(units::radians_per_second_squared_t maxAngAccel);

    // Drive to pose function
    bool DriveToPose(frc::Pose2d startPose, frc::Pose2d endPose, units::radian_t startAngle, units::radian_t endAngle, units::meter_t distThreshold, bool persistVelCommand);

    // Drive arround point function
    bool RotateAroundPoint(frc::Pose2d centerPose, frc::Pose2d startPose, frc::Pose2d endPose, units::radian_t startAngle, units::radian_t endAngle, units::meter_t distThreshold, bool persistVelCommand);

    // 
    frc::SwerveModulePosition GetModulePosition(int moduleID);
    frc::SwerveModuleState GetModuleState(int moduleID);
    frc::ChassisSpeeds GetVelocity();
    frc::Pose2d GetPose();

    // 
    void Update(Robot::Mode mode, double t) override;
    void VisionUpdate(frc::Pose2d pose, units::second_t timestamp);
    void ResetPose(frc::Pose2d pose);

    // 
    void SetCoast();
    void SetBrake();

private:
    // Drivetrain hardware
    hardware::Pigeon2 m_gyro;
    hardware::TalonFX m_steeringMotors[4];
    hardware::TalonFX m_driveMotors[4];
    hardware::CANcoder m_encoders[4];

    // Kinematics class used to convert between golbal, robot, and swerve module speeds
    frc::SwerveDriveKinematics<4> m_kinematics;

    // Kalman filter based state observer, estimates robot field relative pose
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

    // translation and heading PID controllers
    frc::PIDController m_xController{Constants::kPathFollowingKp, Constants::kPathFollowingKi, Constants::kPathFollowingKd, Constants::kDt};
    frc::PIDController m_yController{Constants::kPathFollowingKp, Constants::kPathFollowingKi, Constants::kPathFollowingKd, Constants::kDt};
    frc::PIDController m_thetaController{Constants::kPathFollowingAngleKp, Constants::kPathFollowingAngleKi, Constants::kPathFollowingAngleKd, Constants::kDt};

    // Drivetrain velocity limits (m/s and deg/s)
    units::meters_per_second_t m_maxVel;
    units::radians_per_second_t m_maxAngVel;

    // Drivetrain acceleration limits (m/s^2 and deg/s^2)
    units::meters_per_second_squared_t m_maxAccel;
    units::radians_per_second_squared_t m_maxAngAccel;

    // Drivetrain goal velocities (m/s and deg/s)
    units::meters_per_second_t m_xVel, m_yVel;
    units::radians_per_second_t m_w;

    // Last goal velocity
    frc::ChassisSpeeds m_lastSpeeds;

    // Module positions
    frc::Translation2d m_fl = frc::Translation2d{Constants::kWheelBaseLength / 2, Constants::kWheelBaseWidth / 2};
    frc::Translation2d m_fr = frc::Translation2d{Constants::kWheelBaseLength / 2, -Constants::kWheelBaseWidth / 2};
    frc::Translation2d m_bl = frc::Translation2d{-Constants::kWheelBaseLength / 2, Constants::kWheelBaseWidth / 2};
    frc::Translation2d m_br = frc::Translation2d{-Constants::kWheelBaseLength / 2, -Constants::kWheelBaseWidth / 2};

    // Private constructor
    SwerveDrive2();
};