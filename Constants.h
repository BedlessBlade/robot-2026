#pragma once

#include <frc/geometry/Transform3d.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

class Constants {
public:
  static constexpr double kInchesPerMeter = 39.37;

  // Driverstation constants
  static constexpr int kDriverControllerId = 0;
  static constexpr int kOperatorControllerId = 1;

  // Constants for the drivetrain
  static constexpr int kPigeonCanId = 13;

  static constexpr auto kWheelBaseWidth = 26.75_in; //left to right
  static constexpr auto kWheelBaseLength = 15.75_in; //front to back

  static constexpr int kFlSteeringMotorId = 6;
  static constexpr int kFrSteeringMotorId = 5;
  static constexpr int kBlSteeringMotorId = 7;
  static constexpr int kBrSteeringMotorId = 8;
  static constexpr int kFlDriveMotorId = 2;
  static constexpr int kFrDriveMotorId = 1;
  static constexpr int kBlDriveMotorId = 3;
  static constexpr int kBrDriveMotorId = 4;

  static constexpr int kFlEncoderId = 10;
  static constexpr int kFrEncoderId = 9;
  static constexpr int kBlEncoderId = 11;
  static constexpr int kBrEncoderId = 12;

  static constexpr units::turn_t kEncoderOffsets[] = {
      -0.149658203125_tr, -0.23193359375_tr, -0.056640625_tr,
      -0.26904296875_tr};

  // kS, kV, kP, kI, kD
  static constexpr std::tuple<double, double, double, double, double>
      kSteeringMotorGains{0.0, 0.0, 30.0, 0.0, 0.0};

  static constexpr double kWheelRadius =
      (3.875 / kInchesPerMeter) / 2; // meters

  static constexpr double kDriveCurrentLimit = 60; // Amps
  static constexpr auto kDriveMaxAccelerationFast =
      units::meters_per_second_t{13.3} / 1_s;
  static constexpr auto kDriveMaxAccelerationSlow =
      units::meters_per_second_t{5} / 1_s;
  static constexpr auto kDriveMaxAngularAccelerationFast =
      units::radians_per_second_t{50} / 1_s;
  static constexpr auto kDriveMaxAngularAccelerationSlow =
      units::radians_per_second_t{50} / 1_s;
  static constexpr double kDriveVelocityMultiplier = 1 / 4.65;
  static constexpr double kDriveGearRatio = 1.0 / 6.75;
  static constexpr double kDriveControlMultipler = 4.65; // meters per second
  static constexpr double kDriveAngularControlMultiplier =
      5.0;                              // radians per second
  static constexpr double kMaxV = 4.65; // meters per second

  static constexpr double kDefaultMaxV = 1.0; // meters per second
  static constexpr double kDefaultMaxW = 1.0; // radians per second

  // Vision
  static constexpr frc::Transform3d kFrontCameraTransform{
      frc::Translation3d{6.298_in, -10.622_in, 23_in},
      frc::Rotation3d{0_rad, 20_deg, 20_deg}};
  static constexpr frc::Transform3d kBackCameraTransform{
      frc::Translation3d{4.75_in, -11.25_in, 32.75_in},
      frc::Rotation3d{0_rad, 0_rad, 0_deg}};
  static constexpr auto kBlockedTags = {1, 2, 3, 4, 5, 12, 13, 14, 15, 16};


  // Locations
  static constexpr double kFieldLength = 650.12 / kInchesPerMeter;
  static constexpr double kFieldWidth = 316.64 / kInchesPerMeter;

  //TODO: update probably
  static constexpr double kStartLineOffset = (25 * 12) / kInchesPerMeter;
  static constexpr double kStartOffsetY =
      kFieldWidth / 2 - (24 / kInchesPerMeter); // meters
  static constexpr double kStartOffsetX = 0;    // meters   

  //field object positions/sizes
  static constexpr double kHubWidth = 47.00 / kInchesPerMeter;
  static constexpr double kBlueHubX = 158.32 / kInchesPerMeter;
  static constexpr double kBlueHubY = 181.56 / kInchesPerMeter;
  static constexpr double kRedHubX = 491.80 / kInchesPerMeter;
  static constexpr double kRedHubY = 135.08 / kInchesPerMeter;

  //assuming origin as bottom left corner of the arena, as per diagram on frc resources
  static constexpr double kHubX = 158.84 / kInchesPerMeter;
  static constexpr double kHubY = 182.11 / kInchesPerMeter;


  static constexpr double kReefWidth = 65.2 / kInchesPerMeter;
  static constexpr double kReefOffset =
      (144 / kInchesPerMeter) + kReefWidth / 2;
  
  static constexpr double kReefSpacing = 0.515; // meters
  static constexpr double kCoralSpacing = 13 / kInchesPerMeter;
  
  


  // Path following
  static constexpr double kPathFollowingKp = 6.0;
  static constexpr double kPathFollowingKi = 0.005;
  static constexpr double kPathFollowingKd = 0;
  static constexpr double kPathFollowingAngleKp = 3.0;
  static constexpr double kPathFollowingAngleKi = 0.004;
  static constexpr double kPathFollowingAngleKd = 0.0;
  static constexpr double kPathFollowingMaxV = 1.5;       // meters per second
  static constexpr double kPathFollowingMaxW = 0.5;       // radians per second
  static constexpr double kPathFollowingTolerance = 0.04; // meters
  static constexpr double kPathFollowingVelocityTolerance =
      0.1; // meters per second

  static constexpr auto kFeederDistance = 0.515_m;
  static constexpr double kBrakeDistance = 0.25; // meters

  static constexpr int kSpindexerMotorId = 0;
  static constexpr int kKickerMotorId= 0;
  static constexpr double kSpinSpeed = 0.25;
  static constexpr double kKickSpeed = 0.25;

  static constexpr int kAzimuthMotorId = 0;
  static constexpr double kShooterAzimuthP = 0.0;
  static constexpr double kShooterAzimuthI = 0.0;
  static constexpr double kShooterAzimuthD = 0.0;
  static constexpr double kMinShooterAzimuth = 0.0;
  static constexpr double kMaxShooterAzimuth = 0.0;
  static constexpr double kShooterAzimuthS = 0.0;
  static constexpr double kShooterAzimuthV = 0.0;
  static constexpr double kShooterAzimuthA = 0.0;
  static constexpr double kShooterAzimuthG = 0.0;
  static constexpr double kShooterAzimuthCos = 0.0;
  static constexpr double kShooterAzimuthCosRatio = 0;
  static constexpr double kAzimuthMotorRevsToRevs = 212.5 / 1; //Rotations to motor to rotations of turret

  //Shooter Motor ids
  static constexpr int kShooterMotorId = -1;

  static constexpr int kIntakeMotorId = 0;

};