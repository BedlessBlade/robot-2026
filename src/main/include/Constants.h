#pragma once

#include <frc/geometry/Transform3d.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

// Fix stupid windows compiler issue
#include <numbers>
#ifndef M_PI
# define M_PI std::numbers::pi
#endif

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

//  static constexpr pathplanner::PIDConstants translationConstants{5.0,  0.0,  0.0};
 // static constexpr pathplanner::PIDConstants rotationConstants{ 5.0,  0.0,  0.0};


  // Vision
  static constexpr frc::Transform3d kLeftCameraTransform{
      frc::Translation3d{-6.229_in, 12.359_in, 26.747_in},
      frc::Rotation3d{0_rad, 0_rad, -35_deg}};
  static constexpr frc::Transform3d kRightCameraTransform{
      frc::Translation3d{4.75_in, -11.25_in, 26.747_in},
      frc::Rotation3d{0_rad, 0_rad, 35_deg}};
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
  static constexpr double kPathFollowingVelocityTolerance = 0.1; // meters per second

  static constexpr auto kFeederDistance = 0.515_m;
  static constexpr double kBrakeDistance = 0.25; // meters  

  //Indexer ------------------------------------------------------
  static constexpr int kIndexerSpinMotorID = 14;
  static constexpr int kIndexerUpMotorID = 15;
  static constexpr double kIndexerSpinMotorSpeed = 0.3;
  static constexpr double kIndexerUpMotorSpeed = -0.5;

  //Climber ------------------------------------------------------
  static constexpr int kClimbMotorID = 24;
  static constexpr double kClimbStowed = 0.0;
  static constexpr double kClimbClimbed = 0.5;
  static constexpr double kClimbExtended = 1.0;

  static constexpr double kClimberP = 0.0;
  static constexpr double kClimberI = 0.0;
  static constexpr double kClimberD = 0.0;
  static constexpr double kMinClimberOutput = -0.1;
  static constexpr double kMaxClimberOutput = 0.1;
  
  static constexpr double kClimberS = 0.0;
  static constexpr double kClimberV = 0.0;
  static constexpr double kClimberA = 0.0;
  static constexpr double kClimberG = 0.0;
  static constexpr double kClimberCos = 0.0;
  static constexpr double kClimberCosRatio = 0.0;

  static constexpr double kClimberCruiseVel = 0.0;
  static constexpr double kClimberMaxAccel = 0.0;
  static constexpr double kClimberAllowedErr = 0.0;
  

  //Intake ------------------------------------------------------
  static constexpr int kIntakeMotorId = 25;
  static constexpr double kIntakeForward = 0.75; // 0.7 was too slow during testing, test 0.9 and adjust f
  static constexpr double kIntakeReverse = -0.75;
  static constexpr int kIntakePneumCanId = 25;
  static constexpr int kIntakePneumId1 = 2;
  static constexpr int kIntakePneumId2 = 3;

  static constexpr double kStartIntakeThresh = 0.5;

  //Shooter ------------------------------------------------------
  // Shooter motor CANIDs
  static constexpr int kShooterMotorRightId = 17;
  static constexpr int kShooterMotorLeftId = 18;

  // Shooter motor PID/ FF gains
  static constexpr double kShooterMotorP = 10.0;
  static constexpr double kShooterMotorI = 0.0;
  static constexpr double kShooterMotorD = 0.0;
  static constexpr double kShooterMotorV = 0.12;
  static constexpr double kShooterMotorS = 0.1;

  // Shooter limits and misc
  static constexpr double velErrorTol = 10;
  static constexpr double azimuthErrorTol = 10;
  
  // Turret Motor CANID
  static constexpr int kAzimuthMotorId = 0;
  
  // Turret motor PID/ FF gains
  static constexpr double kShooterAzimuthP = 0.0;
  static constexpr double kShooterAzimuthI = 0.0;
  static constexpr double kShooterAzimuthD = 0.0;
  static constexpr double kMinAzimuthOutput = -0.1;
  static constexpr double kMaxAzimuthOutput = 0.1;

  static constexpr double kShooterAzimuthS = 0.0;
  static constexpr double kShooterAzimuthV = 0.0;
  static constexpr double kShooterAzimuthA = 0.0;
  static constexpr double kShooterAzimuthG = 0.0;
  static constexpr double kShooterAzimuthCos = 0.0;
  static constexpr double kShooterAzimuthCosRatio = 0.0;

  // Turret limits and misc
  static constexpr double kMinShooterAzimuth = 0.0;
  static constexpr double kMaxShooterAzimuth = 270.0;
  static constexpr double kAzimuthMotorRevsToRevs = 212.5 / 1;
  
  // Hood linear actuator ports
  static constexpr int kLeftHoodServoPort = 9;
  static constexpr int kRightHoodServoPort = 8;

  //Compressor

  static constexpr double kMinPressure = 100;
  static constexpr double kMaxPressure = 120;

  // Hood limits
};