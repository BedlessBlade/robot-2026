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
  static constexpr units::second_t kDt = 0.005_s;

  static constexpr double kInchesPerMeter = 39.37;

  // Driverstation constants
  static constexpr int kDriverControllerId = 0;
  static constexpr int kOperatorControllerId = 1;

  // Constants for the drivetrain
  static constexpr int kPigeonCanId = 13;
  static constexpr double kSlowMode = 0.3;
  static constexpr double kMediumMode = 0.5;
  static constexpr double kShootingMode = 0.3;

  //todo: update robot length
  static constexpr double kRobotWidth = 28.625 / kInchesPerMeter;
  static constexpr double kRobotLength = 40.0 / kInchesPerMeter;
  static constexpr double kBumperLength = 3.75 / kInchesPerMeter;

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

  static constexpr double kWheelRadius = (3.875 / kInchesPerMeter) / 2; // meters

  static constexpr double kDriveCurrentLimit = 60; // Amps
  static constexpr auto kDriveMaxAccelerationFast = units::meters_per_second_t{10} / 1_s;
  static constexpr auto kDriveMaxAccelerationSlow = units::meters_per_second_t{4} / 1_s;
  static constexpr auto kDriveMaxAngularAccelerationFast = units::radians_per_second_t{50} / 1_s;
  static constexpr auto kDriveMaxAngularAccelerationSlow = units::radians_per_second_t{20} / 1_s;
  static constexpr double kDriveVelocityMultiplier = 1 / 4.65;
  static constexpr double kDriveGearRatio = 1.0 / 6.75;
  static constexpr double kDriveControlMultipler = 4.65; // meters per second
  static constexpr double kDriveAngularControlMultiplier = 5.0; // radians per second
  static constexpr double kMaxV = 4.65; // meters per second

  static constexpr double kDefaultMaxV = 1.0; // meters per second
  static constexpr double kDefaultMaxW = 1.0; // radians per second

//  static constexpr pathplanner::PIDConstants translationConstants{5.0,  0.0,  0.0};
 // static constexpr pathplanner::PIDConstants rotationConstants{ 5.0,  0.0,  0.0};


  // Vision
  static constexpr frc::Transform3d kLeftCameraTransform{
      frc::Translation3d{-6.229_in, 12.941_in, 26.747_in},
      frc::Rotation3d{0_rad, 0_rad, -35_deg}};

  static constexpr frc::Transform3d kRightCameraTransform{
      frc::Translation3d{-6.229_in, 10.058_in, 26.747_in},
      frc::Rotation3d{0_rad, 0_rad, 35_deg}};

  static constexpr frc::Transform3d kBackLeftCameraTransform{
      frc::Translation3d{-8.707_in, 13.228_in, 22.794_in},
      frc::Rotation3d{0_rad, 0_rad, -145_deg}};

  static constexpr frc::Transform3d kBackRightCameraTransform{
      frc::Translation3d{-8.707_in, 9.772_in, 22.794_in},
      frc::Rotation3d{0_rad, 0_rad, 145_deg}};


  // Locations
  static constexpr double kFieldLength = 650.12 / kInchesPerMeter;
  static constexpr double kFieldWidth = 316.64 / kInchesPerMeter;
  static constexpr frc::Translation2d kFieldCenter = {units::meter_t{kFieldLength / 2}, units::meter_t{kFieldWidth / 2}};
  static constexpr units::inch_t kHubOffset = 12_in;

  // start offsets
  static constexpr double kStartLineOffset = 156.06 / kInchesPerMeter;
  static constexpr double kStartOffsetX = kRobotWidth / 2;    // meters   
  static constexpr double kStartOffsetY = 60 / kInchesPerMeter;
  static constexpr double kStartOffsetYFar = 132.7 / kInchesPerMeter;

  // field object positions/sizes
  static constexpr double kHubWidth = 47.00 / kInchesPerMeter;
  static constexpr double kBlueHubX = 181.56 / kInchesPerMeter;
  static constexpr double kBlueHubY = 158.32 / kInchesPerMeter;
  static constexpr double kRedHubX = 469.11 / kInchesPerMeter;
  static constexpr double kRedHubY = 158.32 / kInchesPerMeter;


  // Path following
  static constexpr double kPathFollowingKp = 6.0;
  static constexpr double kPathFollowingKi = 0.0;
  static constexpr double kPathFollowingKd = 0;
  static constexpr double kPathFollowingAngleKp = 3.0;
  static constexpr double kPathFollowingAngleKi = 0.0;
  static constexpr double kPathFollowingAngleKd = 0.0;
  static constexpr double kPathFollowingTau = 0.0;
  static constexpr double kPathFollowingMaxV = 3;                   // meters per second
  static constexpr double kPathFollowingMaxW = 5;                   // radians per second
  static constexpr double kPathFollowingTolerance = 0.2;           // meters
  static constexpr double kPathFollowingAngleTolerance = 3;         // degrees
  static constexpr double kPathFollowingVelocityTolerance = 0.1;    // meters per second

  static constexpr auto kFeederDistance = 0.515_m;
  static constexpr double kBrakeDistance = 0.25; // meters  

  //Indexer ------------------------------------------------------
  static constexpr int kIndexerSpinMotorID = 14;
  static constexpr int kIndexerUpMotorID = 15;
  static constexpr double kIndexerSpinMotorSpeed = 0.6;
  static constexpr double kIndexerUpMotorSpeed = -1; 

  //Intake ------------------------------------------------------
  static constexpr int kIntakeMotorId = 25;
  static constexpr double kIntakeForward = 0.75; // 0.7 was too slow during testing, test 0.9
  static constexpr double kIntakeReverse = -0.75;
  static constexpr int kIntakePneumCanId = 22;
  static constexpr int kIntakePneumId1 = 3;
  static constexpr int kIntakePneumId2 = 4;

  static constexpr double kIntakeForce = 0.75;

  static constexpr double kStartIntakeThresh = 0.5;

  //Shooter ------------------------------------------------------
  // Shooter motor CANIDs
  static constexpr int kShooterMotorRightId = 17;
  static constexpr int kShooterMotorLeftId = 18;

  // Shooter motor PID/ FF gains
  static constexpr double kShooterMotorP = 0.2; // was 0.15
  static constexpr double kShooterMotorI = 0.0;
  static constexpr double kShooterMotorD = 0.0;
  static constexpr double kShooterMotorV = 0.12;
  static constexpr double kShooterMotorS = 0.1;

  // Shooter azimuth maxmotion gains
  static constexpr double kShooterAzimuthCV = 8250; //RPS --> RPM conversion for CV
  static constexpr double kShooterAzimuthAcc = 8520 * 3; //Units in RPM/second
  static constexpr double kShooterAzimuthTol = 10; //Units in rotations

  // Shooter limits and misc
  static constexpr double velErrorTol = 1;
  static constexpr double azimuthErrorTol = 3;
  
  // Turret Motor CANID
  static constexpr int kAzimuthMotorId = 16; // This needs to be updated
  
  // Turret motor PID/ FF gains
  static constexpr double kShooterAzimuthP = 0.15;
  static constexpr double kShooterAzimuthI = 0.0;
  static constexpr double kShooterAzimuthD = 1.5;
  static constexpr double kMinAzimuthOutput = -0.75;
  static constexpr double kMaxAzimuthOutput = 0.75;

  static constexpr double kShooterAzimuthS = 0.0;
  static constexpr double kShooterAzimuthV = 0.00109;
  static constexpr double kShooterAzimuthA = 0.0;

  // Turret limits and misc
  static constexpr double kMinShooterAzimuth = 0.0;
  static constexpr double kMaxShooterAzimuth = 320.0;
  static constexpr double kAzimuthMotorRevsToRevs = 233.5 / 1;
  static constexpr frc::Transform2d kTurretOffset{-4_in, -9.5_in, 0_deg}; // turret is 4" back in x, 9.5" over
  static constexpr units::second_t kPhaseDelay = 0.05_s; // reduced from 0.1 s
  
  // Distance to TPS power series constants
  static constexpr double kDist2TPSA = 23.2;
  static constexpr double kDist2TPSB = 0.438;

  // Distance to ToF power series constants
  static constexpr double kDist2ToFA = 0.507; 
  static constexpr double kDist2ToFB = 0.573;

  // Velocity to Distance power series constants
  static constexpr double kVel2DistA = 0.196;
  static constexpr double kVel2DistB = 2.37;

  // Calibration range
  static constexpr double kMinShooterCal = 33.0;
  static constexpr double kMaxShooterCal = 100.0;

  // Hood linear actuator ports
  static constexpr int kLeftHoodServoPort = 9;
  static constexpr int kRightHoodServoPort = 8;

  // auto 
  static constexpr double kAutoShootPreloadTime = 1.5; // seconds
  static constexpr double kAutoShootFullTime = 3.0; 
  static constexpr double kAutoIntakeToggleDelay = 2.0; 
  static constexpr double kAutoIntakeProcessingTime = 0.0;

  //Compressor ------------------------------------------------------
  static constexpr double kMinPressure = 100;
  static constexpr double kMaxPressure = 120;
  static constexpr int kPneumaticCanId = 22;

  //LEDs ------------------------------------------------------
  static constexpr int kLEDSportid = 7;
  static constexpr int kLEDSlength = 300;
  static constexpr units::second_t kLEDSbreathetime{1.5_s};
};