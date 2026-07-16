// Other includes
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/velocity.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <math.h>

// Subsystem includes
#include "Constants.h"
#include "Robot.h"
#include "systems/SwerveDrive2.h"

// We need to initialize the gyro and kinematics members. The kinematics
// constructor needs the positions of the four wheels. The coordinate system is
// +x is towards the front of the robot, and +y is to the robot's left.
SwerveDrive2::SwerveDrive2() : m_gyro{Constants::kPigeonCanId},
                               m_steeringMotors{{Constants::kFlSteeringMotorId}, {Constants::kFrSteeringMotorId}, {Constants::kBlSteeringMotorId}, {Constants::kBrSteeringMotorId}},
                               m_driveMotors{{Constants::kFlDriveMotorId}, {Constants::kFrDriveMotorId}, {Constants::kBlDriveMotorId}, {Constants::kBrDriveMotorId}},
                               m_encoders{{Constants::kFlEncoderId}, {Constants::kFrEncoderId}, {Constants::kBlEncoderId}, {Constants::kBrEncoderId}},
                               m_kinematics{m_fl, m_fr, m_bl, m_br},
                               m_poseEstimator{m_kinematics, m_gyro.GetRotation2d(), {GetModulePosition(0), GetModulePosition(1), GetModulePosition(2), GetModulePosition(3)}, frc::Pose2d{}} 
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
    auto currentLimitConfig = configs::CurrentLimitsConfigs{}.WithSupplyCurrentLimitEnable(true).WithSupplyCurrentLimit(units::ampere_t{Constants::kDriveCurrentLimit});

    for (int i = 0; i < 4; i++) {
        bool inverted = true;

        if (i == 0 || i == 3) {
            inverted = false;
        };

        // Configure encoders
        m_encoders[i].GetConfigurator().Apply(configs::MagnetSensorConfigs{}.WithSensorDirection(signals::SensorDirectionValue::CounterClockwise_Positive)
                                                                            .WithMagnetOffset(Constants::kEncoderOffsets[i]));

        // Configure steering motors
        m_steeringMotors[i].GetConfigurator().Apply(configs::TalonFXConfiguration{}.WithSlot0(config)
                                                                                   .WithFeedback(configs::FeedbackConfigs{}.WithRemoteCANcoder(m_encoders[i]))
                                                                                   .WithMotorOutput(configs::MotorOutputConfigs{}.WithInverted(inverted)));

        // Set coast on steering motors
        m_steeringMotors[i].SetNeutralMode(signals::NeutralModeValue::Coast);

        // Configure Drive motors
        m_driveMotors[i].GetConfigurator().Apply(configs::TalonFXConfiguration{}.WithCurrentLimits(currentLimitConfig)
                                                                                .WithMotorOutput(configs::MotorOutputConfigs{}.WithInverted(inverted)));
    }

    // Default to coast mode
    SetCoast();
}

// Update function
void SwerveDrive2::Update(Robot::Mode mode, double t) {
    // Update the estimation of where the robot thinks it is on the field
    m_poseEstimator.Update(m_gyro.GetRotation2d(), {GetModulePosition(0), GetModulePosition(1), GetModulePosition(2), GetModulePosition(3)});

    if (mode == Robot::kAuto || mode == Robot::kTeleop) {
        frc::ChassisSpeeds desiredSpeeds = frc::ChassisSpeeds{m_xVel, m_yVel, m_w};

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
            units::meters_per_second_squared_t desiredAccel = units::meters_per_second_t{std::hypot(xVelDiff.value(), yVelDiff.value())} / 0.005_s;
            units::meters_per_second_squared_t obtainableAccel = units::meters_per_second_squared_t{std::clamp(desiredAccel.value(), 0.0, m_maxAccel.value())};
            double accelAngle = std::atan2(xVelDiff.value(), yVelDiff.value());

            // Find atainable angular acceleration
            units::radians_per_second_squared_t desiredAngAccel = (desiredSpeeds.omega - m_lastSpeeds.omega) / 0.005_s;
            units::radians_per_second_squared_t obtainableAngAccel = units::radians_per_second_squared_t{std::clamp(desiredAngAccel.value(), -m_maxAngAccel.value(), m_maxAngAccel.value())};

            // calculate final desired speed
            xVelDiff = std::cos(accelAngle) * obtainableAccel * 0.005_s;
            yVelDiff = std::sin(accelAngle) * obtainableAccel * 0.005_s;
            units::radians_per_second_t omegaVelDiff = obtainableAngAccel * 0.005_s;

            desiredSpeeds = frc::ChassisSpeeds{m_lastSpeeds.vx + xVelDiff, m_lastSpeeds.vx + yVelDiff, m_lastSpeeds.omega + omegaVelDiff};
        }

        // store last speed command
        frc::ChassisSpeeds m_lastSpeeds = desiredSpeeds;

        // Use the WPILib kinematics class to determine the individual wheel angles and velocities.
        auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(desiredSpeeds, GetPose().Rotation()));

        // Prevent velocities from clipping
        frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&states, units::meters_per_second_t{Constants::kMaxV});
        auto [fl, fr, bl, br] = states;

        // Optimize the angle setpoints to make the wheels reach the correct angle
        // as fast as possible (not go the long way around).
        fl.Optimize(m_encoders[0].GetPosition().GetValue());
        fr.Optimize(m_encoders[1].GetPosition().GetValue());
        bl.Optimize(m_encoders[2].GetPosition().GetValue());
        br.Optimize(m_encoders[3].GetPosition().GetValue());

        // Decrease the speed of modules that aren't pointing in the correct
        // direction.
        fl.speed *= (fl.angle - frc::Rotation2d{units::radian_t{m_encoders[0].GetPosition().GetValue()}}).Cos();
        fr.speed *= (fr.angle - frc::Rotation2d{units::radian_t{m_encoders[1].GetPosition().GetValue()}}).Cos();
        bl.speed *= (bl.angle - frc::Rotation2d{units::radian_t{m_encoders[2].GetPosition().GetValue()}}).Cos();
        br.speed *= (br.angle - frc::Rotation2d{units::radian_t{m_encoders[3].GetPosition().GetValue()}}).Cos();

        // Set the positions for the wheel angles
        m_steeringMotors[0].SetControl(controls::PositionVoltage{units::turn_t{fl.angle.Radians().value() / 2 / M_PI}}.WithSlot(0));
        m_steeringMotors[1].SetControl(controls::PositionVoltage{units::turn_t{fr.angle.Radians().value() / 2 / M_PI}}.WithSlot(0));
        m_steeringMotors[2].SetControl(controls::PositionVoltage{units::turn_t{bl.angle.Radians().value() / 2 / M_PI}}.WithSlot(0));
        m_steeringMotors[3].SetControl(controls::PositionVoltage{units::turn_t{br.angle.Radians().value() / 2 / M_PI}}.WithSlot(0));

        // Use open loop control on the drive motors to get close enough
        // Using closed-loop velocity control with CTRE devices at lower speeds
        // can cause jitter.
        m_driveMotors[0].SetControl(controls::DutyCycleOut{fl.speed.value() * Constants::kDriveVelocityMultiplier});
        m_driveMotors[1].SetControl(controls::DutyCycleOut{fr.speed.value() * Constants::kDriveVelocityMultiplier});
        m_driveMotors[2].SetControl(controls::DutyCycleOut{bl.speed.value() * Constants::kDriveVelocityMultiplier});
        m_driveMotors[3].SetControl(controls::DutyCycleOut{br.speed.value() * Constants::kDriveVelocityMultiplier});
    }
}

// Drive to pose
bool SwerveDrive2::DriveToPose(frc::Pose2d startPose, frc::Pose2d endPose, units::radian_t startAngle, units::radian_t endAngle, units::meter_t distThreshold, bool persistVelCommand) {
    // start to end (P21) and robot to end (P2R) vectors
    frc::Translation2d P21 = frc::Translation2d{endPose.X() - startPose.X(), endPose.Y() - startPose.Y()};
    frc::Translation2d P2R = frc::Translation2d{endPose.X() - GetPose().X(), endPose.Y() - GetPose().Y()};

    if (P2R.Norm() > distThreshold) {
        // Normal and tangent path error
        double robotAngle = std::acos(P21.Dot(P2R).value() / (P21.Norm().value() * P2R.Norm().value()));
        int robotAngleSign = P21.Cross(P2R).value() / std::abs(P21.Cross(P2R).value());
        units::meter_t errorNorm = P2R.Norm() * std::sin(robotAngle) * robotAngleSign;
        units::meter_t errorTan = P2R.Norm() * std::cos(robotAngle);

        // Normal and tangent velocity commands
        units::meters_per_second_t normVelCommand = units::meters_per_second_t{m_xController.Calculate(0.0, errorNorm.value())};
        units::meters_per_second_t tanVelCommand = units::meters_per_second_t{m_yController.Calculate(0.0, errorTan.value())};

        // Convert path velocity to field coordinate system
        double pathAngle = P21.Angle().Radians().value();
        units::meters_per_second_t xVelCommand = tanVelCommand * std::cos(pathAngle) + normVelCommand * std::cos(pathAngle + M_PI/2);
        units::meters_per_second_t yVelCommand = tanVelCommand * std::sin(pathAngle) + normVelCommand * std::sin(pathAngle + M_PI/2);

        // Heading error
        units::radian_t errorHeading = (endAngle - (endAngle - startAngle) * errorTan / P21.Norm()) - GetPose().Rotation().Radians();

        // Heading velocity command
        units::radians_per_second_t wCommand = units::radians_per_second_t{m_thetaController.Calculate(0.0, errorHeading.value())};

        // Set goal
        m_xVel = xVelCommand;
        m_yVel = yVelCommand;
        m_w = wCommand;

        // Return path not finished
        return false;
    
    } else {
        // Path end behavior
        if (!persistVelCommand) {
            m_xVel = 0_mps;
            m_yVel = 0_mps;
            m_w = 0_rad_per_s;
        }

        // Reset PID controllers
        m_xController.Reset();
        m_yController.Reset();
        m_thetaController.Reset();

        // Return path finished
        return true;
    }
}

// Rotate arrond point
bool SwerveDrive2::RotateAroundPoint(frc::Pose2d centerPose, frc::Pose2d startPose, frc::Pose2d endPose, units::radian_t startAngle, units::radian_t endAngle, units::meter_t distThreshold, bool persistVelCommand) {
    // Center to start (P1C), robot (PRC), and end pose (P2C) vectors
    frc::Translation2d P1C = frc::Translation2d{startPose.X() - centerPose.X(), startPose.Y() - centerPose.Y()};
    frc::Translation2d P2C = frc::Translation2d{endPose.X() - centerPose.X(), endPose.Y() - centerPose.Y()};
    frc::Translation2d PRC = frc::Translation2d{GetPose().X() - centerPose.X(), GetPose().X() - centerPose.Y()};
    frc::Translation2d P2R = frc::Translation2d{endPose.X() - GetPose().X(), endPose.Y() - GetPose().X()};
    
    if (P2R.Norm() > distThreshold) {
        // Path angles and radius
        double robotAngle = std::acos(PRC.Dot(P1C).value() / (PRC.Norm().value() * P1C.Norm().value()));
        double robotEndAngle = std::acos(P2C.Dot(P1C).value() / (P2C.Norm().value() * P1C.Norm().value()));
        units::meter_t pathRadius = P1C.Norm();

        // Normal and tangent path errors
        units::meter_t robotArclength = pathRadius * robotAngle;
        units::meter_t endArclength = pathRadius * robotEndAngle;
        units::meter_t errorTan = endArclength - robotArclength;
        units::meter_t errorNorm = pathRadius - PRC.Norm();

        // Normal and tangent velocity commands
        units::meters_per_second_t normVelCommand = units::meters_per_second_t{m_xController.Calculate(0.0, errorNorm.value())};
        units::meters_per_second_t tanVelCommand = units::meters_per_second_t{m_yController.Calculate(0.0, errorTan.value())};

        // Convert path velocity to field coordinate system
        double pathAngle = PRC.Angle().Radians().value();
        units::meters_per_second_t xVelCommand = tanVelCommand * std::cos(pathAngle) + normVelCommand * std::cos(pathAngle + M_PI/2);
        units::meters_per_second_t yVelCommand = tanVelCommand * std::sin(pathAngle) + normVelCommand * std::sin(pathAngle + M_PI/2);

        // Heading error
        units::radian_t errorHeading = (endAngle - (endAngle - startAngle) * errorTan / endArclength) - GetPose().Rotation().Radians();

        // Heading velocity command
        units::radians_per_second_t wCommand = units::radians_per_second_t{m_thetaController.Calculate(0.0, errorHeading.value())};

        // Set goal
        m_xVel = xVelCommand;
        m_yVel = yVelCommand;
        m_w = wCommand;

        // Return path not finished
        return false;
    
    } else {
        // Path end behavior
        if (!persistVelCommand) {
            m_xVel = 0_mps;
            m_yVel = 0_mps;
            m_w = 0_rad_per_s;
        }

        // Reset PID controllers
        m_xController.Reset();
        m_yController.Reset();
        m_thetaController.Reset();

        // Return path finished
        return true;
    }
}

// Get module position
frc::SwerveModulePosition SwerveDrive2::GetModulePosition(int moduleID) {
    return frc::SwerveModulePosition{units::meter_t{m_driveMotors[moduleID].GetPosition().GetValue().value() * 2 * M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio}, 
                                     frc::Rotation2d{m_encoders[moduleID].GetPosition().GetValue() * 2 * M_PI}};
}

// Get module state
frc::SwerveModuleState SwerveDrive2::GetModuleState(int moduleID) {
    return frc::SwerveModuleState{units::meters_per_second_t{m_driveMotors[moduleID].GetVelocity().GetValue().value() * 2 * M_PI * Constants::kWheelRadius * Constants::kDriveGearRatio}, 
                                  m_encoders[moduleID].GetPosition().GetValue()};
}

// Get robot velocity
frc::ChassisSpeeds SwerveDrive2::GetVelocity() {
    return m_kinematics.ToChassisSpeeds(GetModuleState(0), GetModuleState(1), GetModuleState(2), GetModuleState(3));
}

// Get robot pose estimate
frc::Pose2d SwerveDrive2::GetPose() {
    return m_poseEstimator.GetEstimatedPosition();
}

// Set coast mode on drive motors
void SwerveDrive2::SetCoast() {
    for (int i = 0; i < 4; i++) {m_driveMotors[i].SetNeutralMode(signals::NeutralModeValue::Coast);}
}

// Set brake mode on drive motors
void SwerveDrive2::SetBrake() {
    for (int i = 0; i < 4; i++) {m_driveMotors[i].SetNeutralMode(signals::NeutralModeValue::Brake);}
}

// Set max velocity
void SwerveDrive2::SetMaxVelocity(units::meters_per_second_t maxVel) {
    m_maxVel = maxVel;
}

// Set max angular velocity
void SwerveDrive2::SetMaxAngularVelocity(units::radians_per_second_t maxAngVel) {
    m_maxAngVel = maxAngVel;
}

// Set max acceleration
void SwerveDrive2::SetMaxAcceleration(units::meters_per_second_squared_t maxAccel) {
    m_maxAccel = maxAccel;
}

// Set max angular acceleration
void SwerveDrive2::SetMaxAngularAcceleration(units::radians_per_second_squared_t maxAngAccel) {
    m_maxAngAccel = maxAngAccel;
}