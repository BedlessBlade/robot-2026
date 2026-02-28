#pragma once

#include "Robot.h"
#include "System.h"
#include "Constants.h"

#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>

#include <rev/SparkBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>

#include <frc/Servo.h>

using namespace::rev::spark;

class Climber : public System {
    public:
    Climber();

    static Climber &GetInstance() {
      static Climber instance;
      return instance;
    }

    frc::Servo m_linearActuator{8};

    rev::spark::SparkMax m_climbMotor{Constants::kClimbMotorID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController m_climberController = m_climbMotor.GetClosedLoopController();
    rev::spark::SparkMaxConfig m_climberConfig;
    rev::spark::SparkRelativeEncoder m_climberEncoder = m_climbMotor.GetEncoder();

    void Update(Robot::Mode mode);
    void SetClimber(double position, SparkBase::ControlType controlType);
    double GetClimberPosition();

    double m_climberExtension = 0.0;
};