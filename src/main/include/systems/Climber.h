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

class Climber : public System {
    public:
    static Climber &GetInstance() {
      static Climber instance;
      return instance;
    }

    rev::spark::SparkMax m_climbMotor{Constants::kClimbMotorID, rev::spark::SparkMax::MotorType::kBrushless};

    rev::spark::SparkClosedLoopController m_climberController = m_climbMotor.GetClosedLoopController();

    void SetClimber(double position);
    double GetClimber();
    double GetClimberPosition();

    bool m_isClimberExtended = false;
    double m_climberExtension = 0.0;
};