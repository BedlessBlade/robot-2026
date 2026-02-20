#pragma once

#include "Robot.h"
#include "System.h"
#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>
#include <rev/SparkMax.h>

class Climber : public frc::TimedRobot {
    public:
    static Climber &GetInstance() {
      static Climber instance;
      return instance;
    }

    
};