#pragma once

#include "auto/TaskList.h"

#include <frc/DriverStation.h>

class AutoDot : public TaskList {
public:
  
  AutoDot(frc::DriverStation::Alliance alliance, int position, int endBehavior);
};
