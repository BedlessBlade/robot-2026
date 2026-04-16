#pragma once

#include "auto/TaskList.h"

#include <frc/DriverStation.h>

class AutoCenterTwo : public TaskList {
public:
  
  AutoCenterTwo(frc::DriverStation::Alliance alliance, int position, bool endInCenter);
};
