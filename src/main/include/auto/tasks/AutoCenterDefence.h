#pragma once

#include "auto/TaskList.h"

#include <frc/DriverStation.h>

class AutoCenterDefence : public TaskList {
public:
  
  AutoCenterDefence(frc::DriverStation::Alliance alliance, int position);
};
