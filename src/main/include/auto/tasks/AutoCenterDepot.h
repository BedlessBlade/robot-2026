#pragma once

#include "auto/TaskList.h"

#include <frc/DriverStation.h>

class AutoCenterDepot : public TaskList {
public:
  
  AutoCenterDepot(frc::DriverStation::Alliance alliance, int position, int endBehavior);
};
