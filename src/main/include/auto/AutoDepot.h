#pragma once

#include "TaskList.h"

#include <frc/DriverStation.h>

class AutoDepot : public TaskList {
public:
  
  AutoDepot(frc::DriverStation::Alliance alliance, int position);
};
