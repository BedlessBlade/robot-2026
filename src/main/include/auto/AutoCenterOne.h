#pragma once

#include "TaskList.h"

#include <frc/DriverStation.h>

class AutoCenterOne : public TaskList {
public:
  
  AutoCenterOne(frc::DriverStation::Alliance alliance, int position);
};
