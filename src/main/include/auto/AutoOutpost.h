#pragma once

#include "TaskList.h"

#include <frc/DriverStation.h>

class AutoOutpost : public TaskList {
public:
  
  AutoOutpost(frc::DriverStation::Alliance alliance, int position);
};
