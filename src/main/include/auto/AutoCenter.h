#pragma once

#include "TaskList.h"

#include <frc/DriverStation.h>

class AutoCenter : public TaskList {
public:
  
  AutoCenter(frc::DriverStation::Alliance alliance, int position);
};
