#pragma once

#include "auto/TaskList.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

class AutoDepot : public TaskList {
public:
    AutoDepot(frc::DriverStation::Alliance alliance, int position, int endBehavior);

};