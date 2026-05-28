#pragma once

#include "auto/TaskList.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

class AutoOutpost : public TaskList {
public:
    AutoOutpost(frc::DriverStation::Alliance alliance, int position, int endBehavior);

};