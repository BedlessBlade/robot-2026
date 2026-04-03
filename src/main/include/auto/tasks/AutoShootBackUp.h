#pragma once

#include "auto/TaskList.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

class AutoShootBackUp : public TaskList {
public:
    AutoShootBackUp(frc::DriverStation::Alliance alliance, int position);

};