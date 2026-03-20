#pragma once

#include "TaskList.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

class AutoShoot : public TaskList {
public:
    AutoShoot(frc::DriverStation::Alliance alliance, int position);

};