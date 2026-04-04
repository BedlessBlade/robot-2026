#include "auto/tasks/AutoShoot.h"

#include <frc/DriverStation.h>

#include "auto/FollowPath.h"
#include "auto/StartShooter.h"

#include "systems/SwerveDrive.h"


AutoShoot::AutoShoot(frc::DriverStation::Alliance alliance, int position)  {
    // shoot
    m_tasks.push_back(std::make_shared<StartShooter>());
}