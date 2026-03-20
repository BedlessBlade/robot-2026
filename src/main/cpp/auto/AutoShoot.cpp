#include "auto/AutoShoot.h"
#include "auto/TaskList.h"
#include <frc/DriverStation.h>
#include "auto/FollowPath.h"
#include "systems/SwerveDrive.h"
#include "auto/StartShooter.h"


AutoShoot::AutoShoot(frc::DriverStation::Alliance alliance, int position)  {
    m_tasks.push_back(std::make_shared<StartShooter>());
}