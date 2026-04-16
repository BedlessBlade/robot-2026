#include "auto/tasks/AutoShoot.h"

#include <frc/DriverStation.h>

#include "auto/FollowPath.h"
#include "auto/Delay.h"
#include "auto/StartShooter.h"
#include "auto/StopShooter.h"

#include "systems/SwerveDrive.h"


AutoShoot::AutoShoot(frc::DriverStation::Alliance alliance, int position)  {
    // shoot
    m_tasks.push_back(std::make_shared<StartShooter>());
    m_tasks.push_back(std::make_shared<Delay>(Constants::kAutoShootPreloadTime));
    m_tasks.push_back(std::make_shared<StopShooter>());

}