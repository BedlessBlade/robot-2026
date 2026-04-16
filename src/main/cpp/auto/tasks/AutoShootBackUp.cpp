#include "auto/tasks/AutoShootBackUp.h"

#include <frc/DriverStation.h>

#include "Locations.h"

#include "auto/FollowPath.h"
#include "auto/Delay.h"
#include "auto/DriveVelocity.h"
#include "auto/StartShooter.h"
#include "auto/StopShooter.h"

#include "systems/SwerveDrive.h"


AutoShootBackUp::AutoShootBackUp(frc::DriverStation::Alliance alliance, int position) {
  // you get to guess what this does
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position),
      Locations::GetInstance().GetStartPosition(alliance, position)
        .TransformBy(frc::Transform2d{ (alliance == frc::DriverStation::Alliance::kRed ? 1 : -1) * 2_m, 0_m, 0_deg })
    }, false, false));
  m_tasks.push_back(std::make_shared<StartShooter>());
  m_tasks.push_back(std::make_shared<Delay>(Constants::kAutoShootPreloadTime));
  m_tasks.push_back(std::make_shared<StopShooter>());
}