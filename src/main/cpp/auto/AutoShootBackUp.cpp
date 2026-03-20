#include "auto/AutoShootBackUp.h"
#include "auto/TaskList.h"
#include <frc/DriverStation.h>
#include "auto/FollowPath.h"
#include "auto/DriveVelocity.h"
#include "systems/SwerveDrive.h"
#include "Locations.h"
#include "auto/StartShooter.h"


AutoShootBackUp::AutoShootBackUp(frc::DriverStation::Alliance alliance, int position) {
  // m_tasks.push_back(std::make_shared<DriveVelocity>(
  //   alliance == frc::DriverStation::kBlue ? -1 : 1, 0, 0));
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position),
      Locations::GetInstance().GetStartPosition(alliance, position)
        .TransformBy(frc::Transform2d{ (alliance ? -1 : 1) * 2_m, 0_m, 0_deg })
    }, false, false));
  m_tasks.push_back(std::make_shared<StartShooter>());
}