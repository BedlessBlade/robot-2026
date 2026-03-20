#include "auto/AutoOutpost.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Constants.h"
#include "Locations.h"

#include "auto/FollowPath.h"
#include "auto/TaskList.h"
#include "auto/StartShooter.h"
#include "auto/StopShooter.h"
#include "systems/SwerveDrive.h"
// #include ""


AutoOutpost::AutoOutpost(frc::DriverStation::Alliance alliance, int position) {
    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
          Locations::GetInstance().GetStartPosition(alliance, position),
          Locations::GetInstance().GetOutpostPosition(alliance)},
      false, false));
    m_tasks.push_back(std::make_shared<StartShooter>());
}