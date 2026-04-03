#include "auto/tasks/AutoCenterDefence.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Constants.h"
#include "Locations.h"
#include "Util.h"

#include "auto/FollowPath.h"
#include "auto/DriveVelocity.h"
#include "auto/Delay.h"

#include "systems/SwerveDrive.h"


AutoCenterDefence::AutoCenterDefence(frc::DriverStation::Alliance alliance, int position){  
  // todo: adjust delay for movement

  // alliance - red = 0, blue = 1
  
  bool onLeft = position < 3;

  // safeguard to prevent auto from running when not in the right position
  if (position != 2 && position != 4) { 
    // m_tasks.push_back(std::make_shared<FollowPath>(
    //   std::vector<frc::Pose2d>{
    //     Locations::GetInstance().GetStartPosition(alliance, position),
    //     Locations::GetInstance().GetStartPosition(alliance, position + (onLeft ? 1 : -1))
    //   }, false, false));  
  return; }

  // goes straight forward to the center of the field
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position),
      Locations::GetInstance().GetCenterPosition(alliance, onLeft)[5]
    }, false, false));
  
  // Moves forward to disturb fuel + other robots
    m_tasks.push_back(std::make_shared<DriveVelocity>(0, (onLeft ? -1 : 1) * 0.5, (onLeft ? -1 : 1) * Constants::kPathFollowingMaxW));
    m_tasks.push_back(std::make_shared<Delay>(1));
    m_tasks.push_back(std::make_shared<DriveVelocity>(0, 0, 0));
}