#include "auto/AutoDepot.h"

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

#include "auto/StartIntake.h"
#include "auto/StopIntake.h"
#include "auto/DeployIntake.h"
#include "auto/StowIntake.h"


AutoDepot::AutoDepot(frc::DriverStation::Alliance alliance, int position) {
  // // this is test pseudo code for autos,  uses set points which still need to be made and this one has move shoot and climb
  
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position), 
      Locations::GetInstance().GetDepotPosition(alliance, true)
    }, false, false));
  m_tasks.push_back(std::make_shared<DeployIntake>());
  m_tasks.push_back(std::make_shared<StartIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetDepotPosition(alliance, true), 
      Locations::GetInstance().GetDepotPosition(alliance, false)
    }, false, false));
  m_tasks.push_back(std::make_shared<StopIntake>());
  m_tasks.push_back(std::make_shared<StowIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetDepotPosition(alliance, false), 
      Locations::GetInstance().GetDepotPosition(alliance, false).RotateBy(90_deg)
    }, false, false));
  m_tasks.push_back(std::make_shared<StartShooter>());
}