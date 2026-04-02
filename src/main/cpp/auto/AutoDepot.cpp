#include "auto/AutoDepot.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Constants.h"
#include "Locations.h"

#include "auto/FollowPath.h"
#include "auto/TaskList.h"
#include "auto/Delay.h"

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
      Locations::GetInstance().GetDepotPosition(alliance)[0]
    }, false, false));
  m_tasks.push_back(std::make_shared<DeployIntake>());
  // m_tasks.push_back(std::make_shared<StartIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetDepotPosition(alliance)[0], 
      Locations::GetInstance().GetDepotPosition(alliance)[1]
    }, false, false));
  m_tasks.push_back(std::make_shared<Delay>(Constants::kIntakeAutoProcessingTime));
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetDepotPosition(alliance)[1],
      Locations::GetInstance().GetDepotPosition(alliance)[0]
    }, false, false));
  // m_tasks.push_back(std::make_shared<StopIntake>());
  // m_tasks.push_back(std::make_shared<StowIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetDepotPosition(alliance)[0], 
      frc::Pose2d{
        Locations::GetInstance().GetDepotPosition(alliance)[0].Translation(),
        (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)
      }
    }, false, false));
  // m_tasks.push_back(std::make_shared<StartShooter>());
}