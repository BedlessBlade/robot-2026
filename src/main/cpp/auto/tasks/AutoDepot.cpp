#include "auto/tasks/AutoDepot.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Constants.h"
#include "Locations.h"

#include "auto/FollowPath.h"
#include "auto/Delay.h"

#include "auto/StartShooter.h"
#include "auto/StopShooter.h"

#include "auto/StartIntake.h"
#include "auto/StopIntake.h"
#include "auto/DeployIntake.h"
#include "auto/StowIntake.h"


AutoDepot::AutoDepot(frc::DriverStation::Alliance alliance, int position, bool endInCenter) {
  // alliance - red = 0, blue = 1

  bool onLeft = position < 3;

  // start intake deploy and move in front of depot
  m_tasks.push_back(std::make_shared<DeployIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position), 
      Locations::GetInstance().GetDepotPosition(alliance)[0]
    }, false, false));
  
  // start the intake and move forward to collect balls
  m_tasks.push_back(std::make_shared<StartIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetDepotPosition(alliance)[0], 
      Locations::GetInstance().GetDepotPosition(alliance)[1]
    }, false, false));
  
  // give time to process
  m_tasks.push_back(std::make_shared<Delay>(Constants::kIntakeAutoProcessingTime));
  m_tasks.push_back(std::make_shared<StopIntake>());

  // move back and spin around
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetDepotPosition(alliance)[1],
      Locations::GetInstance().GetDepotPosition(alliance)[0],
      Locations::GetInstance().GetDepotPosition(alliance)[2] 
    }, false, false));
  
  // shoot collected balls
  m_tasks.push_back(std::make_shared<StartShooter>());
  m_tasks.push_back(std::make_shared<Delay>(Constants::kAutoShootFullTime));

  // move into center
  if (endInCenter) {
    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
        Locations::GetInstance().GetDepotPosition(alliance)[2],
        Locations::GetInstance().GetStartPosition(alliance, 2),
        Locations::GetInstance().GetCenterPosition(alliance, true)[0]
      }, false, false));
  }
}