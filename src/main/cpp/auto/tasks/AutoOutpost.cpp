#include "auto/tasks/AutoOutpost.h"

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

AutoOutpost::AutoOutpost(frc::DriverStation::Alliance alliance, int position, bool endInCenter) {
  bool onLeft = position < 3;

  // back up to shoot
  m_tasks.push_back(std::make_shared<DeployIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position),
      Locations::GetInstance().GetShootingPosition(alliance, onLeft)
    }, false, false));

  // shoot preloaded balls
  m_tasks.push_back(std::make_shared<StartShooter>());

  //Drives to Outpost
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetShootingPosition(alliance, onLeft),
      Locations::GetInstance().GetOutpostPosition(alliance)[0],
      Locations::GetInstance().GetOutpostPosition(alliance)[1],
    }, false, false));

  //Stops shooter and starts intake once at Outpost to catch balls
  m_tasks.push_back(std::make_shared<StartIntake>());
  m_tasks.push_back(std::make_shared<Delay>(Constants::kAutoShootFullTime)); //CHANGE TIME AT TESTING!

  //Stops intake and shooter after all balls are emptied
  m_tasks.push_back(std::make_shared<StopIntake>());
  m_tasks.push_back(std::make_shared<StopShooter>());

  //Drives to neutral zone
  if (endInCenter) {
    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
        Locations::GetInstance().GetOutpostPosition(alliance)[1],
        Locations::GetInstance().GetStartPosition(alliance, 4),
        Locations::GetInstance().GetCenterPosition(alliance, false)[0]
      }, false, false));
  }
}