#include "auto/AutoCenterOne.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Constants.h"
#include "Locations.h"
#include "Util.h"

#include "auto/FollowPath.h"
#include "auto/TaskList.h"
#include "auto/Delay.h"
#include "auto/StartShooter.h"
#include "auto/StopShooter.h"
#include "auto/StartIntake.h"
#include "auto/StopIntake.h"
#include "auto/DeployIntake.h"
#include "auto/StowIntake.h"
#include "systems/SwerveDrive.h"


AutoCenterOne::AutoCenterOne(frc::DriverStation::Alliance alliance, int position){
  // todos: clean up and make more readable
  //        tune shooting time
  
  bool onLeft = position < 3;

  // safeguard to prevent auto from running when not in the right position
  if (position != 2 && position != 4) {
    return;
  }

  // back up to shoot
  m_tasks.push_back(std::make_shared<DeployIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position),
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[0]
    }, false, false));

  // shoot preloaded balls
  m_tasks.push_back(std::make_shared<StartShooter>());
  m_tasks.push_back(std::make_shared<Delay>(2.0));  
  m_tasks.push_back(std::make_shared<StopShooter>());
  
  // goes over the ramp and line up to collect balls
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[0],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[1],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[2],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[3]
    }, false, false));
  
  // starts intake & moves forward to collect balls
  m_tasks.push_back(std::make_shared<StartIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[3],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[4],
    }, false, false));
  
  // give time for intake to finish intaking
  m_tasks.push_back(std::make_shared<Delay>(Constants::kIntakeAutoProcessingTime));
  
  // stop intaking balls, go to ramp, go over and move to shooting position
  m_tasks.push_back(std::make_shared<StopIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[4],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[1],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[0],
    }, false, false));
  
  // shoot balls for 2 seconds - should be enough to empty hopper
  m_tasks.push_back(std::make_shared<StartShooter>());
  m_tasks.push_back(std::make_shared<Delay>(2.0));
  m_tasks.push_back(std::make_shared<StopShooter>());

  // Go back over ramp & start teleop in neutral zone
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[0],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[1],
    }, false, false));
  m_tasks.push_back(std::make_shared<Delay>(1.0));
}