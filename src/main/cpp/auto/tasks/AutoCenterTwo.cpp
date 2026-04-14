#include "auto/tasks/AutoCenterTwo.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Constants.h"
#include "Locations.h"
#include "Util.h"

#include "auto/FollowPath.h"
#include "auto/Delay.h"

#include "auto/StartShooter.h"
#include "auto/StopShooter.h"

#include "auto/StartIntake.h"
#include "auto/StopIntake.h"
#include "auto/DeployIntake.h"
#include "auto/StowIntake.h"

#include "systems/SwerveDrive.h"


AutoCenterTwo::AutoCenterTwo(frc::DriverStation::Alliance alliance, int position){  
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

  m_tasks.push_back(std::make_shared<DeployIntake>());
  
  // run twice
  for (int i = 0; i < 2; i++) {
  // go over ramp
    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
        Locations::GetInstance().GetStartPosition(alliance, position),
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[0]
      }, false, false));

    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[0],
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[1],
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[2]
      }, false, false));
    
    // start intake & move forward to collect balls
    m_tasks.push_back(std::make_shared<StartIntake>());
    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[2],
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[3],
      }, false, false));
    
    // give time for intake to finish intaking
    m_tasks.push_back(std::make_shared<Delay>(Constants::kIntakeAutoProcessingTime));
    
    // stop intaking balls, move into shooting position
    m_tasks.push_back(std::make_shared<StopIntake>());
    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[3],
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[0]
      }, false, false));

    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
        Locations::GetInstance().GetCenterPosition(alliance, onLeft)[0],
        Locations::GetInstance().GetShootingPosition(alliance, onLeft),
      }, false, false));

    // empty the hopper
    m_tasks.push_back(std::make_shared<StartShooter>());
    m_tasks.push_back(std::make_shared<Delay>(Constants::kAutoShootFullTime));
    m_tasks.push_back(std::make_shared<StopShooter>());
  }

  // Go back over ramp & start teleop in neutral zone
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetShootingPosition(alliance, onLeft),
      Locations::GetInstance().GetCenterPosition(alliance, onLeft)[0],
    }, false, false));
}