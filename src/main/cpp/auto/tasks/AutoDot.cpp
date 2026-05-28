#include "auto/tasks/AutoDot.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <memory>

#include "Locations.h"

#include "auto/FollowPath.h"

#include "auto/DeployIntake.h"

// alliance - red = 0, blue = 1
AutoDot::AutoDot(frc::DriverStation::Alliance alliance, int position, int endBehavior){
  // todos: make more efficiency

  bool onLeft = position < 3;

  if (position != 2 && position != 4) { 
    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
        Locations::GetInstance().GetStartPosition(alliance, position),
        Locations::GetInstance().GetStartPosition(alliance, onLeft ? 2 : 4)
      }, false, false));
    // return;
  }

  
  // go over line and to dot
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position),
      Locations::GetInstance().GetCenterPosition(alliance, onLeft)[0],
      Locations::GetInstance().GetDotPosition(alliance, onLeft ? 1 : 3),
    }, false, true)); 
    m_tasks.push_back(std::make_shared<DeployIntake>());

}