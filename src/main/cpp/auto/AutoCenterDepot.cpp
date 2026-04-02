#include "auto/AutoCenterDepot.h"

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


AutoCenterDepot::AutoCenterDepot(frc::DriverStation::Alliance alliance, int position) {

  bool onLeft = position < 3;

  // safeguard to prevent auto from running when not in the right position
  if (position != 2 && position != 4) {
    return;
  }

  // goes to a position half a robot width past the edge of the ramp
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      Locations::GetInstance().GetStartPosition(alliance, position),
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[1],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[2],
      Locations::GetInstance().GetAutoCenterPositions(alliance, onLeft)[3]
    }, false, false));
  

  // starts intake & moves forward to collect balls
  m_tasks.push_back(std::make_shared<DeployIntake>());
  m_tasks.push_back(std::make_shared<StartIntake>());

    if (onLeft) {
        m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetAutoCenterPositions(alliance, true)[3],
            Locations::GetInstance().GetAutoCenterPositions(alliance, true)[4],
        }, false, false));

        m_tasks.push_back(std::make_shared<Delay>(0.5));
    
        // stop intaking balls, go to ramp & go over ramp 
        m_tasks.push_back(std::make_shared<StopIntake>());
        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetAutoCenterPositions(alliance, true)[4],
                Locations::GetInstance().GetAutoCenterPositions(alliance, true)[1],
                Locations::GetInstance().GetAutoCenterPositions(alliance, true)[0],
            }, false, false));

        m_tasks.push_back(std::make_shared<StartShooter>());
        m_tasks.push_back(std::make_shared<Delay>(2.0));
        m_tasks.push_back(std::make_shared<StopShooter>());

        // Go back over ramp
        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetAutoCenterPositions(alliance, true)[0],
                Locations::GetInstance().GetDepotPosition(alliance)[0]
        }, false, false));

    } else {

        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetAutoCenterPositions(alliance, false)[3],
                Locations::GetInstance().GetAutoCenterPositions(alliance, false)[5],
            }, false, false));

        m_tasks.push_back(std::make_shared<Delay>(0.5));

        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetAutoCenterPositions(alliance, true)[5],
                Locations::GetInstance().GetAutoCenterPositions(alliance, true)[0],
            }, false, false));
        
        m_tasks.push_back(std::make_shared<StartShooter>());
        m_tasks.push_back(std::make_shared<Delay>(2.0));
        m_tasks.push_back(std::make_shared<StopShooter>());

        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetAutoCenterPositions(alliance, true)[0], 
                Locations::GetInstance().GetDepotPosition(alliance)[0]
            }, false, false));
        
        
    }
  
    m_tasks.push_back(std::make_shared<StartIntake>());
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetDepotPosition(alliance)[0], 
            Locations::GetInstance().GetDepotPosition(alliance)[1]
        }, false, false));
    m_tasks.push_back(std::make_shared<Delay>(0.5));
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetDepotPosition(alliance)[1],
            Locations::GetInstance().GetDepotPosition(alliance)[0]
        }, false, false));
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
        Locations::GetInstance().GetDepotPosition(alliance)[0], 
        frc::Pose2d{
            Locations::GetInstance().GetDepotPosition(alliance)[0].Translation(),
            (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)
        }
    }, false, false));
  
  // give time for intake to finish intaking
  


}