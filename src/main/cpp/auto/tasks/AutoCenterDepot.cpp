#include "auto/tasks/AutoCenterDepot.h"

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


AutoCenterDepot::AutoCenterDepot(frc::DriverStation::Alliance alliance, int position, bool endInCenter) {

    bool onLeft = position < 3;

    // safeguard to prevent auto from running when not in the right position
    if (position != 2 && position != 4) { 
        // m_tasks.push_back(std::make_shared<FollowPath>(
        //   std::vector<frc::Pose2d>{
        //     Locations::GetInstance().GetStartPosition(alliance, position),
        //     Locations::GetInstance().GetStartPosition(alliance, position + (onLeft ? 1 : -1))
        //   }, false, false));
    return; }

    // Go over ramp, move above/below ball island
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetStartPosition(alliance, position),
            Locations::GetInstance().GetCenterPosition(alliance, onLeft)[0],
            Locations::GetInstance().GetCenterPosition(alliance, onLeft)[1],
            Locations::GetInstance().GetCenterPosition(alliance, onLeft)[2]
        }, false, false));
    

    // starts intake & moves forward to collect balls
    m_tasks.push_back(std::make_shared<DeployIntake>());
    m_tasks.push_back(std::make_shared<StartIntake>());

    // path for left side - follows regular left center path
    if (onLeft) {
        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetCenterPosition(alliance, true)[2],
                Locations::GetInstance().GetCenterPosition(alliance, true)[3],
            }, false, false));
        
        // delay to let intake process
        m_tasks.push_back(std::make_shared<Delay>(Constants::kIntakeAutoProcessingTime));
    
        // stop intaking balls, go to ramp & go over ramp and get in shooting position
        m_tasks.push_back(std::make_shared<StopIntake>());
        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetCenterPosition(alliance, true)[3],
                Locations::GetInstance().GetCenterPosition(alliance, true)[0],
                Locations::GetInstance().GetShootingPosition(alliance, true)
            }, false, false));

    // path for right side - crosses neutral zone to get to left side, then follows left center to end
    } else {
        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetCenterPosition(alliance, false)[2],
                Locations::GetInstance().GetCenterPosition(alliance, false)[4],
            }, false, false));

        // delay to let intake process
        m_tasks.push_back(std::make_shared<Delay>(Constants::kIntakeAutoProcessingTime));
        
        // cross ramp and go to shooting position
        m_tasks.push_back(std::make_shared<FollowPath>(
            std::vector<frc::Pose2d>{
                Locations::GetInstance().GetCenterPosition(alliance, false)[4],
                Locations::GetInstance().GetShootingPosition(alliance, true),
            }, false, false));
        
    }

    // shoot everything collected from the ball island
    m_tasks.push_back(std::make_shared<StartShooter>());
    m_tasks.push_back(std::make_shared<Delay>(Constants::kAutoShootFullTime));
    m_tasks.push_back(std::make_shared<StopShooter>());

    // Move in front of depot
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetShootingPosition(alliance, true),
            Locations::GetInstance().GetDepotPosition(alliance)[0]
        }, false, false));

    // Move forward to collect balls
    m_tasks.push_back(std::make_shared<StartIntake>());
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetDepotPosition(alliance)[0], 
            Locations::GetInstance().GetDepotPosition(alliance)[1]
        }, false, false));
    
    // Delay for intake processing
    m_tasks.push_back(std::make_shared<Delay>(Constants::kIntakeAutoProcessingTime));

    // Move back, turn around
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetDepotPosition(alliance)[1],
            Locations::GetInstance().GetDepotPosition(alliance)[0],
            Locations::GetInstance().GetDepotPosition(alliance)[2]
        }, false, false));

    // Shoot
    m_tasks.push_back(std::make_shared<StartShooter>());
    m_tasks.push_back(std::make_shared<Delay>(Constants::kAutoShootFullTime));
    
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetDepotPosition(alliance)[2],
            Locations::GetInstance().GetStartPosition(alliance, 2),
            Locations::GetInstance().GetCenterPosition(alliance, true)[0]
        }, false, false));
}