#include "auto/AutoCenter.h"

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
#include "auto/ToggleIntake.h"
#include "systems/SwerveDrive.h"


AutoCenter::AutoCenter(frc::DriverStation::Alliance alliance, bool onLeft) {
  // todos: tune delays at buzz on wednesday (or thursday)
  //        clean up and make more readable after assuring it works (ie convert all inches to meters)
  //        check to make sure degrees are accurate (i rushed them at the end)
  
  // alliance - red = 0, blue = 1

  // checks if the robot is on the top half of the field (left side for red, right side for blue)
  bool m_onTop = (alliance == 0 && onLeft) || (alliance == 1 && !onLeft);
  // bool m_onTop = SwerveDrive::GetInstance().GetPose2d().Y().value() < Constants::kFieldWidth / 2;
  
  // back up to make shooter work
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      frc::Pose2d{
        units::meter_t{(alliance == 0 ? Constants::kFieldLength - (0.5 * Constants::kStartLineOffset) : 0.5 * Constants::kStartLineOffset)},
        units::meter_t{(Constants::kFieldWidth / 2) + ((m_onTop == 0 ? -1 : 1) * Constants::kStartOffsetY)},
        (alliance == 0 ? 180_deg : 0_deg)
      }},
  false, false));

  // shoot preloaded balls
  m_tasks.push_back(std::make_shared<StartShooter>());
  m_tasks.push_back(std::make_shared<Delay>(2.0));  
  m_tasks.push_back(std::make_shared<StopShooter>());
  
  // goes to a position half a robot width past the edge of the ramp
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(), 
      frc::Pose2d{
        units::meter_t{alliance == 0 ? Constants::kFieldLength : 0} 
          + ((alliance == 0 ? -1 : 1) * units::meter_t{Constants::kStartLineOffset + (Constants::kRobotWidth / 2)} + 44.40_in),
        units::meter_t{(Constants::kFieldWidth / 2) + ((m_onTop == 0 ? -1 : 1) * Constants::kStartOffsetY)}, 
        (alliance == 0 ? 180_deg : 0_deg)
      }}, false, false));
  
  // moves the robot above/below the balls & aligns it
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      // x value is the middle of the field +- 1/4 length of the fuel rectangle. 
      frc::Pose2d{
        units::meter_t{Constants::kFieldLength / 2} + (alliance == 0 ? 17.975_in : -17.975_in), 
        (m_onTop ? 0_m + 25.62_in : (units::meter_t{Constants::kFieldWidth} - 25.62_in)), 
        (m_onTop ? -90_deg : 90_deg)
      }}, false, false));
  
  // starts intake & moves forward to collect balls
  m_tasks.push_back(std::make_shared<ToggleIntake>());
  m_tasks.push_back(std::make_shared<StartIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      // robot returns to starting y position, aligning with the ramp
      frc::Pose2d{
        units::meter_t{Constants::kFieldLength / 2} + (alliance == 0 ? 17.975_in : -17.975_in),
        units::meter_t{(Constants::kFieldWidth / 2) + ((m_onTop == 0 ? -1 : 1) * Constants::kStartOffsetY)}, 
        (m_onTop ? -90_deg : 90_deg)
      }}, false, false));
  
  // give time for intake to finish intaking (todo: tune probably. dont let this be too long)
  m_tasks.push_back(std::make_shared<Delay>(0.5));
  
  // stop intaking balls, rotate to face ramp
  m_tasks.push_back(std::make_shared<StopIntake>());
  m_tasks.push_back(std::make_shared<ToggleIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      frc::Pose2d{
        units::meter_t{Constants::kFieldLength / 2} + (alliance == 0 ? 17.975_in : -17.975_in),
        units::meter_t{(Constants::kFieldWidth / 2) + ((m_onTop == 0 ? -1 : 1) * Constants::kStartOffsetY)},  
        (alliance == 0 ? 180_deg : 0_deg)
      }}, false, false));

  // go over ramp and start shooting balls
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      frc::Pose2d{
        units::meter_t{(alliance == 0 ? Constants::kFieldLength - (0.5 * Constants::kStartLineOffset) : 0.5 * Constants::kStartLineOffset)},
        units::meter_t{(Constants::kFieldWidth / 2) + ((m_onTop == 0 ? -1 : 1) * Constants::kStartOffsetY)},
        (alliance == 0 ? 180_deg : 0_deg)
      }},
  false, false));
  m_tasks.push_back(std::make_shared<StartShooter>());
}