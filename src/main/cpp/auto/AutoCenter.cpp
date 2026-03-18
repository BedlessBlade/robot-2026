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


AutoCenter::AutoCenter(frc::DriverStation::Alliance alliance) {
  //alliance - red = 0, blue = 1

  //checks if the robot is on the top half of the field (left side for red, right side for blue)
  auto m_onTop = SwerveDrive::GetInstance().GetPose2d().Y().value() < Constants::kFieldWidth / 2;
  //shoot preloaded balls
  //todo: tune time @ buzz on wednesday
  m_tasks.push_back(std::make_shared<StartShooter>());
  m_tasks.push_back(std::make_shared<Delay>(2.0));  
  m_tasks.push_back(std::make_shared<StopShooter>());
  //Go completely over ramp
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(), // position might be optional depending on how we want to start matches
      frc::Pose2d{
        units::meter_t{(alliance ? Constants::kFieldLength - Constants::kStartLineOffset : Constants::kStartLineOffset)} + 44.40_in + units::meter_t{Constants::kRobotWidth / 2},
        SwerveDrive::GetInstance().GetPose2d().Y(),
        (alliance == 0 ? 180_deg : 0_deg)
      }}, false, false));
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      //x - field middle +- offset to get to middle of balls on alliance side. 
      frc::Pose2d{
        units::meter_t{Constants::kFieldLength / 2} + (alliance == 0 ? 17.975_in : -17.975_in), 
        (m_onTop ? 25.62_in : units::inch_t{Constants::kFieldWidth} - 25.62_in), 
        -90_deg
      }}, false, false));
  m_tasks.push_back(std::make_shared<ToggleIntake>());
  m_tasks.push_back(std::make_shared<StartIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      frc::Pose2d{
        SwerveDrive::GetInstance().GetPose2d().X(), 
        units::meter_t{(Constants::kFieldWidth / 2) + (m_onTop == 0 ? -1 : 1) * Constants::kStartOffsetY}, 
        (m_onTop ? 90_deg : -90_deg)
      }}, false, false));
  m_tasks.push_back(std::make_shared<StopIntake>());
  m_tasks.push_back(std::make_shared<ToggleIntake>());
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      SwerveDrive::GetInstance().GetPose2d().RotateBy((alliance == 0 ? 90_deg : -90_deg))},
  false, false));
  m_tasks.push_back(std::make_shared<FollowPath>(
    std::vector<frc::Pose2d>{
      SwerveDrive::GetInstance().GetPose2d(),
      //todo:insert correct final pos
      SwerveDrive::GetInstance().GetPose2d().TransformBy(frc::Transform2d{0_m, 0_m, 0_deg})},
  false, false));
  m_tasks.push_back(std::make_shared<StartShooter>());
}
