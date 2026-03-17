#include "auto/AutoOutpost.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Constants.h"
#include "Locations.h"

#include "auto/FollowPath.h"
#include "auto/TaskList.h"
#include "auto/StartShooter.h"
#include "auto/StopShooter.h"
#include "systems/SwerveDrive.h"
// #include ""


AutoOutpost::AutoOutpost(frc::DriverStation::Alliance alliance) {

  // this is test pseudo code for autos,  uses set points which still need to be made and this one has move shoot and climb
    m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
          SwerveDrive::GetInstance().GetPose2d(), // position might be optional depending on how we want to start matches
          Locations::GetInstance().GetOutpostPosition(alliance)},
      false, false));
    m_tasks.push_back(std::make_shared<StartShooter>());


    // m_tasks.push_back(std::make_shared<StartIntake>());
    // m_tasks.push_back(std::make_shared<Delay>(1.0)); // should be tested and changed
    // m_tasks.push_back(std::make_shared<FollowPath>(
      // std::vector<frc::Pose2d>{
          // Locations::GetInstance().GetShootPosition(alliance), // just so we can have a set point for when we want to shoot
      // false, false));
    // m_tasks.push_back(std::make_shared<StartShooter>());
    // m_tasks.push_back(std::make_shared<FollowPath>(
    //   std::vector<frc::Pose2d>{
    //       Locations::GetInstance().GetClimbPosition(alliance),
    //   false, false));


}
