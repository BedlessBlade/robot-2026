#include "auto/AutoShootBackUp.h"
#include "auto/TaskList.h"
#include <frc/DriverStation.h>
#include "auto/FollowPath.h"
#include "systems/SwerveDrive.h"
#include "auto/StartShooter.h"


AutoShootBackUp::AutoShootBackUp(frc::DriverStation::Alliance alliance) {
    frc::Pose2d pose = SwerveDrive::GetInstance().GetPose2d();
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            SwerveDrive::GetInstance().GetPose2d(),
            SwerveDrive::GetInstance().GetPose2d().TransformBy(frc::Transform2d{2_m, 0_m, 0_deg})},   
    false, false));
    m_tasks.push_back(std::make_shared<StartShooter>());
}