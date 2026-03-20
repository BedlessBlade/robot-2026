#include "auto/AutoShootBackUp.h"
#include "auto/TaskList.h"
#include <frc/DriverStation.h>
#include "auto/FollowPath.h"
#include "auto/DriveVelocity.h"
#include "systems/SwerveDrive.h"
#include "auto/StartShooter.h"


AutoShootBackUp::AutoShootBackUp(frc::DriverStation::Alliance alliance) {
    // m_tasks.push_back(std::make_shared<DriveVelocity>(
    //   alliance == frc::DriverStation::kBlue ? -1 : 1, 0, 0));
    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            SwerveDrive::GetInstance().GetPose2d(),
            frc::Pose2d{
                units::meter_t{Constants::kFieldLength - (Constants::kStartLineOffset - Constants::kStartOffsetX)} + 2_m,
                units::meter_t{(Constants::kFieldWidth / 2) + Constants::kStartOffsetY},
                180_deg
            }},
            // SwerveDrive::GetInstance().GetPose2d().TransformBy(frc::Transform2d{2_m, 0_m, 0_deg})},   
    false, false));
    m_tasks.push_back(std::make_shared<StartShooter>());
}