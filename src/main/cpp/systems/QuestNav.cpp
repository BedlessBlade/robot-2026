#include "systems/QuestNav.h"
#include <networktables/NetworkTableInstance.h>

QuestNav::QuestNav() {
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("questnav");
    
    // 2025/2026 update: Use subscribers and DoubleArrays
    m_posSub = table->GetDoubleArrayTopic("position").Subscribe({});
    m_eulerSub = table->GetDoubleArrayTopic("eulerAngles").Subscribe({});
}

frc::Pose2d QuestNav::GetQuestPose() {
    auto position = m_posSub.Get();
    auto euler = m_eulerSub.Get();

    // Check size: QuestNav now sends [x, y, z] and [roll, pitch, yaw]
    if (position.size() < 3 || euler.size() < 3) {
        return frc::Pose2d();
    }

    // QuestNav 2026: position[0]=X, position[2]=Y (Quest Z), euler[1]=Yaw
    // Use parentheses for single-argument construction to avoid brace-init ambiguity
    return frc::Pose2d(
        units::meter_t(position[0]),
        units::meter_t(position[2]),
        frc::Rotation2d(units::degree_t(euler[1]))
    );
}

void QuestNav::Calibrate() {
    // Implement via the "resetPose" entry in the questnav table if needed
}
