#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/DoubleArrayTopic.h>

namespace nt { class NetworkTable; }

class QuestNav {
public:
    QuestNav();
    frc::Pose2d GetQuestPose();
    void Calibrate();

private:

    // Store the Subscriber type (Subscribe() returns SubscriberType)
    nt::DoubleArrayTopic::SubscriberType m_posSub;
    nt::DoubleArrayTopic::SubscriberType m_eulerSub;
};
