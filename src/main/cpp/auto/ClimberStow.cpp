#include "auto/ClimberStow.h"
#include "systems/Climber.h"

using namespace rev::spark;

void ClimberStow::Start(double t) {
    Climber::GetInstance().SetClimber(Constants::kClimbStowed, SparkBase::ControlType::kMAXMotionPositionControl);
}

void ClimberStow::Update(double t) {}

void ClimberStow::Stop() {}

bool ClimberStow::IsDone() const { return false; }

ClimberStow::ClimberStow() {}
