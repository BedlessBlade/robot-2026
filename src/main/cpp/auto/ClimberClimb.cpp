#include "auto/ClimberClimb.h"
#include "systems/Climber.h"

using namespace rev::spark;

void ClimberClimb::Start(double t) {
    Climber::GetInstance().SetClimber(Constants::kClimbClimbed, SparkBase::ControlType::kMAXMotionPositionControl);
}

void ClimberClimb::Update(double t) {}

void ClimberClimb::Stop() {}

bool ClimberClimb::IsDone() const { return false; }

ClimberClimb::ClimberClimb() {}
