#include "auto/ClimberExtend.h"
#include "systems/Climber.h"

using namespace rev::spark;

void ClimberExtend::Start(double t) {
    Climber::GetInstance().SetClimber(Constants::kClimbExtended, SparkBase::ControlType::kMAXMotionPositionControl);
}

void ClimberExtend::Update(double t) {}

void ClimberExtend::Stop() {}

bool ClimberExtend::IsDone() const { return false; }

ClimberExtend::ClimberExtend() {}
