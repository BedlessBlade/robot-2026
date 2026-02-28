#include "Constants.h"
#include "systems/Climber.h"
#include "auto/ClimberAuto.h"

using namespace rev::spark;

void ClimberAuto::ClimberStowAuto() {
    Climber::GetInstance().SetClimber(Constants::kClimbStowed, SparkBase::ControlType::kMAXMotionPositionControl);
}

void ClimberAuto::Start(double t) {}
void ClimberAuto::Update(double t) {}
void ClimberAuto::Stop() {}
bool ClimberAuto::IsDone() const { return false; }

ClimberAuto::ClimberAuto() {}
