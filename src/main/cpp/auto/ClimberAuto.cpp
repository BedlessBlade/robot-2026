#include "Constants.h"
#include "systems/Climber.h"
#include "auto/ClimberAuto.h"
#include "auto/TaskList.h"
#include "auto/ParallelTask.h"

using namespace rev::spark;

void ClimberAuto::ClimberStowAuto() {
    Climber::GetInstance().SetClimber(Constants::kClimbStowed, SparkBase::ControlType::kMAXMotionPositionControl);
}
void ClimberAuto::ClimberClimbAuto() {
    Climber::GetInstance().SetClimber(Constants::kClimbClimbed, SparkBase::ControlType::kMAXMotionPositionControl);
}
void ClimberAuto::ClimberExtendAuto() {
    Climber::GetInstance().SetClimber(Constants::kClimbExtended, SparkBase::ControlType::kMAXMotionPositionControl);
}

void ClimberAuto::Start(double t) {}
void ClimberAuto::Update(double t) {}
void ClimberAuto::Stop() {}
bool ClimberAuto::IsDone() const { return false; }