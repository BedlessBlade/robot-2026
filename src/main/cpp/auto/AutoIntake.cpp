#include "auto/AutoIntake.h"
#include "systems/SupaIntake.h"

void AutoIntake::Activate() {
    SupaIntake::GetInstance().Extend();
    SupaIntake::GetInstance().SpinIn();
}

void AutoIntake::Deactivate() {
    SupaIntake::GetInstance().Retract();
    SupaIntake::GetInstance().SpinStop();
}

void AutoIntake::Start(double t) {}

void AutoIntake::Update(double t) {}

void AutoIntake::Stop() {}

bool AutoIntake::IsDone() const { return false; }

AutoIntake::AutoIntake() {}