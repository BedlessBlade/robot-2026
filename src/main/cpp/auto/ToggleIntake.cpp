#include "auto/ToggleIntake.h"
#include "systems/SupaIntake.h"


void ToggleIntake::Start(double t) { SupaIntake::GetInstance().ToggleIntake(); }
void ToggleIntake::Update(double t) {}
void ToggleIntake::Stop() {}
bool ToggleIntake::IsDone() const {return true;}
ToggleIntake::ToggleIntake() {}