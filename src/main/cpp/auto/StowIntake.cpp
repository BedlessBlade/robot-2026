#include "auto/StowIntake.h"
#include "systems/SupaIntake.h"


void StowIntake::Start(double t) {SupaIntake::GetInstance().SetIntake(false);}
void StowIntake::Update(double t) {}
void StowIntake::Stop() {}
bool StowIntake::IsDone() const {return true;}
StowIntake::StowIntake() {}