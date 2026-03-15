#include "auto/StopIntake.h"
#include "systems/SupaIntake.h"


void StopIntake::Start(double t) {SupaIntake::GetInstance().SetMotors(0.0);}
void StopIntake::Update(double t) {}
void StopIntake::Stop() {}
bool StopIntake::IsDone() const {return true;}
StopIntake::StopIntake() {}