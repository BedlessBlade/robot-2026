#include "auto/StartIntake.h"
#include "systems/SupaIntake.h"
#include "Constants.h"


void StartIntake::Start(double t) {SupaIntake::GetInstance().SetMotors(Constants::kIntakeForward);}
void StartIntake::Update(double t) {}
void StartIntake::Stop() {}
bool StartIntake::IsDone() const {return true;}
StartIntake::StartIntake() {}