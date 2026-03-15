#include "auto/DeployIntake.h"
#include "systems/SupaIntake.h"


void DeployIntake::Start(double t) {SupaIntake::GetInstance().ToggleIntake();}
void DeployIntake::Update(double t) {}
void DeployIntake::Stop() {}
bool DeployIntake::IsDone() const {return true;}
DeployIntake::DeployIntake() {}