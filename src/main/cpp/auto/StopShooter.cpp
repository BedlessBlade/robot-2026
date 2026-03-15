#include "auto/StopShooter.h"

#include "systems/Shooter.h"

void StopShooter::Start(double t) {Shooter::GetInstance().StopShooting();}
void StopShooter::Update(double t) {}
void StopShooter::Stop() {}
bool StopShooter::IsDone() const {return true;}
StopShooter::StopShooter() {}