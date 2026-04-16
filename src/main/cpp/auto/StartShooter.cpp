#include "auto/StartShooter.h"
#include "systems/Shooter.h"

void StartShooter::Start(double t) { Shooter::GetInstance().StartShooting(); }
void StartShooter::Update(double t) {}
void StartShooter::Stop() {}
bool StartShooter::IsDone() const { return true; }
StartShooter::StartShooter() {}