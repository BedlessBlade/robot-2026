#include "auto/ShooterOff.h"
#include "systems/Shooter.h"

void ShooterOff::Start(double t) {
    Shooter::GetInstance().adjustMotorSpeed(0.0, 0.0);
}

void ShooterOff::Update(double t) {}

void ShooterOff::Stop() {}

bool ShooterOff::IsDone() const { return false; }

ShooterOff::ShooterOff() {}
