#include "auto/ShooterAuto.h"
#include "systems/Shooter.h"
#include "systems/Indexer.h"

void ShooterAuto::ShooterOff() {
    Indexer::GetInstance().StopIndexing();
    Shooter::GetInstance().SetShooterSpeed(0_rpm);
}
void ShooterAuto::ShooterOn(units::angular_velocity::turns_per_second_t speed, double hoodAngle, double turretAngle) {
    if (hoodAngle < 0.2) {
        Shooter::GetInstance().SetHoodState(0.2);
    } else if (hoodAngle > 1) {
        Shooter::GetInstance().SetHoodState(1);
    } else {
        Shooter::GetInstance().SetHoodState(hoodAngle);
    }

    Shooter::GetInstance().SetAngle(turretAngle);

    Indexer::GetInstance().StartIndexing();
    Shooter::GetInstance().SetShooterSpeed(speed);
}


void ShooterAuto::Update(double t) {}

void ShooterAuto::Stop() {}

bool ShooterAuto::IsDone() const { return false; }

ShooterAuto::ShooterAuto() {}
