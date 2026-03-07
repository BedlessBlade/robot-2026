#include "auto/AutoStationaryShoot.h"
#include "systems/Shooter.h"
#include "systems/Indexer.h"

void AutoStationaryShoot::ShooterOff() {
    Indexer::GetInstance().StopIndexing();
    Shooter::GetInstance().SetShooterSpeed(0_rpm);
}
void AutoStationaryShoot::ShooterOn(units::angular_velocity::turns_per_second_t speed, double angle) {
    if (angle < 0.2) {
        Shooter::GetInstance().SetHoodState(0.2);
    } else if (angle > 1) {
        Shooter::GetInstance().SetHoodState(1);
    } else {
        Shooter::GetInstance().SetHoodState(angle);
    }

    Indexer::GetInstance().StartIndexing();
    Shooter::GetInstance().SetShooterSpeed(speed);
}

void AutoStationaryShoot::Update(double t) {}

void AutoStationaryShoot::Stop() {}

bool AutoStationaryShoot::IsDone() const { return false; }

AutoStationaryShoot::AutoStationaryShoot() {}