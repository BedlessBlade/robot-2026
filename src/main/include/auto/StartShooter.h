#pragma once

#include "Constants.h"
#include "Task.h"
#include "systems/Shooter.h"

class StartShooter : public Task {
public:
    void Start(double t) override;
    void Update(double t) override;
    void Stop() override;
    bool IsDone() const override;

    StartShooter();

private:

};