#pragma once

#include "Constants.h"
#include "Task.h"
#include "systems/SupaIntake.h"

class StowIntake : public Task {
public:
    void Start(double t) override;
    void Update(double t) override;
    void Stop() override;
    bool IsDone() const override;

    StowIntake();

private:

};