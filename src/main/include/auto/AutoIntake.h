#pragma once

#include "Task.h"

class AutoIntake : public Task {
public:
  void Activate();
  void Deactivate();

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  AutoIntake();
};
