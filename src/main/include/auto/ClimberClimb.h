#pragma once

#include "Task.h"

class ClimberClimb : public Task {
public:
  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  ClimberClimb();
};