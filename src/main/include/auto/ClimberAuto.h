#pragma once

#include "Task.h"

class ClimberAuto : public Task {
public:
  void ClimberStowAuto();
  void ClimberClimbAuto();
  void ClimberExtendAuto();

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  ClimberAuto();
};