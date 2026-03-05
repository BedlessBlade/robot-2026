#pragma once

#include "Task.h"
#include <cmath>
#include <units/angular_velocity.h>

class AutoStationaryShoot : public Task {
public:
  void ShooterOff();
  void ShooterOn(units::angular_velocity::turns_per_second_t speed, double angle);

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  AutoStationaryShoot();
};