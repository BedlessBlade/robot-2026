#pragma once

#include "Robot.h"

class System {
public:
  virtual void Update(Robot::Mode mode, double t);
};
