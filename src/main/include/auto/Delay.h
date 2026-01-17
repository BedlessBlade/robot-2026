#pragma once

#include "Task.h"

class Delay : public Task {
public:
  Delay(double t);

  void Start(double t) override;
  void Update(double t) override;
  bool IsDone() const override;

private:
  double m_t;
  double m_doneTime;
  double m_lastTime = -1;
};
