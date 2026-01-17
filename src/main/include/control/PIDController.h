#pragma once

class PIDController {
public:
  PIDController(double kP, double kI, double kD);

  void Reset();

  // Passing -1 to dt will keep track of the dt automatically. Note that you
  // must use the same dt source for all calls to Update after Reset or else it
  // will break.
  double Update(double current, double setpoint, double dt = -1);

private:
  double m_kP, m_kI, m_kD;
  double m_lastError, m_integral, m_lastTimestmap;
};
