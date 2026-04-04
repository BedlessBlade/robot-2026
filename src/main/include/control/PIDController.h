#pragma once
#include <frc/filter/LinearFilter.h>

class PIDController {
public:
  PIDController(double kP, double kI, double kD, double Tau);

  void Reset();

  // Passing -1 to dt will keep track of the dt automatically. Note that you
  // must use the same dt source for all calls to Update after Reset or else it
  // will break.
  double Update(double current, double setpoint, double dt = -1);

private:
  double m_kP, m_kI, m_kD, m_Tau;
  double m_lastError, m_integral, m_lastTimestmap;

  // Derivative filter
  frc::LinearFilter<double> dTermFilter = frc::LinearFilter<double>::SinglePoleIIR(m_Tau, 0.005_s);
};
