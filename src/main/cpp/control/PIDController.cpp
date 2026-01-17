#include "control/PIDController.h"

#include <frc/Timer.h>

PIDController::PIDController(double kP, double kI, double kD) {
  m_kP = kP;
  m_kI = kI;
  m_kD = kD;

  Reset();
}

void PIDController::Reset() {
  m_lastError = 0;
  m_integral = 0;
  m_lastTimestmap = frc::Timer::GetFPGATimestamp().value();
}

double PIDController::Update(double current, double setpoint, double dt) {
  double error = setpoint - current;

  if (dt < 0) {
    double timestamp = frc::Timer::GetFPGATimestamp().value();
    dt = timestamp - m_lastTimestmap;
    m_lastTimestmap = timestamp;
  }

  m_integral += error * dt;

  double rate = (error - m_lastError) / dt;
  m_lastError = error;

  return m_kP * error + m_kI * m_integral + m_kD * rate;
}
