#include "auto/Delay.h"
#include "systems/SwerveDrive.h"

Delay::Delay(double t) : m_t{t} { m_doneTime = 0; }

void Delay::Start(double t) { 
  SwerveDrive::GetInstance().Brake();
  m_doneTime = t + m_t; 
}

void Delay::Update(double t) { m_lastTime = t; }

bool Delay::IsDone() const { 
  if (m_lastTime >= m_doneTime) {
    SwerveDrive::GetInstance().Coast();
    return true;
  }
  return false; 
}
