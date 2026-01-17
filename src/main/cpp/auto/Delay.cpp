#include "auto/Delay.h"

Delay::Delay(double t) : m_t{t} { m_doneTime = 0; }

void Delay::Start(double t) { m_doneTime = t + m_t; }

void Delay::Update(double t) { m_lastTime = t; }

bool Delay::IsDone() const { return m_lastTime >= m_doneTime; }
