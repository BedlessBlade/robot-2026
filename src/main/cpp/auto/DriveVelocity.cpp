#include "auto/DriveVelocity.h"

#include "systems/SwerveDrive.h"

DriveVelocity::DriveVelocity(double vx, double vy, double w)
    : m_vx{vx}, m_vy{vy}, m_w{w} {}

void DriveVelocity::Start(double t) {
  SwerveDrive::GetInstance().DriveVelocity(m_vx, m_vy, m_w);
  m_done = true;
}

void DriveVelocity::Update(double t) {}

void DriveVelocity::Stop() {}

bool DriveVelocity::IsDone() const { return m_done; }
