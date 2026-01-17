#include "auto/FollowPath.h"

#include "Constants.h"
#include "systems/SwerveDrive.h"

FollowPath::FollowPath(std::vector<frc::Pose2d> points, bool resetPose,
                       bool persist)
    : m_points{points}, m_resetPose{resetPose}, m_persist{persist} {}

void FollowPath::Start(double t) {
  if (m_resetPose && m_points.size() > 0) {
    SwerveDrive::GetInstance().ResetPose(m_points[0]);
  }

  if (m_points.size() > 1) {
    m_pointIndex++;
  }

  m_started = true;

  SwerveDrive::GetInstance().DisableRamp();
}

void FollowPath::Update(double t) {
  if (AtPoint() && m_pointIndex < m_points.size() - 1) {
    m_pointIndex++;
    for (int i = 0; i < 3; i++) {
      m_controllers[i].Reset();
    }
  }

  auto robotPose = SwerveDrive::GetInstance().GetPose2d();
  auto vx =
      m_controllers[0].Update(robotPose.Translation().X().value(),
                              m_points[m_pointIndex].Translation().X().value());
  if (vx < -Constants::kPathFollowingMaxV) {
    vx = -Constants::kPathFollowingMaxV;
  }
  if (vx > Constants::kPathFollowingMaxV) {
    vx = Constants::kPathFollowingMaxV;
  }

  auto vy =
      m_controllers[1].Update(robotPose.Translation().Y().value(),
                              m_points[m_pointIndex].Translation().Y().value());
  if (vy < -Constants::kPathFollowingMaxV) {
    vy = -Constants::kPathFollowingMaxV;
  }
  if (vy > Constants::kPathFollowingMaxV) {
    vy = Constants::kPathFollowingMaxV;
  }

  double angleSetpoint = m_points[m_pointIndex].Rotation().Radians().value();
  if (angleSetpoint < -Constants::kPathFollowingMaxV) {
    angleSetpoint = -Constants::kPathFollowingMaxW;
  }
  if (vx > Constants::kPathFollowingMaxV) {
    angleSetpoint = Constants::kPathFollowingMaxW;
  }
  double currentAngle = robotPose.Rotation().Radians().value();
  double angleError = angleSetpoint - currentAngle;
  if (angleError > M_PI) {
    angleSetpoint -= 2 * M_PI;
  }

  auto w = m_controllers[2].Update(currentAngle, angleSetpoint);

  SwerveDrive::GetInstance().DriveVelocity(vx, vy, w);
}

void FollowPath::Stop() {
  SwerveDrive::GetInstance().DriveVelocity(0, 0, 0);
  SwerveDrive::GetInstance().EnableRamp();
}

bool FollowPath::IsDone() const {
  return m_started && !m_persist && AtPoint() &&
         SwerveDrive::GetInstance().VelocityMagnitude() <
             Constants::kPathFollowingVelocityTolerance;
}

bool FollowPath::AtPoint() const {
  return SwerveDrive::GetInstance()
             .GetPose2d()
             .Translation()
             .Distance(m_points[m_pointIndex].Translation())
             .value() < Constants::kPathFollowingTolerance;
}
