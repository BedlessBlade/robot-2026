#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <vector>

#include "Robot.h"

class Locations {
public:
  static Locations &GetInstance() {
    static Locations instance;
    return instance;
  }

  void Generate(frc::DriverStation::Alliance alliance);
  const std::vector<frc::Pose2d> GetCoralPositions() const;
  const std::vector<frc::Pose2d> GetAlgaePositions() const;
  const std::vector<frc::Pose2d> GetFeederPositions() const;

  frc::Pose2d GetStartPosition(frc::DriverStation::Alliance alliance,
                               int i) const;

private:
  Locations();

  std::vector<frc::Pose2d> m_coralPositions, m_algaePositions;
};
