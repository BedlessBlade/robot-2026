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

  frc::Pose2d GetStartPosition(frc::DriverStation::Alliance alliance, int position) const;
  frc::Pose2d GetOutpostPosition(frc::DriverStation::Alliance alliance) const;
  frc::Pose2d GetShootingPosition(frc::DriverStation::Alliance alliance, bool onLeft) const;

  std::vector<frc::Pose2d> GetDepotPosition(frc::DriverStation::Alliance alliance) const;
  std::vector<frc::Pose2d> GetCenterPosition(frc::DriverStation::Alliance alliance, bool onLeft) const;


private:
  Locations();
};