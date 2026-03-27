#include "Locations.h"

#include <cmath>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include "Constants.h"
#include "Util.h"

void Locations::Generate(frc::DriverStation::Alliance alliance) { }

frc::Pose2d Locations::GetOutpostPosition(frc::DriverStation::Alliance alliance) const {
  return frc::Pose2d{ 20_in, 25.62_in, 180_deg }
    .RotateAround(Constants::kFieldCenter, alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg);
}

std::vector<frc::Pose2d> Locations::GetDepotPosition(frc::DriverStation::Alliance alliance) const {
  return std::vector<frc::Pose2d>{
    frc::Pose2d{
      13_in + units::meter_t{Constants::kRobotWidth / 2} + 27_in,
      units::meter_t{Constants::kFieldWidth} - 82.84_in,
      180_deg
    }.RotateAround(Constants::kFieldCenter, alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg),
    frc::Pose2d{
      13_in + units::meter_t{Constants::kRobotWidth / 2},
      units::meter_t{Constants::kFieldWidth} - 82.32_in,
      180_deg
    }.RotateAround(Constants::kFieldCenter, alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)
  };
}

std::vector<frc::Pose2d> Locations::GetAutoCenterPositions(frc::DriverStation::Alliance alliance, bool onLeft) const {
  return std::vector<frc::Pose2d>{
    // 0 - Move to shooting position
    frc::Pose2d{
      units::meter_t{0.5 * Constants::kStartLineOffset},
      units::meter_t{(Constants::kFieldWidth / 2) + ((onLeft ? 1 : -1) * Constants::kStartOffsetY)},
      0_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),

    // 1 - Move straight to 1ft across ramp
    frc::Pose2d{
      units::meter_t{Constants::kStartLineOffset + (Constants::kRobotWidth / 2)} + 44.40_in + 1_ft,
      units::meter_t{(Constants::kFieldWidth / 2) + ((onLeft ? 1 : -1) * Constants::kStartOffsetY)}, 
      0_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),

    // 2 - Move along y-axis to the wall (directly in front of trench)
    frc::Pose2d{
      units::meter_t{Constants::kStartLineOffset + (Constants::kRobotWidth / 2)} + 56.40_in,
      (onLeft ? (units::meter_t{Constants::kFieldWidth} - 25.62_in) : 0_m + 25.62_in),
      (onLeft ? -1 : 1) * 90_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),

    // 3 - Move forward to be in line (y) with balls in center
    frc::Pose2d{
      units::meter_t{Constants::kFieldLength / 2} - 17.975_in, 
      (onLeft ? (units::meter_t{Constants::kFieldWidth} - 25.62_in) : 0_m + 25.62_in), 
      (onLeft ? -1 : 1) * 90_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),

    // 4 - Move straight forward to collect balls. Go to center of field on y axis
    frc::Pose2d{
      units::meter_t{Constants::kFieldLength / 2} - 17.975_in,
      units::meter_t{(Constants::kFieldWidth / 2) + (onLeft ? 1 : -1) * (13 + Constants::kRobotWidth / 2)}, 
      (onLeft ? -1 : 1) * 90_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),

    // // 5 - Rotate to face ramp
    // frc::Pose2d{
    //   units::meter_t{Constants::kFieldLength / 2} - 17.975_in,
    //   units::meter_t{(Constants::kFieldWidth / 2) + ((onLeft ? 1 : -1) * Constants::kStartOffsetY)},  
    //   0_deg
    // }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg))
  };
}

frc::Pose2d Locations::GetStartPosition(frc::DriverStation::Alliance alliance, int i) const {
  double y = Constants::kFieldWidth / 2;
  double onLeft = i < 3;

  switch (i) {
  case 1: case 5: 
    y += (onLeft ? 1 : -1) * Constants::kStartOffsetYFar;
    break;
  case 2: case 4: 
    y += (onLeft ? 1 : -1) * Constants::kStartOffsetY;
    break;
  default:
    break;
  }

  return frc::Pose2d{
    units::meter_t{Constants::kStartLineOffset - Constants::kStartOffsetX}, 
    units::meter_t{y}, 0_deg
  }.RotateAround(Constants::kFieldCenter, alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg);
}

Locations::Locations() {}
