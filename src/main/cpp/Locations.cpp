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

frc::Pose2d Locations::GetDepotPosition(frc::DriverStation::Alliance alliance, bool withRobotOffset) const {
  return frc::Pose2d{ 
    27_in + units::meter_t{withRobotOffset ? Constants::kRobotWidth / 2 : 0}, 
    units::meter_t{Constants::kFieldWidth} - 82.32_in, 
    180_deg 
  }.RotateAround(Constants::kFieldCenter, alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg);
}

std::vector<frc::Pose2d> Locations::GetAutoCenterPositions(frc::DriverStation::Alliance alliance, bool onLeft) const {
  return std::vector<frc::Pose2d>{
    frc::Pose2d{
      units::meter_t{0.5 * Constants::kStartLineOffset},
      units::meter_t{(Constants::kFieldWidth / 2) + ((onLeft ? -1 : 1) * Constants::kStartOffsetY)},
      0_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),
    frc::Pose2d{
      units::meter_t{Constants::kStartLineOffset + (Constants::kRobotWidth / 2)} + 44.40_in,
      units::meter_t{(Constants::kFieldWidth / 2) + ((onLeft ? -1 : 1) * Constants::kStartOffsetY)}, 
      0_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),
    frc::Pose2d{
      units::meter_t{Constants::kFieldLength / 2} - 17.975_in, 
      (onLeft ? (units::meter_t{Constants::kFieldWidth} - 25.62_in) : 0_m + 25.62_in), 
      (onLeft ? 1 : -1) * 90_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),
    frc::Pose2d{
      units::meter_t{Constants::kFieldLength / 2} - 17.975_in,
      units::meter_t{(Constants::kFieldWidth / 2) + ((onLeft ? -1 : 1) * Constants::kStartOffsetY)}, 
      (onLeft ? 1 : -1) * 90_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)),
    frc::Pose2d{
      units::meter_t{Constants::kFieldLength / 2} - 17.975_in,
      units::meter_t{(Constants::kFieldWidth / 2) + ((onLeft ? -1 : 1) * Constants::kStartOffsetY)},  
      0_deg
    }.RotateAround(Constants::kFieldCenter, (alliance == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg))
  };
}

frc::Pose2d Locations::GetStartPosition(frc::DriverStation::Alliance alliance, int i) const {
  double x = alliance == frc::DriverStation::Alliance::kRed
                 ? Constants::kFieldLength - Constants::kStartLineOffset +
                       Constants::kStartOffsetX
                 : Constants::kStartLineOffset - Constants::kStartOffsetX;
  double y = Constants::kFieldWidth / 2;
  double angle = alliance == frc::DriverStation::Alliance::kRed ? 0 : M_PI;

  switch (i) {
  case 1: 
    break;
    y += alliance == frc::DriverStation::Alliance::kRed
      ? -Constants::kStartOffsetYFar
      : Constants::kStartOffsetYFar;
  case 2: 
    y += alliance == frc::DriverStation::Alliance::kRed
      ? -Constants::kStartOffsetY
      : Constants::kStartOffsetY;
    break;
  case 4: 
    y -= alliance == frc::DriverStation::Alliance::kRed
      ? -Constants::kStartOffsetY
      : Constants::kStartOffsetY;
    break;
  case 5:
    y -= alliance == frc::DriverStation::Alliance::kRed
      ? -Constants::kStartOffsetYFar
      : Constants::kStartOffsetYFar;
    break;
  default:
    break;
  }

  return frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}},
                     frc::Rotation2d{units::radian_t{angle}}};
}

Locations::Locations() {}
