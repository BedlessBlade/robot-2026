#include "Locations.h"

#include <cmath>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include "Constants.h"

void Locations::Generate(frc::DriverStation::Alliance alliance) {
//   m_algaePositions.clear();
//   m_algaePositions.push_back(frc::Pose2d{frc::Translation2d{yeah}});
}

// const std::vector<frc::Pose2d> Locations::GetAlgaePositions() const {
//   return m_coralPositions;
// }


frc::Pose2d Locations::GetStartPosition(frc::DriverStation::Alliance alliance,
                                        int i) const {
  double x = alliance == frc::DriverStation::Alliance::kRed
                 ? Constants::kFieldLength - Constants::kStartLineOffset +
                       Constants::kStartOffsetX
                 : Constants::kStartLineOffset - Constants::kStartOffsetX;
  double y = Constants::kFieldWidth / 2;
  double angle = alliance == frc::DriverStation::Alliance::kRed ? 0 : M_PI;

  switch (i) {
  case 1:
    y += alliance == frc::DriverStation::Alliance::kRed
             ? -Constants::kStartOffsetY
             : Constants::kStartOffsetY;
    angle += M_PI / 4;
    break;
  case 3:
    y -= alliance == frc::DriverStation::Alliance::kRed
             ? -Constants::kStartOffsetY
             : Constants::kStartOffsetY;
    angle -= M_PI / 4;
    break;
  default:
    break;
  }

  return frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}},
                     frc::Rotation2d{units::radian_t{angle}}};
}

Locations::Locations() {}
