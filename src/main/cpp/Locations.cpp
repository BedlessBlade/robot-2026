#include "Locations.h"

#include <cmath>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include "Constants.h"
#include "Util.h"

void Locations::Generate(frc::DriverStation::Alliance alliance) {
//   m_algaePositions.clear();
//   m_algaePositions.push_back(frc::Pose2d{frc::Translation2d{yeah}});
}

// const std::vector<frc::Pose2d> Locations::GetAlgaePositions() const {
//   return m_coralPositions;
// }


frc::Pose2d Locations::GetOutpostPosition(frc::DriverStation::Alliance alliance) const {
  if (alliance == frc::DriverStation::Alliance::kBlue) {
    return frc::Pose2d{
      units::meter_t{20 / Constants::kInchesPerMeter}, 
      units::meter_t{25.62 / Constants::kInchesPerMeter},
      0_deg
    };
  } else {
    return frc::Pose2d{
      units::meter_t{Constants::kFieldLength - (20 / Constants::kInchesPerMeter)}, 
      units::meter_t{Constants::kFieldWidth - (25.62 / Constants::kInchesPerMeter)},
      0_deg
    };
  }
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

frc::Pose2d Locations::GetStartPosition(frc::DriverStation::Alliance alliance,
                                        int i) const {
  double x = alliance == frc::DriverStation::Alliance::kRed
                 ? Constants::kFieldLength - Constants::kStartLineOffset +
                       Constants::kStartOffsetX
                 : Constants::kStartLineOffset - Constants::kStartOffsetX;
  double y = Constants::kFieldWidth / 2;
  double angle = alliance == frc::DriverStation::Alliance::kRed ? 0 : M_PI;

  switch (i) {
  case 1: //should be 2
    break;
  case 2: //should be 2
    y += alliance == frc::DriverStation::Alliance::kRed
             ? -Constants::kStartOffsetY
             : Constants::kStartOffsetY;
    // angle += M_PI ;
    break;
  case 4: // should be 4
    y -= alliance == frc::DriverStation::Alliance::kRed
             ? -Constants::kStartOffsetY
             : Constants::kStartOffsetY;
    // angle -= M_PI / 4;
    break;
  case 5:
    break;
  default:
    break;
  }

  return frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}},
                     frc::Rotation2d{units::radian_t{angle}}};
}

Locations::Locations() {}
