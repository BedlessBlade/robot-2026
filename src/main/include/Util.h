#pragma once

#include "Constants.h"
#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

class Util {
public:
  // Squares the value while keeping the sign
  static double Exp(double value) {
    if (value >= 0) {
      return std::pow(value, 2);
    }

    return -std::pow(value, 2);
  }

  // Finds the absolute value of the distance between two angles in radians
  static double AngleDistance(frc::Rotation2d a, frc::Rotation2d b) {
    double angleA = a.Radians().value();
    double angleB = b.Radians().value();

    return std::abs(angleA - angleB);
  }

  static frc::Pose2d ReflectX(frc::Pose2d pose) {
    pose.X() = units::meter_t{Constants::kFieldLength} - pose.X();
    return pose;
  }
  static frc::Pose2d ReflectY(frc::Pose2d pose) {
    pose.Y() = units::meter_t{Constants::kFieldWidth} - pose.Y();
    return pose;
  }
};
