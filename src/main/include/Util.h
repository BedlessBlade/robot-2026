#pragma once

#include <cmath>
#include <frc/geometry/Rotation2d.h>

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
};
