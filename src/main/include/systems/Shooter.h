#pragma once

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

class Shooter {

public:
    static Shooter &GetInstance() {
        static Shooter instance;
        return instance;
    }

    void SetAngle(double angle);

    double GetAngle();

private:

    rev::spark::SparkMax m_azimuthMotor{Constants::kAzimuthMotorId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController m_azimuthController = m_azimuthMotor.GetClosedLoopController();

    rev::spark::SparkMaxConfig m_azimuthConfig;

    Shooter();
};