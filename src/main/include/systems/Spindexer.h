#pragma once

#include <rev/SparkMax.h>

#include "Constants.h"

class Spindexer {

public:
    static Spindexer &GetInstance() {
        static Spindexer instance;
        return instance;
    }

    void StartSpin();

    void StopSpin();

private:

    rev::spark::SparkMax m_spinMotor{Constants::kSpindexerMotorId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax m_kickMotor{Constants::kKickerMotorId, rev::spark::SparkMax::MotorType::kBrushless};

    Spindexer();
};