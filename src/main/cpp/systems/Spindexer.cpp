#include "systems/Spindexer.h"
#include "Constants.h"



void Spindexer::StartSpin() {
    m_spinMotor.Set(Constants::kSpinSpeed);
    IsSpinning = true;
}

void Spindexer::StopSpin() {
    m_spinMotor.Set(0.0);
    IsSpinning = false;
}

void Spindexer::StartKick() {
    m_kickMotor.Set(Constants::kKickSpeed);
    IsKicking = true;
}

void Spindexer::StopKick() {
    m_kickMotor.Set(0.0);
    IsKicking = false;
}