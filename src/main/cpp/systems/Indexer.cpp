#include "systems/Indexer.h"
#include "Controllers.h"

using namespace rev::spark;

void Indexer::Update(Robot::Mode mode) {
    if (mode == Robot::Mode::kTeleop) {
        if (Controllers::GetInstance().GetOperatorController().GetRightTriggerAxis() > 0.5) {
            if (IndexerSafe()) {
                StartIndexing();
            } else {
                StopIndexing();
            }
        } else {
            StopIndexing();
        }
    }
}

void Indexer::StartIndexing() {
    //Activates the motors.
    m_IndexerSpinMotor.Set(Constants::kIndexerSpinMotorSpeed);
    m_IndexerUpMotor.Set(Constants::kIndexerUpMotorSpeed);
    //Updates the indexer state.
    m_indexerState = true;
}

void Indexer::StopIndexing() {
    //Stops the motors.
    m_IndexerSpinMotor.Set(0.0);
    m_IndexerUpMotor.Set(0.0);
    //Updates the indexer state.
    m_indexerState = false;
}

bool Indexer::IndexerSafe() const {
    //if (turrent in correct spot) {
    //  if (shooter at target speed)
    //      return true;
    //  }
    //}
    return true;
}