#include "systems/Indexer.h"
#include "Controllers.h"
#include "systems/Shooter.h"

using namespace rev::spark;

void Indexer::Update(Robot::Mode mode) {
    if(m_indexerState == true) {
        m_IndexerSpinMotor.Set(Constants::kIndexerSpinMotorSpeed);
        m_IndexerUpMotor.Set(Constants::kIndexerUpMotorSpeed);
    } else if(m_indexerState == false){
        m_IndexerSpinMotor.Set(0.0);
        m_IndexerUpMotor.Set(0.0);
    }
}

void Indexer::StartIndexing() {
    m_indexerState = true;
}

void Indexer::StopIndexing() {
    m_indexerState = false;
}