#include "systems/Indexer2.h"

Indexer2::Indexer2() {
    // Default constructor, add current limits and other one time configs here
}

void Indexer2::handleStateTransition() {
    switch (m_desiredState) {
        case DesiredIndexerStates::IDLE:
            queueSystemState(IndexerStates::IDLE);
            break;
        
        case DesiredIndexerStates::INDEXING:
            queueSystemState(IndexerStates::INDEXING);
            break;

        default:
            break;
    }
}

void Indexer2::applyStates() {
    switch (m_currentState) {
        case IndexerStates::IDLE:
            m_IndexerSpinMotor.Set(0.0);
            m_IndexerUpMotor.Set(0.0);
            break;
        
        case IndexerStates::INDEXING:
            m_IndexerSpinMotor.Set(Constants::kIndexerSpinMotorSpeed);
            m_IndexerUpMotor.Set(Constants::kIndexerUpMotorSpeed);
            break;

        default:
            break;
    }
}

void Indexer2::StartIndexing() {
    setDesiredState(DesiredIndexerStates::INDEXING);
}

void Indexer2::StopIndexing() {
    setDesiredState(DesiredIndexerStates::IDLE);
}