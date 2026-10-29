#pragma once

#include <rev/SparkBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include "Constants.h"
#include "Robot.h"
#include "SMSubsytemBase.h"

// System and desired states
enum IndexerStates {IDLE, RUNNING, MANUAL};
enum DesiredIndexerStates {IDLE, INDEXING};

class Indexer2 : public SMSubSystemBase<IndexerStates, DesiredIndexerStates> {
public:
    // singleton getinstance
    static Indexer2 &GetInstance() {
        static Indexer2 instance;
        return instance;
    }

    // interface functions
    void StartIndexing();
    void StopIndexing();

private:
    //Instantiate indexer motors
    rev::spark::SparkMax m_IndexerSpinMotor{Constants::kIndexerSpinMotorID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax m_IndexerUpMotor{Constants::kIndexerUpMotorID, rev::spark::SparkMax::MotorType::kBrushless};

    // add configuration with current limits

    // Private state machine handlers
    void handleStateTransition() override;
    void applyStates() override;
    void logSubsystem() override;

    // Private singleton constructor 
    Indexer2();
};