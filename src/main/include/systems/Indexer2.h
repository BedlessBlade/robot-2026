#pragma once

#include <rev/SparkBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include "Constants.h"
#include "Robot.h"
#include "SMSubsytemBase.h"

enum IndexerStates {IDLE, INDEXING, MANUAL};

class Indexer2 : public SMSubSystemBase<IndexerStates> {
public:

private:


};

// class Indexer : public SMSubsystemBase {       

// };

//Initializes the SparkMax objects.
//rev::spark::SparkMax m_IndexerSpinMotor{Constants::kIndexerSpinMotorID, rev::spark::SparkMax::MotorType::kBrushless};
//rev::spark::SparkMax m_IndexerUpMotor{Constants::kIndexerUpMotorID, rev::spark::SparkMax::MotorType::kBrushless};

//void StartIndexing();
//void StopIndexing();