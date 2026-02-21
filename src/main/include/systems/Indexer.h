#pragma once

#include <rev/SparkBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include "Constants.h"
#include "System.h"
#include "Robot.h"

class Indexer : public System {
    //Sets the GetInstance() function (very important).
    public: 
        static Indexer &GetInstance(){
            static Indexer instance;
            return instance;
        }

        //Initializes the SparkMax objects.
        rev::spark::SparkMax m_IndexerSpinMotor{Constants::kIndexerSpinMotorID, rev::spark::SparkMax::MotorType::kBrushless};
        rev::spark::SparkMax m_IndexerUpMotor{Constants::kIndexerUpMotorID, rev::spark::SparkMax::MotorType::kBrushless};

        //Defines functions to use in the Indexer.cpp file.
        void Update(Robot::Mode mode);
        void StartIndexing();
        void StopIndexing();
        bool IndexerSafe() const;

        bool m_indexerState = false; //Tracks if the indexer is activated.

};