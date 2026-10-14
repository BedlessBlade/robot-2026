#pragma once

#include "Robot.h"

//#include "ctre/phoenix6/SignalLogger.hpp"

//using namespace ctre;

template <typename States> class SMSubSystemBase {
public:
    // singleton getinstance
    static SMSubSystemBase &GetInstance() {
        static SMSubSystemBase instance;
        return instance;
    }

    // Subsystem state getter
    States getState() {
        return m_currentState;
    }

    // Compare subsytem currentState to state
    bool isState(States state) {
        return m_currentState == state;
    }

    // Queue subsytem state
    void queueState(States nextState) {
        // Update currentState to nextState and set firstStep to true if currentState is null or not equal to nextState
        if (m_currentState == NULL || !(m_currentState == nextState)) {
            m_currentState = nextState;
            m_firstStep = true;

        // Set firstStep to false
        } else {
            m_firstStep = false;
        }
    }

    // Initalize subsystem state
    bool stateInit() {
        // Set firstStep false and return true if firstStep is true
        if (m_firstStep) {
            m_firstStep = false;
            return true
        
        // return false
        } else {
            return false
        }
    }

    // Update function
    void Update() {
        inputPeriodic();
        handleStateMachine();
        outputPeriodic();

        // Log current state
        //phoenix6::SignalLogger::Write(m_subsystemName + "/currState", m_currentState);
        //phoenix6::SignalLogger::WriteBoolean(m_subsystemName + "/currState", m_currentState);
    }

private:
    // Internal state variables
    std::string m_subsystemName;
    bool m_firstStep;
    States m_currentState;

    // Periodic functions and state handler
    virtual void inputPeriodic();
    virtual void handleStateMachine();
    virtual void outputPeriodic();

    // Private singleton constructor
    SMSubSystemBase() : m_firstStep = true;
};