#pragma once

#include "Robot.h"

//#include "ctre/phoenix6/SignalLogger.hpp"

//using namespace ctre;

template <enum States> class SMSubSystemBase {
public:
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
            m_firstSetp = true;

        // Set firstStep to false
        } else {
            m_firstSetp = false;
        }
    }

    // Initalize subsystem state
    bool stateInit() {
        // Set firstStep false and return true if firstStep is true
        if (m_firstSetp) {
            m_firstSetp = false;
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
    std::string m_subsystemName;
    bool m_firstSetp;
    States m_states;
    States m_currentState;

    virtual void inputPeriodic();
    virtual void handleStateMachine();
    virtual void outputPeriodic();
};