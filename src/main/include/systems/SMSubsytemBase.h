#pragma once

#include "Robot.h"

template <typename SystemStates, typename DesiredStates> class SMSubSystemBase {
public:
    // setDesiredState: Sets m_desiredState to be evaluated by the state transition handler
    void setDesiredState(DesiredStates state) {
        m_desiredState = state;
    }

    // getSystemState: returns current system state
    SystemStates getSystemState() {
        return m_currentState;
    }

    // compareSystemState: compares argument to current system state
    bool compareSystemState(SystemStates state) {
        return m_currentState == state;
    }

    // update: called periodically by the robot code, calls state handlers, logs current state, updates m_lastState
    void update() {
        handleStateTransition();
        applyStates();

        // Log values
        logSubsystem();

        // Store Last subsystem state
        m_lastState = m_currentState;
    }

protected:
    // Internal state variables
    bool m_firstStep;
    SystemStates m_currentState;
    SystemStates m_lastState;
    DesiredStates m_desiredState;
    
    // handleStateTransition: evaluates desired state triggers => evaluates state transition guard conditions => queues system state
    // applyStates: evaluates entry and while actions, sets subsystem outputs
    virtual void handleStateTransition();
    virtual void applyStates();
    virtual void logSubsystem();

    // queueSystemState: queues system state, sets m_firstStep true if not equal to current state
    void queueSystemState(SystemStates nextState) {
        // Update currentState to nextState and set firstStep to true if currentState is null or not equal to nextState
        if (m_currentState == NULL || !(m_currentState == nextState)) {
            m_currentState = nextState;
            m_firstStep = true;

        // Set firstStep to false
        } else {
            m_firstStep = false;
        }
    }

    // stateInit: resets m_firstStep
    bool stateReInit() {
        // Set firstStep false and return true if firstStep is true
        if (m_firstStep) {
            m_firstStep = false;
            return true
        
        // return false
        } else {
            return false
        }
    }
};