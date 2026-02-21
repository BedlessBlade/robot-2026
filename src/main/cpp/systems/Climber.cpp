#include "systems/Climber.h"
using namespace rev::spark;

void Climber::SetClimber(double position) {
    m_climberController.SetSetpoint(position, SparkBase::ControlType::kPosition);
    //Work In Progress, 
}

double Climber::GetClimber() {
    return m_climberController.GetSetpoint();
}

double Climber::GetClimberPosition() {
    return m_climberController.GetMAXMotionSetpointPosition();
}