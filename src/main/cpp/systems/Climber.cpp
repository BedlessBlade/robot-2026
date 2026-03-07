#include "systems/Climber.h"
#include "Controllers.h"

using namespace rev::spark;

Climber::Climber() {
    m_climberConfig.closedLoop
    .P(Constants::kClimberP)
    .I(Constants::kClimberI)
    .D(Constants::kClimberD)
    .OutputRange(Constants::kMinClimberOutput, Constants::kMaxClimberOutput);

    m_climberConfig.closedLoop.feedForward
    .kS(Constants::kClimberS)
    .kV(Constants::kClimberV)
    .kA(Constants::kClimberA)
    .kG(Constants::kClimberG)
    .kCos(Constants::kClimberCos)
    .kCosRatio(Constants::kClimberCosRatio);

    m_climberConfig.closedLoop.maxMotion
    .CruiseVelocity(Constants::kClimberCruiseVel)
    .MaxAcceleration(Constants::kClimberMaxAccel)
    .AllowedProfileError(Constants::kClimberAllowedErr);
}

void Climber::Update(Robot::Mode mode) {
    if (mode == Robot::Mode::kTeleop) {
        if (m_climberlevel == climberlevel::EXTENDED) { //D-Pad Up
            m_climberController.SetSetpoint(Constants::kClimbExtended, SparkBase::ControlType::kMAXMotionPositionControl, ClosedLoopSlot::kSlot0);
        } else if (m_climberlevel == climberlevel::CLIMBED) { //D-Pad Left
            m_climberController.SetSetpoint(Constants::kClimbClimbed, SparkBase::ControlType::kMAXMotionPositionControl, ClosedLoopSlot::kSlot0);
        } else if (m_climberlevel == climberlevel::STOWED) { //D-Pad Down
            m_climberController.SetSetpoint(Constants::kClimbStowed, SparkBase::ControlType::kMAXMotionPositionControl, ClosedLoopSlot::kSlot0);
        }
    }
}

void Climber::SetClimber(double level) {
    if(level == 0) {
        m_climberlevel = climberlevel::STOWED;
    } else if(level == 1) {
        m_climberlevel = climberlevel::CLIMBED;
    } else if(level == 2) {
        m_climberlevel = climberlevel::EXTENDED;
    }
}

double Climber::GetClimberPosition() {
    return m_climberEncoder.GetPosition();
}