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
        if (Controllers::GetInstance().GetOperatorController().GetPOV() == 0) { //D-Pad Up
            Climber::SetClimber(Constants::kClimbExtended, SparkBase::ControlType::kMAXMotionPositionControl);
        } else if (Controllers::GetInstance().GetOperatorController().GetPOV() == 270) { //D-Pad Left
            Climber::SetClimber(Constants::kClimbClimbed, SparkBase::ControlType::kMAXMotionPositionControl);
        } else if (Controllers::GetInstance().GetOperatorController().GetPOV() == 180) { //D-Pad Down
            Climber::SetClimber(Constants::kClimbStowed, SparkBase::ControlType::kMAXMotionPositionControl);
        }
    }
}

void Climber::SetClimber(double position, SparkBase::ControlType controlType) {
    m_climberExtension = position;
    m_climberController.SetSetpoint(position, controlType, ClosedLoopSlot::kSlot0);
}

double Climber::GetClimberPosition() {
    return m_climberEncoder.GetPosition();
}