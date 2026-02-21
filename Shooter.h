#pragma once

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"
#include "System.h"

class Shooter : public System {

public:
    static Shooter &GetInstance() {
        static Shooter instance;
        return instance;
    }
    double angleOfTurret;

    void SetAngle(double angle);

    bool IsAtSetpoint();

    void ShootingMotor();

    void ExtendMotor();

    void Update(Robot::Mode mode, double t) override;

private:
    // change back to Turret motor for once done
    rev::spark::SparkMax m_azimuthTurretMotor{Constants::kAzimuthMotorId, rev::spark::SparkMax::MotorType::kBrushless}; // get motor for turret rotation
    rev::spark::SparkClosedLoopController m_azimuthController = m_azimuthTurretMotor.GetClosedLoopController(); // get controller for turret rotation

    rev::spark::SparkMaxConfig m_azimuthConfig; //configured in the cpp file

    double 

    Shooter();
};