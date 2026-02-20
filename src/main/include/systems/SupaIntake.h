//  -- SupaIntake.h by patib --
// pre-defines functions and the locations of the motors and pistons for SupaIntake.cpp
// last update: 2/19/2026

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DoubleSolenoid.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <rev/SparkMax.h>

#include "Robot.h"
#include "Constants.h"
#include "systems/System.h"

using namespace ctre::phoenix6;

class SupaIntake : public System {
public:
    //Static Instance
    static SupaIntake &GetInstance() {
        static SupaIntake instance;
        return instance;
    }

    //run when stopping robot
    void exitSystem();

    //Motors (apply to both)
    void SpinIn();      //Motor in, out, & idle
    void SpinOut();
    void SpinStop();

    //Pistons (apply to both)
    void Extend();      //Pistons Up & Down
    void Retract();

    //Mainly used by robot.cpp
    void Update(Robot::Mode mode, double t);
    bool ArmDown();    // get if the arm is down
    bool intakeSafe(); // sees if motor is completly down

    //spin state enum
    enum class spinCur { IN, OUT, IDLE }; // What state is the motor in?

    //Piston state enums 
    enum class pistLoc { UP, DOWN }; // What state is the piston in?

    //get the motor state
    SupaIntake::spinCur GetMotorCase(); // tell Robot.cpp what state the motor is in

private:
    
    // add 2 motors
    rev::spark::SparkMax m_sparkMotor{Constants::kIntakeMotorId, rev::spark::SparkMax::MotorType::kBrushless};

    // add 2 pistons
    frc::DoubleSolenoid m_solenoid{frc::PneumaticsModuleType::REVPH, Constants::kp1IntakePneumId, -1};
    //frc::DoubleSolenoid m_solenoid2{frc::PneumaticsModuleType::REVPH, Constants::kp2IntakePneumId, -1};
    //May need the second solenoid? PATRICK FIX

    //Make states usable in .cpp file
    spinCur motorState = spinCur::IDLE;
    pistLoc pnuemState = pistLoc::UP; 

    SupaIntake();
};