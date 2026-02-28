#include <pathplanner/lib/commands/PathPlannerAuto.h>

using namespace pathplanner;

frc2::CommandPtr PathPlannerAuto::getAutonomousCommand(){
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return PathPlannerAuto("MoveAndClimb").ToPtr();
};