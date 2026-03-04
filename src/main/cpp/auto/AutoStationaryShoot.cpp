#include "auto/AutoStationaryShoot.h"


AutoStationaryShoot::AutoStationaryShoot() {
/*
  this is test pseudo code for autos, thinking is using drive velocity instead of using setpoints just for this one
  we could time the robot movement and then have the turret align with the shoot command
    m.tasks.push_back(std::make_shared<DriveVelocity(1.0,0.0,0.0));
    m.tasks.push_back(std::make_shared<Wait>(1.0));
    m.tasks.push_back(std::make_shared<DriveVelocity());
    m.tasks.push_back(std::make_shared<shoot>));
*/
}
