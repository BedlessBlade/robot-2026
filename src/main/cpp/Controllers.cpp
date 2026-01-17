#include "Controllers.h"

#include "Constants.h"

Controllers::Controllers()
    : m_driverController{Constants::kDriverControllerId},
      m_operatorController{Constants::kOperatorControllerId} {}

frc::XboxController &Controllers::GetDriverController() {
  return m_driverController;
}

frc::XboxController &Controllers::GetOperatorController() {
  return m_operatorController;
}
