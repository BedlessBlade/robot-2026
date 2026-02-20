#include "systems/LEDs.h"
#include "Controllers.h"
#include "Robot.h"

void LEDs::LEDsInit() {
  //Initializes the LEDs.
  LEDs::GetInstance().m_led.SetLength(LEDs::GetInstance().kLength);
  LEDs::GetInstance().m_led.SetData(LEDs::GetInstance().m_ledBuffer);
  LEDs::GetInstance().m_led.Start();
}

//Main function; causes the LEDs to change in accordance with inputs.
void LEDs::LEDCheck(std::string mode, std::string alliance) {
  //Checks if the X Button is pressed down, and causes that to flip a switch.
  if(!xButtonPressed && Controllers::GetInstance().GetDriverController().GetXButtonPressed()) {
      xButtonPressed = true;
    } else if (xButtonPressed && Controllers::GetInstance().GetDriverController().GetXButtonPressed()) {
      xButtonPressed = false;
    }
  if (mode == "Disabled") { //Turns the LEDs off when disabled.
    m_off.ApplyTo(m_ledBuffer);
  } else if (alliance == "Red") { //Sets the LEDs when on the Red Alliance.
    if (mode == "Teleop") { //Sets the LEDs when in TeleOp.
      m_red.ApplyTo(m_ledBuffer);

      if(xButtonPressed && (Controllers::GetInstance().GetDriverController().GetLeftTriggerAxis() > 0.5)) { //Changes LEDs when Intake is activated.
        m_scrollingRedAndYellow.ApplyTo(m_ledBuffer);
      } else if (xButtonPressed) { //Changes LEDs when Intake is deployed.
        m_redAndYellow.ApplyTo(m_ledBuffer);
      }
    } if (mode == "Auto") { //Sets the LEDs when in Auto.
      m_breatheRed.ApplyTo(m_ledBuffer);
    }
  } else if (alliance == "Blue") { //Sets the LEDs when on the Blue Alliance.
    if (mode == "Teleop") { //Sets the LEDs when in TeleOp.
      m_blue.ApplyTo(m_ledBuffer);

      if(xButtonPressed && (Controllers::GetInstance().GetDriverController().GetLeftTriggerAxis() > 0.5)) { //Changes LEDs when Intake is activated.
        m_scrollingBlueAndYellow.ApplyTo(m_ledBuffer);
      } else if (xButtonPressed) { //Changes LEDs when Intake is deployed.
        m_blueAndYellow.ApplyTo(m_ledBuffer);
      }
    }
    if (mode == "Auto") { //Sets the LEDs when in Auto.
      m_breatheBlue.ApplyTo(m_ledBuffer);
    }
  }
  //Pushes the new color to the LEDs.
  m_led.SetData(m_ledBuffer);
}