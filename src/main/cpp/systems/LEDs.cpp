#include "systems/LEDs.h"
#include "Controllers.h"
#include "Robot.h"

void LEDs::LEDsInit() {
  //Initializes the LEDs.
  LEDs::GetInstance().m_led.SetLength(Constants::kLEDSlength);
  LEDs::GetInstance().m_led.SetData(LEDs::GetInstance().m_ledBuffer);
  LEDs::GetInstance().m_led.Start();
}

//Main function; causes the LEDs to change in accordance with inputs.
void LEDs::Update(Robot::Mode mode, std::string alliance) {
  //Checks if the  D-Pad buttons are pressed down, and causes that to flip a switch.
  if(!intaking && Controllers::GetInstance().GetOperatorController().GetPOV() == 0) {
      intaking = true;
    } else if (intaking && Controllers::GetInstance().GetOperatorController().GetPOV() == 180) {
      intaking = false;
    }
  if (mode == Robot::Mode::kDisabled) { //Turns the LEDs off when disabled.
    m_off.ApplyTo(m_ledBuffer);
  } else if(mode == Robot::Mode::kAuto){ //Makes lights breathe when in auto
    if (alliance == "Red"){
      m_breatheRed.ApplyTo(m_ledBuffer);
    } else if (alliance == "Blue"){
      m_breatheBlue.ApplyTo(m_ledBuffer);
    }
  } else if(mode == Robot::Mode::kTeleop){
    if (alliance == "Red"){//sets base alliance color
      m_red.ApplyTo(m_ledBuffer);
    } else if (alliance == "Blue"){
      m_blue.ApplyTo(m_ledBuffer);
    }
    if(intaking && (Controllers::GetInstance().GetOperatorController().GetLeftTriggerAxis() > 0.5)) { //Changes LEDs when Intake is activated.
      if (alliance == "Red"){//sets forward intaking colors
        m_scrollingRedAndYellow.ApplyTo(m_ledBuffer);
      } else if (alliance == "Blue"){
        m_scrollingBlueAndYellow.ApplyTo(m_ledBuffer);
      }
    } else if(intaking && (Controllers::GetInstance().GetOperatorController().GetLeftBumperButtonPressed())){ // changes LEDs when intake is reversed
      if (alliance == "Red"){//sets reverse intaking colors
        m_scrollingRedAndYellowReverse.ApplyTo(m_ledBuffer);
      } else if (alliance == "Blue"){
        m_scrollingBlueAndYellowReverse.ApplyTo(m_ledBuffer);
      }
    } else if (intaking) { //Changes LEDs when Intake is deployed.
      if (alliance == "Red"){//sets intake deployed colors
        m_redAndYellow.ApplyTo(m_ledBuffer);
      } else if (alliance == "Blue"){
        m_blueAndYellow.ApplyTo(m_ledBuffer);
      }
    }
  } 

  //Pushes the new color to the LEDs.
  m_led.SetData(m_ledBuffer);

}