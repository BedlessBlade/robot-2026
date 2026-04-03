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
  //if else hell
  if(State == LEDSstates::OFF){
    m_off.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::BLUE){
    m_blue.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::BREATHBLUE){
    m_breatheBlue.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::BLUEYELLOW){
    m_blueAndYellow.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::BLUEYELLOWSCROLL){
    m_scrollingBlueAndYellow.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::BLUEYELLOWREVERSE){
    m_scrollingBlueAndYellowReverse.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::RED){
    m_red.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::BREATHERED){
    m_breatheRed.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::REDYELLOW){
    m_redAndYellow.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::REDYELLOWSCROLL){
    m_scrollingRedAndYellow.ApplyTo(m_ledBuffer);
  } else if(State == LEDSstates::REDYELLOWREVERSE){
    m_scrollingRedAndYellowReverse.ApplyTo(m_ledBuffer);
  }
  //Pushes the new color to the LEDs.
  m_led.SetData(m_ledBuffer);
} 


void LEDsSetPattern(LEDs::LEDSstates state) {
  LEDs::GetInstance().State = state;
}




 