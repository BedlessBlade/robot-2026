#include "systems/LEDs.h"
#include "Controllers.h"
#include "Robot.h"

LEDs::LEDs() {
  //Initializes the LEDs.
  m_off.ApplyTo(m_ledBuffer);
  
  m_led.SetLength(Constants::kLEDSlength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
}

void LEDs::SetPattern(LEDs::LEDstates state){
    m_state = state;
}

//Main function; causes the LEDs to change in accordance with inputs.
void LEDs::Update(Robot::Mode mode, std::string alliance) {
  if (m_state > 0 && m_state < 10) {
    std::vector<frc::LEDPattern>{ 
      m_off, m_red, m_blue, m_redBreathe, m_blueBreathe, m_redYellow, m_blueYellow, m_redYellowScroll, m_blueYellowScroll, m_redYellowReverse, m_blueYellowReverse
    }[m_state].ApplyTo(m_ledBuffer);
  }
  //Pushes the new color to the LEDs.
  m_led.SetData(m_ledBuffer);
}