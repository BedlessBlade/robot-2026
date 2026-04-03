#include "systems/LEDs.h"
#include "Controllers.h"
#include "Robot.h"

void LEDs::LEDsInit() {
  //Initializes the LEDs.
  m_led.SetLength(Constants::kLEDSlength);
  m_led.SetData(LEDs::GetInstance().m_ledBuffer);
  m_led.Start();
}

void LEDs::SetPattern(LEDs::LEDSstates state){
    State = state;
}

//Main function; causes the LEDs to change in accordance with inputs.
void LEDs::Update(Robot::Mode mode, std::string alliance) {
  if (State > 0 && State < 10) {
    std::vector<frc::LEDPattern>{ 
      m_off, m_red, m_blue, m_redBreathe, m_blueBreathe, m_redYellow, m_blueYellow, m_redYellowScroll, m_blueYellowScroll, m_redYellowReverse, m_blueYellowReverse
    }[State].ApplyTo(m_ledBuffer);
  }
  //Pushes the new color to the LEDs.
  m_led.SetData(m_ledBuffer);
}