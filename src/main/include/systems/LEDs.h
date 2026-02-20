#pragma once

#include "Robot.h"
#include "System.h"
#include <networktables/NetworkTableInstance.h>
#include <frc/TimedRobot.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc/DriverStation.h>
#include <units/time.h>

class LEDs : public frc::TimedRobot {
  //Sets the GetInstance() function (very important).
  public:
    static LEDs &GetInstance() {
      static LEDs instance;
      return instance;
    }
    //Initializes the LEDs object.
    static constexpr int kLength = 300;
    frc::AddressableLED m_led{9};
    std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
    units::meter_t kLedSpacing{1 / 60.0};

    //Defines static patterns.
    frc::LEDPattern m_off = frc::LEDPattern::Off();
    frc::LEDPattern m_pink = frc::LEDPattern::Solid(frc::Color::kWhite);
    frc::LEDPattern m_red = frc::LEDPattern::Solid(frc::Color::kRed);
    frc::LEDPattern m_blue = frc::LEDPattern::Solid(frc::Color::kBlue);
    frc::LEDPattern m_breatheRed = m_red.Breathe(3_s);
    frc::LEDPattern m_breatheBlue = m_blue.Breathe(3_s);
    frc::LEDPattern m_orange = frc::LEDPattern::Solid(frc::Color::kOrange);
    //Defines multicolor patterns.
    frc::LEDPattern m_redAndYellow = frc::LEDPattern::Steps({
        {0, frc::Color::kRed},
        {0.02, frc::Color::kOrange},
        {0.04, frc::Color::kRed},
        {0.06, frc::Color::kOrange},
        {0.08, frc::Color::kRed},
        {0.1, frc::Color::kOrange},
        {0.12, frc::Color::kRed},
        {0.14, frc::Color::kOrange},
        {0.16, frc::Color::kRed},
        {0.18, frc::Color::kOrange},
        {0.2, frc::Color::kRed},
        {0.22, frc::Color::kOrange},
        {0.24, frc::Color::kRed},
        {0.26, frc::Color::kOrange},
        {0.28, frc::Color::kRed},
        {0.3, frc::Color::kOrange},
        {0.32, frc::Color::kRed},
        {0.34, frc::Color::kOrange},
        {0.36, frc::Color::kRed},
        {0.38, frc::Color::kOrange},
        {0.4, frc::Color::kRed},
        {0.42, frc::Color::kOrange},
        {0.44, frc::Color::kRed},
        {0.46, frc::Color::kOrange},
        {0.48, frc::Color::kRed},
        {0.5, frc::Color::kOrange},
        {0.52, frc::Color::kRed},
        {0.54, frc::Color::kOrange},
        {0.56, frc::Color::kRed},
        {0.58, frc::Color::kOrange},
        {0.6, frc::Color::kRed},
        {0.62, frc::Color::kOrange},
        {0.64, frc::Color::kRed},
        {0.66, frc::Color::kOrange},
        {0.68, frc::Color::kRed},
        {0.7, frc::Color::kOrange},
        {0.72, frc::Color::kRed},
        {0.74, frc::Color::kOrange},
        {0.76, frc::Color::kRed},
        {0.78, frc::Color::kOrange},
        {0.8, frc::Color::kRed},
        {0.82, frc::Color::kOrange},
        {0.84, frc::Color::kRed},
        {0.86, frc::Color::kOrange},
        {0.88, frc::Color::kRed},
        {0.9, frc::Color::kOrange},
        {0.92, frc::Color::kRed},
        {0.94, frc::Color::kOrange},
        {0.96, frc::Color::kRed},
        {0.98, frc::Color::kOrange}
    });
    frc::LEDPattern m_scrollingRedAndYellow = m_redAndYellow.ScrollAtRelativeSpeed(0.15_Hz);
    frc::LEDPattern m_blueAndYellow = frc::LEDPattern::Steps({
        {0, frc::Color::kBlue},
        {0.02, frc::Color::kOrange},
        {0.04, frc::Color::kBlue},
        {0.06, frc::Color::kOrange},
        {0.08, frc::Color::kBlue},
        {0.1, frc::Color::kOrange},
        {0.12, frc::Color::kBlue},
        {0.14, frc::Color::kOrange},
        {0.16, frc::Color::kBlue},
        {0.18, frc::Color::kOrange},
        {0.2, frc::Color::kBlue},
        {0.22, frc::Color::kOrange},
        {0.24, frc::Color::kBlue},
        {0.26, frc::Color::kOrange},
        {0.28, frc::Color::kBlue},
        {0.3, frc::Color::kOrange},
        {0.32, frc::Color::kBlue},
        {0.34, frc::Color::kOrange},
        {0.36, frc::Color::kBlue},
        {0.38, frc::Color::kOrange},
        {0.4, frc::Color::kBlue},
        {0.42, frc::Color::kOrange},
        {0.44, frc::Color::kBlue},
        {0.46, frc::Color::kOrange},
        {0.48, frc::Color::kBlue},
        {0.5, frc::Color::kOrange},
        {0.52, frc::Color::kBlue},
        {0.54, frc::Color::kOrange},
        {0.56, frc::Color::kBlue},
        {0.58, frc::Color::kOrange},
        {0.6, frc::Color::kBlue},
        {0.62, frc::Color::kOrange},
        {0.64, frc::Color::kBlue},
        {0.66, frc::Color::kOrange},
        {0.68, frc::Color::kBlue},
        {0.7, frc::Color::kOrange},
        {0.72, frc::Color::kBlue},
        {0.74, frc::Color :: kOrange },
        {0.76 ,  frc :: Color :: kBlue },
        {0.78 ,  frc :: Color :: kOrange },
      	{0.8,  frc::Color ::kBlue},
      	{0.82,  frc::Color::kOrange},
      	{0.84,  frc::Color::kBlue},
      	{0.86,  frc::Color::kOrange},
      	{0.88,  frc::Color::kBlue},
      	{0.9,  frc::Color::kOrange},
      	{0.92,  frc::Color::kBlue},
      	{0.94,  frc::Color::kOrange},
      	{0.96,  frc::Color::kBlue},
      	{0.98,  frc::Color::kOrange}  
    });
    frc::LEDPattern m_scrollingBlueAndYellow = m_blueAndYellow.ScrollAtRelativeSpeed(0.15_Hz);
    //Defines cool rainbow.
    frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255, 128);
    frc::LEDPattern m_scrollingRainbow = m_rainbow.ScrollAtAbsoluteSpeed(1_mps, kLedSpacing);
    
    
    //Creates functions defined in LEDs.cpp
    void LEDsInit();
    void LEDCheck(std::string mode, std::string alliance);
    void updateXButtonToggle();
    bool xButtonPressed = false;
};