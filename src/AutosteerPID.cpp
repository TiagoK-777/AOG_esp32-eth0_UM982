/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

#include <Arduino.h>
#include "GlobalVariables.h"

void calcSteeringPID(void)
{
    //Proportional only
    pValue = steerSettings.Kp * steerAngleError;
    pwmDrive = (int16_t)pValue;

    errorAbs = abs(steerAngleError);
    int16_t newMax = 0;

    if (errorAbs < LOW_HIGH_DEGREES)
    {
        newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
    }
    else newMax = steerSettings.highPWM;

    //add min throttle factor so no delay from motor resistance.
    if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
    else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;

    //Serial.print(newMax); //The actual steering angle in degrees
    //Serial.print(",");

    //limit the pwm drive
    if (pwmDrive > newMax) pwmDrive = newMax;
    if (pwmDrive < -newMax) pwmDrive = -newMax;

    if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

    if (steerConfig.IsDanfoss)
    {
        // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
        // Danfoss: PWM 50% On = Center Position
        // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
        pwmDrive = (constrain(pwmDrive, -250, 250));

        // Calculations below make sure pwmDrive values are between 65 and 190
        // This means they are always positive, so in motorDrive, no need to check for
        // steerConfig.isDanfoss anymore
        pwmDrive = pwmDrive >> 2; // Devide by 4
        pwmDrive += 128;          // add Center Pos.

        // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
        // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
        // Voh=11.08 volts, Vol=0.185v
        // (11.08/12)*255=235.45
        // (0.185/12)*255=3.93
        // output now lies in the range [67 ... 205], the center position is now 136
        //pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
    }
}

//#########################################################################################

void motorDrive(void)
{
  // Used with Cytron MD30C Driver
  // Steering Motor
  // Dir + PWM Signal
  if (steerConfig.CytronDriver)
  {
    // Cytron MD30C Driver Dir + PWM Signal
    if (pwmDrive >= 0)
    {
      digitalWrite(DIR1_RL_ENABLE, HIGH);  //set the correct direction
    }
    else
    {
      digitalWrite(DIR1_RL_ENABLE, LOW);
      pwmDrive = -1 * pwmDrive;
    }

    //write out the 0 to 255 value using LEDC for ESP32
    ledcWrite(0, pwmDrive);  // Channel 0 = PWM1_LPWM
    pwmDisplay = pwmDrive;
  }
  else
  {
    // IBT 2 Driver Dir1 connected to BOTH enables
    // PWM Left + PWM Right Signal

    if (pwmDrive > 0)
    {
      ledcWrite(1, 0);//Turn off before other one on - Channel 1 = PWM2_RPWM
      ledcWrite(0, pwmDrive); // Channel 0 = PWM1_LPWM
    }
    else
    {
      pwmDrive = -1 * pwmDrive;
      ledcWrite(0, 0);//Turn off before other one on
      ledcWrite(1, pwmDrive); // Channel 1 = PWM2_RPWM
    }

    pwmDisplay = pwmDrive;
  }
}
