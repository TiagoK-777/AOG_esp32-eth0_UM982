/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

#include <Arduino.h>
#include "GlobalVariables.h"

////////////////// User Settings /////////////////////////

// All settings moved to GlobalVariables.h

/////////////////////////////////////////////

// Motor drive connections and sensor pins moved to GlobalVariables.h
//Connect ground only for cytron, Connect Ground and +5v for IBT2

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

#include <IPAddress.h>
#include "BNO08x_AOG.h"
#include <ETH.h>
#include <WiFiUdp.h>

// Buffer For Receiving UDP Data
uint8_t autoSteerUdpData[1460];  // Buffer For Receiving UDP Data (ESP32 max UDP)

//loop time variables in microseconds
// All timing constants moved to GlobalVariables.h
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

// Watchdog timer
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = {0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

uint32_t gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings - moved struct definitions to GlobalVariables.h
// Storage struct and Setup struct are defined in GlobalVariables.h
// steerSettings and steerConfig are defined in AOG_Esp32_UM982.cpp

void steerConfigInit()
{
  if (steerConfig.CytronDriver) 
  {
    pinMode(PWM2_RPWM, OUTPUT);
  }
}

void steerSettingsInit()
{
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void autosteerSetup()
{
  // Set ADC resolution to 12-bit (0-4095) for ESP32
  analogReadResolution(12);

  //PWM rate settings for ESP32 LEDC
  // PWM Frequency: 0=490hz (default), 1=122hz, 2=3921hz
  
  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT);
  pinMode(STEERSW_PIN, INPUT);
  pinMode(REMOTE_PIN, INPUT);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  // Setup PWM for ESP32 LEDC
  // Configure LEDC channels for PWM
  ledcSetup(0, 490, 8);  // Channel 0, 490Hz, 8-bit resolution (default)
  ledcSetup(1, 490, 8);  // Channel 1, 490Hz, 8-bit resolution
  
  if (PWM_Frequency == 1)
  {
    ledcSetup(0, 122, 8);
    ledcSetup(1, 122, 8);
  }
  else if (PWM_Frequency == 2)
  {
    ledcSetup(0, 3921, 8);
    ledcSetup(1, 3921, 8);
  }
  
  // Attach pins to LEDC channels
  ledcAttachPin(PWM1_LPWM, 0);
  ledcAttachPin(PWM2_RPWM, 1);
  
  // Ensure motors are off at startup
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(DIR1_RL_ENABLE, LOW);

  //set up communication - ESP32 I2C
  Wire.end();
  Wire.begin(33, 32); // SDA=GPIO33, SCL=GPIO32 para WT32-ETH01
    
  // Check ADC 
  if(adc.testConnection())
  {
    Serial.println("ADC Connecton OK");
  }
  else
  {
    Serial.println("ADC Connecton FAILED!");
    Autosteer_running = false;
  }

  //50Khz I2C
  //TWBR = 144;   //Is this needed?

  EEPROM.begin(512);                  // Initialize EEPROM for ESP32
  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)            // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, networkAddress);
    EEPROM.commit();                  // Commit changes to flash
  }
  else
  {
    EEPROM.get(10, steerSettings);     // read the Settings
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, networkAddress); 
  }

  steerSettingsInit();
  steerConfigInit();

  if (Autosteer_running) 
  {
    Serial.println("Autosteer running, waiting for AgOpenGPS");
    // Autosteer Led goes Red if ADS1115 is found
    //digitalWrite(AUTOSTEER_ACTIVE_LED, 0);   //Commented to save ESP32 pins
    //digitalWrite(AUTOSTEER_STANDBY_LED, 1);  //Commented to save ESP32 pins
  }
  else
  {
    Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
//    if(!Ethernet_running)Serial.println("Ethernet not available");
    Serial.println("Autosteer disabled, GPS only mode");   
    return;
  }

  adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
  adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

}// End of Setup

void autosteerLoop()
{
  ReceiveUdp();
  //Serial.println("AutoSteer loop");

  // Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = millis();

  if (currentTime - autsteerLastTime >= LOOP_TIME)
  {
    autsteerLastTime = currentTime;

    //reset debounce
    encEnable = true;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    //read all the switches
    workSwitch = digitalRead(WORKSW_PIN);  // read work switch

    if (steerConfig.SteerSwitch == 1)         //steer switch on - off
    {
      steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
    }
    else if (steerConfig.SteerButton == 1)    //steer Button momentary
    {
      reading = digitalRead(STEERSW_PIN);
      if (reading == LOW && previous == HIGH)
      {
        if (currentState == 1)
        {
          currentState = 0;
          steerSwitch = 0;
        }
        else
        {
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
    }
    else                                      // No steer switch and no steer button
    {
      // So set the correct value. When guidanceStatus = 1,
      // it should be on because the button is pressed in the GUI
      // But the guidancestatus should have set it off first
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
      {
        steerSwitch = 0;
        previous = 1;
      }

      // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
      if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
      {
        steerSwitch = 1;
        previous = 0;
      }
    }

    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }

    // Pressure sensor - REMOVED: Not used, only current sensor
    /*
    if (steerConfig.PressureSensor)
    {
      sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
      sensorSample *= 0.25;
      sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
      if (sensorReading >= steerConfig.PulseCountMax)
      {
          steerSwitch = 1; // reset values like it turned off
          currentState = 1;
          previous = 0;
      }
    }
    */

    // Current sensor?
    if (steerConfig.CurrentSensor)
    {
      sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);
      // Adjusted for ESP32 12-bit ADC (0-4095) to AOG scale (0-255)
      sensorSample = sensorSample * 0.06227f;
      sensorReading = sensorReading * 0.7 + sensorSample * 0.3;    
      sensorReading = min(sensorReading, 255.0f);

      if (sensorReading >= steerConfig.PulseCountMax)
      {
          steerSwitch = 1; // reset values like it turned off
          currentState = 1;
          previous = 0;
      }
    }

    remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    /*
      #if Relay_Type == 1
        SetRelays();       //turn on off section relays
      #elif Relay_Type == 2
        SetuTurnRelays();  //turn on off uTurn relays
      #endif
    */

    //get steering position
    if (steerConfig.SingleInputWAS)   //Single Input ADS
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
      steeringPosition = adc.getConversion();
      adc.triggerConversion();//ADS1115 Single Mode

      steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
      helloSteerPosition = steeringPosition - 6800;
    }
    else    //ADS1115 Differential Mode
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
      steeringPosition = adc.getConversion();
      adc.triggerConversion();

      steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
      helloSteerPosition = steeringPosition - 6800;
    }

    //DETERMINE ACTUAL STEERING POSITION

    //convert position to steer angle. 32 counts per degree of steer pot position in my case
    //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (steerConfig.InvertWAS)
    {
      steeringPosition = (steeringPosition - 6805  - steerSettings.wasOffset);   // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    }
    else
    {
      steeringPosition = (steeringPosition - 6805  + steerSettings.wasOffset);   // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    }

    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      //Enable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        // Cytron MD30C: Controle via DIR1_RL_ENABLE e PWM1_LPWM apenas
        // Não precisa controlar PWM2_RPWM
      }
      else digitalWrite(DIR1_RL_ENABLE, 1); //IBT2 enable

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
      //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      calcSteeringPID();  //do the pid
      motorDrive();       //out to motors the pwm value
      // Autosteer Led goes GREEN if autosteering

      //digitalWrite(AUTOSTEER_ACTIVE_LED, 1);   //Commented to save ESP32 pins
      //digitalWrite(AUTOSTEER_STANDBY_LED, 0);  //Commented to save ESP32 pins
    }
    else
    {
      //we've lost the comm to AgOpenGPS, or just stop request
      //Disable H Bridge for IBT2, hyd aux, etc for cytron
      
      pwmDrive = 0; //turn off steering motor FIRST
      pwmDisplay = 0;
      
      if (steerConfig.CytronDriver)
      {
        // Cytron: Zera o PWM via LEDC e coloca DIR em LOW
        digitalWrite(DIR1_RL_ENABLE, LOW); // Direção em LOW como segurança
        ledcWrite(0, 0);  // Desliga PWM no canal 0 (PWM1_LPWM)
        ledcWrite(1, 0);  // Desliga canal 1 também por segurança
      }
      else 
      {
        // IBT2: Desliga enable e zera ambos canais PWM
        digitalWrite(DIR1_RL_ENABLE, 0); //IBT2 disable
        ledcWrite(0, 0);  // Desliga PWM Left
        ledcWrite(1, 0);  // Desliga PWM Right
      }
      
      pulseCount = 0;
      // Autosteer Led goes back to RED when autosteering is stopped
      //digitalWrite (AUTOSTEER_STANDBY_LED, 1);  //Commented to save ESP32 pins
      //digitalWrite (AUTOSTEER_ACTIVE_LED, 0);   //Commented to save ESP32 pins
    }
  } //end of timed loop

  //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
  //delay(1);

  // Speed pulse
  /*
  if (gpsSpeedUpdateTimer < 1000)
  {
      if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
      {
          speedPulseUpdateTimer = 0;

          //130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
          //gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
          float speedPulse = gpsSpeed * 36.1111;

          //Serial.print(gpsSpeed); Serial.print(" -> "); Serial.println(speedPulse);

          if (gpsSpeed > 0.11) { // 0.10 wasn't high enough
              tone(velocityPWM_Pin, uint16_t(speedPulse));
          }
          else {
              noTone(velocityPWM_Pin);
          }
      }
  }
  else  // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
  {
      noTone(velocityPWM_Pin);
  }
  */

  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if ( lastEnc) EncoderFunc();
    }
  }

} // end of main loop

int currentRoll = 0;
int rollLeft = 0;
int steerLeft = 0;

// UDP Receive
void ReceiveUdp()
{
    // When ethernet is not running, return directly. parsePacket() will block when we don't
    if (!Ethernet_running)
    {
        return;
    }

    uint16_t len = Eth_udpAutoSteer.parsePacket();

    // if (len > 0)
    // {
    //  Serial.print("ReceiveUdp: ");
    //  Serial.println(len);
    // }

    // Check for len > 4, because we check byte 0, 1, 3 and 3
    if (len > 4)
    {
        Eth_udpAutoSteer.read(autoSteerUdpData, 1460);

        if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
        {
            if (autoSteerUdpData[3] == 0xFE && Autosteer_running)  //254
            {
                gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
                gpsSpeedUpdateTimer = millis();

                prevGuidanceStatus = guidanceStatus;

                guidanceStatus = autoSteerUdpData[7];
                guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

                //Bit 8,9    set point steer angle * 100 is sent
                steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; //high low bytes

                //Serial.print("steerAngleSetPoint: ");
                //Serial.println(steerAngleSetPoint);

                //Serial.println(gpsSpeed);

                if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
                {
                    watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
                }
                else          //valid conditions to turn on autosteer
                {
                    watchdogTimer = 0;  //reset watchdog
                }

                //Bit 10 Tram
                tram = autoSteerUdpData[10];

                //Bit 11
                relay = autoSteerUdpData[11];

                //Bit 12
                relayHi = autoSteerUdpData[12];

                //----------------------------------------------------------------------------
                //Serial Send to agopenGPS

                int16_t sa = (int16_t)(steerAngleActual * 100);

                PGN_253[5] = (uint8_t)sa;
                PGN_253[6] = sa >> 8;

                // heading
                PGN_253[7] = (uint8_t)9999;
                PGN_253[8] = 9999 >> 8;

                // roll
                PGN_253[9] = (uint8_t)8888;
                PGN_253[10] = 8888 >> 8;

                PGN_253[11] = switchByte;
                PGN_253[12] = (uint8_t)pwmDisplay;

                //checksum
                int16_t CK_A = 0;
                for (uint8_t i = 2; i < PGN_253_Size; i++)
                    CK_A = (CK_A + PGN_253[i]);

                PGN_253[PGN_253_Size] = CK_A;

                //off to AOG
                SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

                //Steer Data 2 -------------------------------------------------
                //if (steerConfig.PressureSensor || steerConfig.CurrentSensor)  //REMOVED PressureSensor
                if (steerConfig.CurrentSensor)
                {
                    if (aog2Count++ > 2)
                    {
                        //Send fromAutosteer2
                        PGN_250[5] = (byte)sensorReading;

                        //add the checksum for AOG2
                        CK_A = 0;

                        for (uint8_t i = 2; i < PGN_250_Size; i++)
                        {
                            CK_A = (CK_A + PGN_250[i]);
                        }

                        PGN_250[PGN_250_Size] = CK_A;

                        //off to AOG
                        SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
                        aog2Count = 0;
                    }
                }

                //Serial.println(steerAngleActual);
                //--------------------------------------------------------------------------
            }

            //steer settings
            else if (autoSteerUdpData[3] == 0xFC && Autosteer_running)  //252
            {
                //PID values
                steerSettings.Kp = ((float)autoSteerUdpData[5]);   // read Kp from AgOpenGPS

                steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

                steerSettings.lowPWM = (float)autoSteerUdpData[7];   // read lowPWM from AgOpenGPS

                steerSettings.minPWM = autoSteerUdpData[8]; //read the minimum amount of PWM for instant on

                float temp = (float)steerSettings.minPWM * 1.2;
                steerSettings.lowPWM = (byte)temp;

                steerSettings.steerSensorCounts = autoSteerUdpData[9]; //sent as setting displayed in AOG

                steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo

                steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi

                steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

                //crc
                //autoSteerUdpData[13];

                //store in EEPROM
                EEPROM.put(10, steerSettings);
                EEPROM.commit();

                // Re-Init steer settings
                steerSettingsInit();
            }

            else if (autoSteerUdpData[3] == 0xFB)  //251 FB - SteerConfig
            {
                uint8_t sett = autoSteerUdpData[5]; //setting0

                if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
                if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
                if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
                if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
                if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
                if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
                if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
                if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

                steerConfig.PulseCountMax = autoSteerUdpData[6];

                //was speed
                //autoSteerUdpData[7];

                sett = autoSteerUdpData[8]; //setting1 - Danfoss valve etc

                if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
                //if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;  //REMOVED - not used
                if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
                if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

                //crc
                //autoSteerUdpData[13];

                EEPROM.put(40, steerConfig);
                EEPROM.commit();

                // Re-Init
                steerConfigInit();

            }//end FB
            else if (autoSteerUdpData[3] == 200) // Hello from AgIO
            {
                if(Autosteer_running)
                {
                int16_t sa = (int16_t)(steerAngleActual * 100);

                helloFromAutoSteer[5] = (uint8_t)sa;
                helloFromAutoSteer[6] = sa >> 8;

                helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
                helloFromAutoSteer[8] = helloSteerPosition >> 8;
                helloFromAutoSteer[9] = switchByte;

                SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
                }
                if(useBNO08x)
                {
                 SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination); 
                }
            }

            else if (autoSteerUdpData[3] == 201)
            {
             //make really sure this is the subnet pgn
             if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
             {
              networkAddress.ipOne = autoSteerUdpData[7];
              networkAddress.ipTwo = autoSteerUdpData[8];
              networkAddress.ipThree = autoSteerUdpData[9];
        
              //save in EEPROM and restart
              EEPROM.put(60, networkAddress);
              EEPROM.commit();
              ESP.restart(); // Reset ESP32
              }
            }//end 201

            //whoami
            else if (autoSteerUdpData[3] == 202)
            {
                //make really sure this is the reply pgn
                if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
                {
                    IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

                    //hello from AgIO
                    uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
                        Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3], 
                        rem_ip[0],rem_ip[1],rem_ip[2], 23 };

                    //checksum
                    int16_t CK_A = 0;
                    for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
                    {
                        CK_A = (CK_A + scanReply[i]);
                    }
                    scanReply[sizeof(scanReply)-1] = CK_A;

                    static uint8_t ipDest[] = { 255,255,255,255 };
                    uint16_t portDest = 9999; //AOG port that listens

                    //off to AOG
                    SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
                }
            }
        } //end if 80 81 7F
    }
}

void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
  Eth_udpAutoSteer.beginPacket(dip, dport);
  Eth_udpAutoSteer.write(data, datalen);
  Eth_udpAutoSteer.endPacket();
}

//ISR Steering Wheel Encoder
void EncoderFunc()
{
  if (encEnable)
  {
    pulseCount++;
    encEnable = false;
  }
}
