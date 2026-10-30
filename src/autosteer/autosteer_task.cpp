/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/


#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include "autosteer/autosteer_task.h"
#include "autosteer/pid.h"
#include "autosteer/safety.h"
#include "hardware/ads1115.h"
#include "hardware/motor_pwm.h"
#include "config/pinout.h"
#include "config/defines.h"
#include "config/settings.h"
#include "state/guidance_state.h"
#include "state/steer_state.h"
#include "state/pending.h"
#include "diagnostics/log.h"
namespace autosteer {
namespace {
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);
Storage steerSettings;
Setup steerConfig;
GuidanceCommand command;
SteerStatus status;
diagnostics::TimingCounters cycleTiming, adcTiming;
uint32_t cycleLateMs = 0;
bool available = false, commandEnabled = false;
uint32_t autsteerLastTime = 0, currentTime = 0, consumedTransition = 0;
uint8_t workSwitch = 0, steerSwitch = 1, remoteSwitch = 0, switchByte = 2;
uint8_t guidanceStatus = 0;
bool guidanceStatusChanged = false;
float gpsSpeed = 0, steerAngleSetPoint = 0, steerAngleActual = 0, steerAngleError = 0;
float sensorReading = 0, sensorSample = 0;
int16_t steeringPosition = 0, helloSteerPosition = 0, pwmDrive = 0, pwmDisplay = 0;
uint8_t currentState = 1, reading = 0, previous = 0, pulseCount = 0;
bool encEnable = false;
uint8_t thisEnc = 0, lastEnc = 0;
float inputWAS[] = {-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50};
float outputWAS[] = {-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50};
template<typename T>
T multiMap(T value, T* _in, T* _out, uint8_t size)
{
    if (value <= _in[0]) return _out[0];
    if (value >= _in[size - 1]) return _out[size - 1];
    uint8_t pos = 1;
    while (value > _in[pos]) pos++;
    if (value == _in[pos]) return _out[pos];
    return (value - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}
void EncoderFunc() {
    if (encEnable) { ++pulseCount; encEnable = false; }
}
void disengage() {
    commandEnabled = false;
    steerSwitch = 1; currentState = 1; previous = 0;
    pwmDrive = 0; pwmDisplay = 0;
    motor::stop(motor::StopSource::Control);
}
void publishCycle(uint32_t startUs) {
    status.available = available;
    status.engaged = commandEnabled && steerSwitch == 0 && !pending::maintenanceActive();
    status.steerAngleActual = steerAngleActual;
    status.sensorReading = sensorReading;
    status.helloSteerPosition = helloSteerPosition;
    status.pwmDisplay = pwmDisplay;
    status.steerSwitch = steerSwitch;
    status.switchByte = (remoteSwitch << 2) | (steerSwitch << 1) | workSwitch;
    status.completedMs = millis();
    status.guidanceTransitionSequence = consumedTransition;
    ++status.cycles;
    uint32_t duration = micros() - startUs;
    if (duration > status.maxCycleUs) status.maxCycleUs = duration;
    cycleTiming.record(startUs, duration, cycleLateMs, CONTROL_PERIOD_MS * 1000);
    state::publishSteer(status);
}
void applyPending() {
    pending::Update update;
    if (!pending::take(update)) return;
    disengage();
    ConfigSnapshot values = config::snapshot();
    if (update.mask & pending::Settings) values.steerSettings = update.values.steerSettings;
    if (update.mask & pending::Config) values.steerConfig = update.values.steerConfig;
    if (update.mask & pending::Network) values.networkAddress = update.values.networkAddress;
    steerSettings = values.steerSettings;
    steerConfig = values.steerConfig;
    config::publish(values);
    // A conversão seguinte já usa o MUX da nova configuração.
    if (available) {
        adc.setMux(steerConfig.SingleInputWAS ? ADS1115_REG_CONFIG_MUX_SINGLE_0 : ADS1115_REG_CONFIG_MUX_DIFF_0_1);
        adc.triggerConversion();
    }
    update.values = values;
    pending::requestPersistence(update);
    status.engaged = false; status.pwmDisplay = 0; status.steerSwitch = 1;
    status.switchByte = (remoteSwitch << 2) | (steerSwitch << 1) | workSwitch;
    state::publishSteer(status);
}
}
diagnostics::Timing takeCycleTiming() { return cycleTiming.take(); }
diagnostics::Timing takeAdcTiming() { return adcTiming.take(); }
bool setup(bool motorReady) {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(WORKSW_PIN, INPUT);
    pinMode(STEERSW_PIN, INPUT);
    pinMode(REMOTE_PIN, INPUT);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    ConfigSnapshot values = config::snapshot();
    steerSettings = values.steerSettings;
    steerConfig = values.steerConfig;
    available = adc.testConnection() && motorReady;
    AOG_LOG_PRINTLN(available ? "ADC Connecton OK" : "ADC Connecton FAILED!");
    adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS);
    adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
    adc.setMux(steerConfig.SingleInputWAS ? ADS1115_REG_CONFIG_MUX_SINGLE_0 : ADS1115_REG_CONFIG_MUX_DIFF_0_1);
    if (available) available = adc.triggerConversion();
    status.available = available;
    state::publishSteer(status);
    AOG_LOG_PRINTLN(available ? "Autosteer running, waiting for AgOpenGPS" : "Autosteer disabled, GPS only mode");
    return available;
}
void poll()
{
  applyPending();
  if (!available) return;
  GuidanceCommand incoming = state::guidanceSnapshot();
  if (incoming.sequence != command.sequence) {
    command = incoming;
    gpsSpeed = command.gpsSpeed;
    steerAngleSetPoint = command.steerAngleSetPoint;
  }

  currentTime = millis();

  if (currentTime - autsteerLastTime >= CONTROL_PERIOD_MS)
  {
    cycleLateMs = status.cycles ? currentTime - autsteerLastTime - CONTROL_PERIOD_MS : 0;
    autsteerLastTime = currentTime;
    uint32_t startUs = micros();
    uint32_t outputEpoch = motor::epoch();
    guidanceStatus = command.guidanceStatus;
    guidanceStatusChanged = command.transitionSequence != consumedTransition;
    consumedTransition = command.transitionSequence;

    encEnable = true;

    workSwitch = digitalRead(WORKSW_PIN);  // read work switch

    reading = digitalRead(STEERSW_PIN);

    if (steerConfig.SteerSwitch == 1)         //steer switch on - off
    {
      if (reading == HIGH)
      {
        currentState = 1;
        steerSwitch = 1;
        previous = reading;  // Bug5 fix: sync previous so edge detection works on next press
      }
    }

    if (steerConfig.SteerButton == 1 || steerConfig.SteerSwitch == 1)
    {
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
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
      {
        steerSwitch = 0;
        previous = 1;
      }

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



    if (steerConfig.CurrentSensor)
    {
      sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);

      if (userSettings.isIBT2CurrentSensor)
      {
        sensorSample = sensorSample * 0.06227f;
      }
      else
      {
        sensorSample = abs(1992.0f - sensorSample) * 0.155f;
      }

      sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
      sensorReading = min(sensorReading, 255.0f);

      if (sensorReading >= (float)steerConfig.PulseCountMax)
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



    uint32_t adcStartedUs = micros();
    bool adcOk = true;
    if (steerConfig.SingleInputWAS)   //Single Input ADS
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
      adcOk = adc.getConversion(steeringPosition);
      adcOk = adc.triggerConversion() && adcOk;//ADS1115 Single Mode

      steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
      helloSteerPosition = steeringPosition - 6800;
    }
    else    //ADS1115 Differential Mode
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
      adcOk = adc.getConversion(steeringPosition);
      adcOk = adc.triggerConversion() && adcOk;

      steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
      helloSteerPosition = steeringPosition - 6800;
    }


    adcTiming.record(adcStartedUs, micros() - adcStartedUs, 0, CONTROL_PERIOD_MS * 1000);
    if (!available || !adcOk || !std::isfinite(steerSettings.steerSensorCounts) || steerSettings.steerSensorCounts <= 0) {
      disengage();
      publishCycle(startUs);
      return;
    }
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

    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    if ((steerAngleActual < inputWAS[0]) || (steerAngleActual > inputWAS[20]) || gpsSpeed > MAX_STEER_SPEED_KMH)
    {
        steerSwitch = 1;
        currentState = 1;
        previous = 0;
        commandEnabled = false;
    }

    steerAngleActual = multiMap<float>(steerAngleActual, inputWAS, outputWAS, 21);

    // Engajamento lógico usa a entrada e o WAS deste ciclo, inclusive parado.
    commandEnabled = safety::commandFresh(millis(), command) && (command.guidanceStatus & 1) &&
        steerSwitch == 0 && std::fabs(steerAngleActual) <= WAS_ANGLE_MAX && !pending::maintenanceActive();
    // O AOG envia velocidade de teste de 8 km/h no Free Drive (PGN 254).
    if (commandEnabled && safety::pwmSpeedAllowed(command)) {
      steerAngleError = steerAngleActual - steerAngleSetPoint;
      pwmDrive = calcSteeringPID(steerAngleError, steerSettings, steerConfig);
      pwmDisplay = motor::drive(pwmDrive, steerConfig, outputEpoch);
    } else {
      motor::stop(motor::StopSource::Control);
      pwmDrive = 0;
      pwmDisplay = 0;
      pulseCount = 0;
    }
    publishCycle(startUs);
  } //end of timed loop




  if (encEnable && steerConfig.ShaftEncoder)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if ( lastEnc) EncoderFunc();
    }
  }

}
}
