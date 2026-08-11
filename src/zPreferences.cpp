#include <Arduino.h>
#include <Preferences.h>
#include "GlobalVariables.h"

static const char* NVS_NAMESPACE = "aog";

static void putSettings(Preferences& prefs)
{
    prefs.putUChar("st_kp", steerSettings.Kp);
    prefs.putUChar("st_lowpwm", steerSettings.lowPWM);
    prefs.putShort("st_wasoff", steerSettings.wasOffset);
    prefs.putUChar("st_minpwm", steerSettings.minPWM);
    prefs.putUChar("st_highpwm", steerSettings.highPWM);
    prefs.putFloat("st_senscnt", steerSettings.steerSensorCounts);
    prefs.putFloat("st_ackfix", steerSettings.AckermanFix);
}

static void getSettings(Preferences& prefs)
{
    steerSettings.Kp = prefs.getUChar("st_kp", 40);
    steerSettings.lowPWM = prefs.getUChar("st_lowpwm", 10);
    steerSettings.wasOffset = prefs.getShort("st_wasoff", 0);
    steerSettings.minPWM = prefs.getUChar("st_minpwm", 9);
    steerSettings.highPWM = prefs.getUChar("st_highpwm", 60);
    steerSettings.steerSensorCounts = prefs.getFloat("st_senscnt", 30);
    steerSettings.AckermanFix = prefs.getFloat("st_ackfix", 1);
}

static void putConfig(Preferences& prefs)
{
    prefs.putUChar("sc_invwas", steerConfig.InvertWAS);
    prefs.putUChar("sc_relayah", steerConfig.IsRelayActiveHigh);
    prefs.putUChar("sc_motordir", steerConfig.MotorDriveDirection);
    prefs.putUChar("sc_singlewas", steerConfig.SingleInputWAS);
    prefs.putUChar("sc_cytron", steerConfig.CytronDriver);
    prefs.putUChar("sc_steersw", steerConfig.SteerSwitch);
    prefs.putUChar("sc_steerbtn", steerConfig.SteerButton);
    prefs.putUChar("sc_shaftenc", steerConfig.ShaftEncoder);
    prefs.putUChar("sc_pressens", steerConfig.PressureSensor);
    prefs.putUChar("sc_currsens", steerConfig.CurrentSensor);
    prefs.putUChar("sc_pulsemax", steerConfig.PulseCountMax);
    prefs.putUChar("sc_danfoss", steerConfig.IsDanfoss);
    prefs.putUChar("sc_useyaxis", steerConfig.IsUseY_Axis);
}

static void getConfig(Preferences& prefs)
{
    steerConfig.InvertWAS = prefs.getUChar("sc_invwas", 0);
    steerConfig.IsRelayActiveHigh = prefs.getUChar("sc_relayah", 0);
    steerConfig.MotorDriveDirection = prefs.getUChar("sc_motordir", 0);
    steerConfig.SingleInputWAS = prefs.getUChar("sc_singlewas", 1);
    steerConfig.CytronDriver = prefs.getUChar("sc_cytron", 1);
    steerConfig.SteerSwitch = prefs.getUChar("sc_steersw", 0);
    steerConfig.SteerButton = prefs.getUChar("sc_steerbtn", 0);
    steerConfig.ShaftEncoder = prefs.getUChar("sc_shaftenc", 0);
    steerConfig.PressureSensor = prefs.getUChar("sc_pressens", 0);
    steerConfig.CurrentSensor = prefs.getUChar("sc_currsens", 0);
    steerConfig.PulseCountMax = prefs.getUChar("sc_pulsemax", 5);
    steerConfig.IsDanfoss = prefs.getUChar("sc_danfoss", 0);
    steerConfig.IsUseY_Axis = prefs.getUChar("sc_useyaxis", 0);
}

static void putNetIP(Preferences& prefs)
{
    prefs.putUChar("ip_one", networkAddress.ipOne);
    prefs.putUChar("ip_two", networkAddress.ipTwo);
    prefs.putUChar("ip_three", networkAddress.ipThree);
}

static void getNetIP(Preferences& prefs)
{
    networkAddress.ipOne = prefs.getUChar("ip_one", 192);
    networkAddress.ipTwo = prefs.getUChar("ip_two", 168);
    networkAddress.ipThree = prefs.getUChar("ip_three", 137);
}

void nvsLoadAll()
{
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);

    if (!prefs.isKey("ident"))
    {
        putSettings(prefs);
        putConfig(prefs);
        putNetIP(prefs);
        prefs.putInt("ident", EEP_Ident);
    }
    else
    {
        getSettings(prefs);
        getConfig(prefs);
        getNetIP(prefs);
    }

    prefs.end();
}

void nvsSaveSettings()
{
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    putSettings(prefs);
    prefs.end();
    Serial.println("Settings saved to NVS");
}

void nvsSaveConfig()
{
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    putConfig(prefs);
    prefs.end();
    Serial.println("Config saved to NVS");
}

void nvsSaveNetIP()
{
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    putNetIP(prefs);
    prefs.end();
    Serial.println("Network IP saved to NVS");
}
