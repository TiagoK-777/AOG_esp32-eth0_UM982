#include <Arduino.h>
#include <Preferences.h>
#include "persist/preferences.h"
#include "config/defines.h"
#include "state/pending.h"
namespace {
static const char* NVS_NAMESPACE = "aog";

static bool putSettings(Preferences& prefs, const ConfigSnapshot& values)
{
    bool ok = true;
    ok = (prefs.putUChar("st_kp", values.steerSettings.Kp) == 1) && ok;
    ok = (prefs.putUChar("st_lowpwm", values.steerSettings.lowPWM) == 1) && ok;
    ok = (prefs.putShort("st_wasoff", values.steerSettings.wasOffset) == 2) && ok;
    ok = (prefs.putUChar("st_minpwm", values.steerSettings.minPWM) == 1) && ok;
    ok = (prefs.putUChar("st_highpwm", values.steerSettings.highPWM) == 1) && ok;
    ok = (prefs.putFloat("st_senscnt", values.steerSettings.steerSensorCounts) == 4) && ok;
    ok = (prefs.putFloat("st_ackfix", values.steerSettings.AckermanFix) == 4) && ok;
    return ok;
}

static void getSettings(Preferences& prefs, ConfigSnapshot& values)
{
    values.steerSettings.Kp = prefs.getUChar("st_kp", 40);
    values.steerSettings.lowPWM = prefs.getUChar("st_lowpwm", 10);
    values.steerSettings.wasOffset = prefs.getShort("st_wasoff", 0);
    values.steerSettings.minPWM = prefs.getUChar("st_minpwm", 9);
    values.steerSettings.highPWM = prefs.getUChar("st_highpwm", 60);
    values.steerSettings.steerSensorCounts = prefs.getFloat("st_senscnt", 30);
    values.steerSettings.AckermanFix = prefs.getFloat("st_ackfix", 1);
}

static bool putConfig(Preferences& prefs, const ConfigSnapshot& values)
{
    bool ok = true;
    ok = (prefs.putUChar("sc_invwas", values.steerConfig.InvertWAS) == 1) && ok;
    ok = (prefs.putUChar("sc_relayah", values.steerConfig.IsRelayActiveHigh) == 1) && ok;
    ok = (prefs.putUChar("sc_motordir", values.steerConfig.MotorDriveDirection) == 1) && ok;
    ok = (prefs.putUChar("sc_singlewas", values.steerConfig.SingleInputWAS) == 1) && ok;
    ok = (prefs.putUChar("sc_cytron", values.steerConfig.CytronDriver) == 1) && ok;
    ok = (prefs.putUChar("sc_steersw", values.steerConfig.SteerSwitch) == 1) && ok;
    ok = (prefs.putUChar("sc_steerbtn", values.steerConfig.SteerButton) == 1) && ok;
    ok = (prefs.putUChar("sc_shaftenc", values.steerConfig.ShaftEncoder) == 1) && ok;
    ok = (prefs.putUChar("sc_pressens", values.steerConfig.PressureSensor) == 1) && ok;
    ok = (prefs.putUChar("sc_currsens", values.steerConfig.CurrentSensor) == 1) && ok;
    ok = (prefs.putUChar("sc_pulsemax", values.steerConfig.PulseCountMax) == 1) && ok;
    ok = (prefs.putUChar("sc_danfoss", values.steerConfig.IsDanfoss) == 1) && ok;
    ok = (prefs.putUChar("sc_useyaxis", values.steerConfig.IsUseY_Axis) == 1) && ok;
    return ok;
}

static void getConfig(Preferences& prefs, ConfigSnapshot& values)
{
    values.steerConfig.InvertWAS = prefs.getUChar("sc_invwas", 0);
    values.steerConfig.IsRelayActiveHigh = prefs.getUChar("sc_relayah", 0);
    values.steerConfig.MotorDriveDirection = prefs.getUChar("sc_motordir", 0);
    values.steerConfig.SingleInputWAS = prefs.getUChar("sc_singlewas", 1);
    values.steerConfig.CytronDriver = prefs.getUChar("sc_cytron", 1);
    values.steerConfig.SteerSwitch = prefs.getUChar("sc_steersw", 0);
    values.steerConfig.SteerButton = prefs.getUChar("sc_steerbtn", 0);
    values.steerConfig.ShaftEncoder = prefs.getUChar("sc_shaftenc", 0);
    values.steerConfig.PressureSensor = prefs.getUChar("sc_pressens", 0);
    values.steerConfig.CurrentSensor = prefs.getUChar("sc_currsens", 0);
    values.steerConfig.PulseCountMax = prefs.getUChar("sc_pulsemax", 5);
    values.steerConfig.IsDanfoss = prefs.getUChar("sc_danfoss", 0);
    values.steerConfig.IsUseY_Axis = prefs.getUChar("sc_useyaxis", 0);
}

static bool putNetIP(Preferences& prefs, const ConfigSnapshot& values)
{
    bool ok = true;
    ok = (prefs.putUChar("ip_one", values.networkAddress.ipOne) == 1) && ok;
    ok = (prefs.putUChar("ip_two", values.networkAddress.ipTwo) == 1) && ok;
    ok = (prefs.putUChar("ip_three", values.networkAddress.ipThree) == 1) && ok;
    return ok;
}

static void getNetIP(Preferences& prefs, ConfigSnapshot& values)
{
    values.networkAddress.ipOne = prefs.getUChar("ip_one", 192);
    values.networkAddress.ipTwo = prefs.getUChar("ip_two", 168);
    values.networkAddress.ipThree = prefs.getUChar("ip_three", 137);
}


}
namespace persist {
bool load(ConfigSnapshot& values) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    bool ok = true;
    if (!prefs.isKey("ident")) {
        ok = putSettings(prefs, values) && ok;
        ok = putConfig(prefs, values) && ok;
        ok = putNetIP(prefs, values) && ok;
        if (ok) ok = prefs.putInt("ident", EEP_Ident) == 4;
    } else {
        getSettings(prefs, values);
        getConfig(prefs, values);
        getNetIP(prefs, values);
    }
    prefs.end();
    return ok;
}
bool save(const ConfigSnapshot& values, uint8_t mask) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    bool ok = true;
    if (mask & pending::Settings) ok = putSettings(prefs, values) && ok;
    if (mask & pending::Config) ok = putConfig(prefs, values) && ok;
    if (mask & pending::Network) ok = putNetIP(prefs, values) && ok;
    prefs.end();
    return ok;
}
}
