#pragma once
#include <cstdint>

struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 137;
};
struct Storage {
    uint8_t Kp = 40;
    uint8_t lowPWM = 10;
    int16_t wasOffset = 0;
    uint8_t minPWM = 9;
    uint8_t highPWM = 60;
    float steerSensorCounts = 30;
    float AckermanFix = 1;
};
struct Setup {
    uint8_t InvertWAS = 0;
    uint8_t IsRelayActiveHigh = 0;
    uint8_t MotorDriveDirection = 0;
    uint8_t SingleInputWAS = 1;
    uint8_t CytronDriver = 1;
    uint8_t SteerSwitch = 0;
    uint8_t SteerButton = 0;
    uint8_t ShaftEncoder = 0;
    uint8_t PressureSensor = 0;
    uint8_t CurrentSensor = 0;
    uint8_t PulseCountMax = 5;
    uint8_t IsDanfoss = 0;
    uint8_t IsUseY_Axis = 0;
};
struct UserSettings {
    bool udpPassthrough = false;
    bool makeOGI = false;
    bool baseLineCheck = false;
    double headingcorr = 0;
    bool filterRoll = false;
    float rollMEA = 1, rollEST = 1, rollQ = 0.01f;
    bool filterHeading = false;
    float headingMEA = 1, headingEST = 1, headingQ = 0.01f;
    bool sendUSB = false;
    bool isIBT2CurrentSensor = false;
    bool invertRoll = true;
    bool useYawRate = true;
};
extern const UserSettings userSettings;

struct ConfigSnapshot {
    Storage steerSettings;
    Setup steerConfig;
    ConfigIP networkAddress;
};
namespace config {
ConfigSnapshot snapshot();
void publish(const ConfigSnapshot& value);
}
