/* Global Variables and Declarations Header
 * This file contains extern declarations for variables shared across .cpp files
 * Include this in any .cpp file that needs access to global variables
 */

#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H

#include <Arduino.h>
#include <WiFiUdp.h>
#include <IPAddress.h>
#include <SimpleKalmanFilter.h>
#include "BNO08x_AOG.h"
#include "zNMEAParser.h"

// Constants
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105
#define twoPI 6.28318530717958647692
#define PIBy2 1.57079632679489661923

// User settings
constexpr bool invertRoll = true;  // Used for IMU with dual antenna
constexpr bool useYawRate = false;  // Enable gyroscope yaw rate (requires BNO08x)
constexpr bool useMagnetometer = true;  // Use RotationVector (with mag) instead of GameRotationVector (no mag)

// ConfigIP struct definition
struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 137;
};

// Storage struct definition
struct Storage {
  uint8_t Kp = 40;
  uint8_t lowPWM = 10;
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;
  float steerSensorCounts = 30;
  float AckermanFix = 1;
};

// Setup struct definition
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

// Forward declarations for cross-file functions
void autosteerSetup();
void autosteerLoop();
void EthernetStart();
void udpNtrip();
void errorHandler();
void GGA_Handler();
void VTG_Handler();
void HPR_Handler();
void readBNO();
void BuildNmea();
void CalculateChecksum();  // Uses global nmea variable
void imuHandler();
void imuDualDelta();
void fuseIMU();
void calcSteeringPID();
void motorDrive();
void ReceiveUdp();
void SendUdp(uint8_t* data, uint8_t length, IPAddress dip, uint16_t dport);

// Global variables from AOG_Teensy_UM982.cpp
extern bool udpPassthrough;
extern bool makeOGI;
extern bool baseLineCheck;
extern double headingcorr;
extern bool filterRoll;
extern float rollMEA;
extern float rollEST;
extern float rollQ;
extern bool filterHeading;
extern float headingMEA;
extern float headingEST;
extern float headingQ;
extern bool sendUSB;
extern bool gotCR;
extern bool gotLF;
extern bool gotDollar;
extern char msgBuf[254];
extern int msgBufLen;
extern uint32_t READ_BNO_TIME;
extern uint32_t gpsReadyTime;
extern ConfigIP networkAddress;
extern byte Eth_myip[4];
extern byte mac[];
extern unsigned int portMy;
extern unsigned int AOGNtripPort;
extern unsigned int AOGAutoSteerPort;
extern unsigned int portDestination;
extern char Eth_NTRIP_packetBuffer[512];
extern WiFiUDP Eth_udpPAOGI;
extern WiFiUDP Eth_udpNtrip;
extern WiFiUDP Eth_udpAutoSteer;
extern IPAddress Eth_ipDestination;
extern byte CK_A;
extern byte CK_B;
extern int relposnedByteCount;
extern uint32_t speedPulseUpdateTimer;
extern byte velocityPWM_Pin;
extern bool dualReadyGGA;
extern bool dualReadyRelPos;
extern bool useBNO08x;
extern uint8_t bno08xAddress;
extern BNO080 bno08x;
extern double baseline;
extern double rollDual;
extern double pitchDual;
extern double relPosD;
extern double heading;
extern byte ackPacket[72];
extern NMEAParser<4> parser;
extern bool isTriggered;
extern bool blink;
extern bool Autosteer_running;
extern bool Ethernet_running;

// Global variables from zHandlers.cpp
extern char nmea[100];
extern char fixTime[12];
extern char latitude[15];
extern char latNS[3];
extern char longitude[15];
extern char lonEW[3];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char altitude[12];
extern char ageDGPS[10];
extern char vtgHeading[12];
extern char speedKnots[10];
extern char umHeading[8];
extern char umRoll[8];
extern int solQuality;
extern char imuHeading[6];
extern char imuRoll[6];
extern char imuPitch[6];
extern char imuYawRate[6];
extern double gpsHeading;
extern double imuGPS_Offset;
extern double correctionHeading;
extern double gyroDelta;
extern double imuCorrected;
extern double yaw;
extern double pitch;
extern double roll;
// Moved to defines above: extern double PIBy2;
// Moved to defines above: extern double twoPI;
extern int16_t imuRollDegrees;
extern double rollDelta;
extern double rollDeltaSmooth;
extern bool GGA_Available;
extern SimpleKalmanFilter rollFilter;
extern SimpleKalmanFilter headingFilter;
// Moved to GlobalVariables.h: const bool invertRoll = true;
extern HardwareSerial* SerialGPS;

// Serial port defines moved from AOG_Esp32_UM982.cpp
#define SerialAOG Serial                //AgIO USB connection
#define SerialRTK Serial1               //RTK radio (GPIO 4/RX, 2/TX)

// Function declarations for optional features
void EncoderFunc();  // May not be defined in all builds

// LED Pin definitions from AOG_Teensy_UM982.cpp
//#define GGAReceivedLED 2          //ESP32 onboard LED (GPIO2) - Commented to save ESP32 pins
//#define Power_on_LED 14           //Red - Commented to save ESP32 pins
//#define Ethernet_Active_LED 15    //Green - Commented to save ESP32 pins
//#define GPSRED_LED 12             //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU) - Commented to save ESP32 pins
//#define GPSGREEN_LED 13           //Green (Flashing = Dual bad, ON = Dual good) - Commented to save ESP32 pins

// Pin definitions from Autosteer.cpp (Motor and Switches)
#define WORKSW_PIN 15
#define STEERSW_PIN 14
#define REMOTE_PIN 35
#define DIR1_RL_ENABLE 36
#define PWM1_LPWM 4
#define PWM2_RPWM 2
//#define AUTOSTEER_ACTIVE_LED 33    //Commented to save ESP32 pins
//#define AUTOSTEER_STANDBY_LED 32   //Commented to save ESP32 pins
#define CURRENT_SENSOR_PIN 39
#define PRESSURE_SENSOR_PIN 36

// Autosteer Configuration Constants
#define LOW_HIGH_DEGREES 5.0
#define LOOP_TIME 100
#define WATCHDOG_THRESHOLD 100
#define WATCHDOG_FORCE_VALUE 10
#define PWM_Frequency 0
#define EEP_Ident 2400
#define CONST_180_DIVIDED_BY_PI 57.2957795130823

extern uint8_t autoSteerUdpData[1460];
extern uint32_t autsteerLastTime;
extern uint32_t currentTime;
extern uint8_t watchdogTimer;
extern uint8_t helloFromIMU[11];
extern uint8_t helloFromAutoSteer[11];
extern int16_t helloSteerPosition;
extern uint8_t PGN_253[14];
extern int8_t PGN_253_Size;
extern uint8_t PGN_250[14];
extern int8_t PGN_250_Size;
extern uint8_t aog2Count;
extern float sensorReading;
extern float sensorSample;
extern uint32_t gpsSpeedUpdateTimer;
extern int16_t EEread;
extern bool isRelayActiveHigh;
extern uint8_t relay;
extern uint8_t relayHi;
extern uint8_t uTurn;
extern uint8_t tram;
extern uint8_t remoteSwitch;
extern uint8_t workSwitch;
extern uint8_t steerSwitch;
extern uint8_t switchByte;
extern uint8_t guidanceStatus;
extern uint8_t prevGuidanceStatus;
extern bool guidanceStatusChanged;
extern float gpsSpeed;
extern float steerAngleActual;
extern float steerAngleSetPoint;
extern int16_t steeringPosition;
extern float steerAngleError;
extern int16_t pwmDrive;
extern int16_t pwmDisplay;
extern float pValue;
extern float errorAbs;
extern float highLowPerDeg;
extern uint8_t currentState;
extern uint8_t reading;
extern uint8_t previous;
extern uint8_t pulseCount;
extern bool encEnable;
extern uint8_t thisEnc;
extern uint8_t lastEnc;
extern Storage steerSettings;
extern Setup steerConfig;

#endif // GLOBALVARIABLES_H
