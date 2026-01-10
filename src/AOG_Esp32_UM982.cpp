/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
Forked from https://github.com/AgHardware/Boards/blob/main/TeensyModules/AIO%20Standard%20v4/Firmware/Autosteer_gps_teensy_v4/Autosteer_gps_teensy_v4.ino
*/

#include <Arduino.h>
#include "GlobalVariables.h"
#include "zNMEAParser.h"
#include <Wire.h>
// #include "BNO08x_AOG.h"  // Commented for UART-RVC mode
#include <SimpleKalmanFilter.h>
// Ethernet for ESP32 WT32-ETH01
#include <ETH.h>
#include <WiFiUdp.h>

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
void CalculateChecksum(char* Sentence);
void imuHandler();
void imuDualDelta();
void fuseIMU();

/************************* User Settings *************************/
bool udpPassthrough = false;  // False = GPS neeeds to send GGA, VTG & HPR messages. True = GPS needs to send KSXT messages only.
bool makeOGI = false;         //Set to true to make PAOGI messages. Else PANDA message will be made.
bool baseLineCheck = false;   //Set to true to use IMU fusion with UM982. Habilitar se estiver usando antenas duplas com UM982, e não tiver IMU conectado.
// Moved to GlobalVariables.h: const bool invertRoll= true;  //Used for IMU with dual antenna
#define baseLineLimit 5       //Max CM differance in baseline

// Heading correction can be enetered into the UM982 config or AOG GUI so this can be 0. If not in UM982 config or AOG GUI, set here.
// Negative number = west, positive number = east.
double headingcorr = 0;
// double headingcorr = 900;  //90deg heading correction (90deg*10)
// Roll correction can be entered in the AOG GUI. If not enter roll correction here.
// Roll correction. Negative number = left; positive number = right.
//double rollcorr = 50;

// Kalman Filtering
// e_mea: Measurement Uncertainty - How much do we expect to our measurement vary
// e_est: Estimation Uncertainty - Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
// q: Process Variance - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01. Should be tunned to your needs.
bool filterRoll = false;
float rollMEA = 1;
float rollEST = 1;
float rollQ = 0.01;

bool filterHeading = false;
float headingMEA = 1;
float headingEST = 1;
float headingQ = 0.01;

// Serial Ports - ESP32 WT32-ETH01 (SerialAOG and SerialRTK defined in GlobalVariables.h)
HardwareSerial* SerialGPS = &Serial2;   //Main position receiver (GGA - GPIO 5/RX, 17/TX)
const int32_t baudAOG = 115200;         //USB connection speed
const int32_t baudGPS = 460800;         //UM982 connection speed
const int32_t baudRTK = 9600;           // most are using Xbee radios with default of 115200

// Send data to AgIO via usb
bool sendUSB = false;

/************************* End User Settings **********************/

SimpleKalmanFilter rollFilter(rollMEA, rollEST, rollQ);
SimpleKalmanFilter headingFilter(headingMEA, headingEST, headingQ);

bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;

#define ImuWire Wire        //Wire é inicializado em autosteerSetup(): SDA=GPIO32, SCL=GPIO33

#define REPORT_INTERVAL 20    //BNO report time, we want to keep reading it quick & often. Its not timed to anything just give constant data.
uint32_t READ_BNO_TIME = 0;   //Used stop BNO data pile up (This version is without resetting BNO everytime)

// LED pin definitions moved to GlobalVariables.h
uint32_t gpsReadyTime = 0;        //Used for GGA timeout

void errorHandler();
void GGA_Handler();
void VTG_Handler();
void HPR_Handler();
void autosteerSetup();
void EthernetStart();
void udpNtrip();
void BuildNmea();
void relPosDecode();
void readBNO();
void autosteerLoop();
void ReceiveUdp();

// Global instances
ConfigIP networkAddress;   //3 bytes
Storage steerSettings;     // 11 bytes - Autosteer settings
Setup steerConfig;         // 9 bytes - Autosteer configuration

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0}; //This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;             // port of this module
unsigned int AOGNtripPort = 2233;       // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;   // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;    // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];       // buffer for receiving ntrip data

// WiFiUDP instance for ESP32 to send and receive packets over UDP
WiFiUDP Eth_udpPAOGI;     //Out port 5544
WiFiUDP Eth_udpNtrip;     //In port 2233
WiFiUDP Eth_udpAutoSteer; //In & Out Port 8888

IPAddress Eth_ipDestination;

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

//Speed pulse output
uint32_t speedPulseUpdateTimer = 0;
//byte velocityPWM_Pin = 36;      // Velocity (MPH speed) PWM pin

bool dualReadyGGA = false;
bool dualReadyRelPos = false;

// booleans to see if we are using BNO08x
bool useBNO08x = false;

// BNO085 UART-RVC mode variables
HardwareSerial SerialRVC(1);  // Use Serial1 for GPIO2 RX
uint32_t lastRvcTime = 0;     // Watchdog for RVC data
uint32_t rvcPacketCount = 0;  // RVC packets received counter
double gyroZ = 0;             // Yaw rate (not available in RVC, set to 0)

// BNO08x I2C variables - COMMENTED for RVC mode
// const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
// const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress = 0;
// BNO080 bno08x;

double baseline = 0;
double rollDual = 0;
double pitchDual = 0;
double relPosD = 0;
double heading = 0;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];   //Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];    //Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<4> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true; //Auto set off in autosteer setup
bool Ethernet_running = false; //Auto set on in ethernet setup
bool GGA_Available = false;    //Do we have GGA on correct port?
uint32_t PortSwapTime = 0;

double roll = 0;
double pitch = 0;
double yaw = 0;

//Fusing BNO with Dual
double rollDelta;
double rollDeltaSmooth;
double correctionHeading;
double gyroDelta;
double imuGPS_Offset;
double gpsHeading;
double imuCorrected;
// Moved to GlobalVariables.h: #define twoPI 6.28318530717958647692
// Moved to GlobalVariables.h: #define PIBy2 1.57079632679489661923

// Buffer to read chars from Serial, to check if "!AOG" is found
uint8_t aogSerialCmd[4] = { '!', 'A', 'O', 'G'};
uint8_t aogSerialCmdBuffer[6];
uint8_t aogSerialCmdCounter = 0;

//-=-=-=-=- UBX binary specific variables
struct ubxPacket
{
	uint8_t cls;
	uint8_t id;
	uint16_t len; //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter; //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload; // We will allocate RAM for the payload if/when needed.
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
    
	////sfe_ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	////sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

// Setup procedure ---------------------------------------------------------------------------------------------------------------
void setup()
{
  delay(500);                         //Small delay so serial can monitor start up

  //pinMode(GGAReceivedLED, OUTPUT);         //Commented to save ESP32 pins
  //pinMode(Power_on_LED, OUTPUT);           //Commented to save ESP32 pins
  //pinMode(Ethernet_Active_LED, OUTPUT);    //Commented to save ESP32 pins
  //pinMode(GPSRED_LED, OUTPUT);             //Commented to save ESP32 pins
  //pinMode(GPSGREEN_LED, OUTPUT);           //Commented to save ESP32 pins
  //pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);  //Commented to save ESP32 pins
  //pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);   //Commented to save ESP32 pins

  // the dash means wildcard
 
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
  if (baseLineCheck) {
    parser.addHandler("G-HPR", HPR_Handler);
  }

  delay(10);
  Serial.begin(baudAOG);
  delay(10);
  Serial.println("Start setup");

  SerialGPS->begin(baudGPS, SERIAL_8N1, 5, 17); // RX=GPIO5, TX=GPIO17
  //SerialGPS->setRxBufferSize(serial_buffer_size); // ESP32 buffer config

  delay(10);
  //SerialRTK.begin(baudRTK, SERIAL_8N1, 4, 2); // RX=GPIO4, TX=GPIO2

  Serial.println("SerialAOG, SerialRTK, SerialGPS initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();
  
  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();

  Serial.println("\r\nStarting BNO085 UART-RVC mode...");

  // Initialize BNO085 UART-RVC on GPIO2 (RX only)
  SerialRVC.begin(115200, SERIAL_8N1, 2, -1);  // RX=GPIO2, TX=not used
  delay(100);
  SerialRVC.flush();  // Clear any garbage data
  while(SerialRVC.available()) SerialRVC.read();  // Empty buffer
  useBNO08x = true;  // Enable RVC mode
  lastRvcTime = millis();

  Serial.println("BNO085 UART-RVC initialized on GPIO2");
  Serial.println("Waiting for RVC packets...");
  
  // Wait for first RVC packet to confirm communication
  uint32_t startTime = millis();
  bool rvcFound = false;
  while(millis() - startTime < 2000) {  // 2 second timeout
    if(SerialRVC.available() >= 2) {
      uint8_t b1 = SerialRVC.read();
      if(b1 == RVC_HEADER) {
        if(SerialRVC.available() && SerialRVC.peek() == RVC_HEADER) {
          rvcFound = true;
          Serial.println("RVC sync found!");
          break;
        }
      }
    }
  }
  
  if(!rvcFound) {
    Serial.println("WARNING: No RVC packets detected! Check BNO085 connection.");
    useBNO08x = false;
  }
  

  delay(100);
  Serial.print("useBNO08x = ");
  Serial.println(useBNO08x);

  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
  
  // Limpa buffer serial do GPS para sincronização
  delay(1000);
  while(SerialGPS->available()) {
    SerialGPS->read(); // Descarta dados antigos
  }
  Serial.println("GPS buffer cleared, ready to receive\n");
}

void loop()
{
    // Read incoming nmea from GPS
    if (SerialGPS->available())
    {
      static bool synced = false;
      
      // Sincroniza com o início da mensagem NMEA
      if (!synced) {
        char c = SerialGPS->read();
        if (c == '$') {
          synced = true;
          parser << c; // Envia o $ para o parser
        }
        // Continua no próximo ciclo até encontrar $
      }
      else if (udpPassthrough)
      {
          //char mChar;
          char incoming = SerialGPS->read();
          //Serial.println(incoming);
          switch (incoming) {
              case '$':
              msgBuf[msgBufLen] = incoming;
              msgBufLen ++;
              gotDollar = true;
              break;
              case '\r':
              msgBuf[msgBufLen] = incoming;
              msgBufLen ++;
              gotCR = true;
              gotDollar = false;
              break;
              case '\n':
              msgBuf[msgBufLen] = incoming;
              msgBufLen ++;
              gotLF = true;
              gotDollar = false;
              break;
              default:
              if (gotDollar)
                  {
                  msgBuf[msgBufLen] = incoming;
                  msgBufLen ++;
                  }
              break;
          }
          if (gotCR && gotLF){
              //Serial.print(msgBuf);
              //Serial.println(msgBufLen);
              if (sendUSB) { SerialAOG.write(msgBuf); } // Send USB GPS data if enabled in user settings
              if (Ethernet_running){
                  Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
                  Eth_udpPAOGI.write((const uint8_t*)msgBuf, msgBufLen);
                  Eth_udpPAOGI.endPacket();
              }
              gotCR = false;
              gotLF = false;
              gotDollar = false;
              memset( msgBuf, 0, 254 );
              msgBufLen = 0;
              if (blink)
              {
                  //digitalWrite(GGAReceivedLED, HIGH);  //Commented to save ESP32 pins
              }
              else
              {
                  //digitalWrite(GGAReceivedLED, LOW);   //Commented to save ESP32 pins
              }

              blink = !blink;
              //digitalWrite(GPSGREEN_LED, HIGH);   //Turn green GPS LED ON - Commented to save ESP32 pins
          }
      }
      else
      {
        // Processa normalmente após sincronização
        while (SerialGPS->available()) {
          char c = SerialGPS->read();
          parser << c;
          
          // Re-sincroniza se perder o stream
          if (c == '$') {
            synced = true;
          }
        }
      }
    }

    udpNtrip();

    // Check for RTK Radio
    /*
    if (SerialRTK.available())
    {
        SerialGPS->write(SerialRTK.read());
    }
    */

    // If both dual messages are ready, send to AgOpen
    //Serial.print(dualReadyGGA);
    //Serial.println(dualReadyRelPos);
    //delay(10);
    
    // Se baseLineCheck=false, envia apenas com GGA. Se true, espera GGA+RelPos
    bool readyToSend = false;
    if (baseLineCheck) {
        // Modo dual antenna - espera GGA + HPR/RelPos
        readyToSend = (dualReadyGGA == true && dualReadyRelPos == true);
    } else {
        // Modo single antenna - só precisa de GGA
        readyToSend = (dualReadyGGA == true);
    }
    
    if (readyToSend)
    {
        // Read BNO immediately before building NMEA to get latest values
        if (useBNO08x) {
            readBNO();
        }
        imuHandler();
        BuildNmea();
        dualReadyGGA = false;
        dualReadyRelPos = false;
    }

    //Read BNO continuously to keep buffer empty
    if((millis() - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x)
    {
      READ_BNO_TIME = millis();
      readBNO();
    }
    
    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();
    
  if (!ETH.linkUp()) 
  {
    //digitalWrite(Power_on_LED, 1);           //Commented to save ESP32 pins
    //digitalWrite(Ethernet_Active_LED, 0);    //Commented to save ESP32 pins
  }
  else
  {
    //digitalWrite(Power_on_LED, 0);           //Commented to save ESP32 pins
    //digitalWrite(Ethernet_Active_LED, 1);    //Commented to save ESP32 pins
  }
}//End Loop
//**************************************************************************

bool calcChecksum()
{
  CK_A = 0;
  CK_B = 0;

  for (int i = 2; i < 70; i++)
  {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}

//Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
//This is called before we send a command message
void calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}
