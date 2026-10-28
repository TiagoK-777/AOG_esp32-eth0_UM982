#pragma once
// Estado privado ao domínio GPS; somente a task de navegação o acessa.
#include "gps/nmea_parser.h"
#include "state/imu_state.h"
#include <SimpleKalmanFilter.h>
namespace gps {
struct Context {
    char fixTime[12]{}, latitude[15]{}, latNS[3]{}, longitude[15]{}, lonEW[3]{};
    char fixQuality[2]{}, numSats[4]{}, HDOP[5]{}, altitude[12]{}, ageDGPS[10]{};
    char vtgHeading[12]{}, speedKnots[10]{}, umHeading[8]{}, umRoll[8]{};
    char imuHeading[10]{}, imuRoll[10]{}, imuPitch[10]{}, imuYawRate[10]{};
    int solQuality = 0;
    double heading = 0, rollDual = 0, pitchDual = 0;
    double gpsHeading = 0, gyroDelta = 0, imuGPS_Offset = 0, imuCorrected = 0;
    double rollDelta = 0, rollDeltaSmooth = 0;
    double pitchDelta = 0, pitchDeltaSmooth = 0;
    uint32_t gpsReadyTime = 0;
    bool dualReadyGGA = false, dualReadyRelPos = false, GGA_Available = false, blink = false;
};
extern Context context;
extern ImuState imu;
extern NMEAParser<4> parser;
extern SimpleKalmanFilter rollFilter, headingFilter;
void errorHandler();
void GGA_Handler();
void VTG_Handler();
void HPR_Handler();
void imuHandler();
void imuDualDelta();
void fuseIMU();
void BuildNmea();
}
