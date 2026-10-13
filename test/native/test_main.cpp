#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <array>
#include <atomic>
#include <cassert>
#include <iostream>
#include <string>
#include <thread>
#include "autosteer/pid.h"
#include "autosteer/autosteer_task.h"
#include "autosteer/safety.h"
#include "config/defines.h"
#include "config/pinout.h"
#include "gps/context.h"
#include "gps/gps.h"
#include "hardware/ads1115.h"
#include "hardware/motor_pwm.h"
#include "imu/rvc.h"
#include "network/pgn_protocol.h"
#include "network/rtcm_buffer.h"
#include "network/udp_tx.h"
#include "persist/preferences.h"
#include "state/pending.h"
#include "state/steer_state.h"
#include "state/shared_value.h"

const UserSettings userSettings = []() constexpr {
    UserSettings settings;
#ifdef TEST_PAOGI
    settings.makeOGI = true;
#endif
#ifdef TEST_DUAL
    settings.baseLineCheck = true;
#endif
    return settings;
}();
std::string lastNmea;
struct Guard { char field[3]{}; uint32_t canary = 0x12345678; } guard;
NMEAParser<1>* boundedParser = nullptr;
bool boundedCalled = false, boundedResult = true;
namespace udp_tx {
bool enqueue(Channel, const uint8_t* bytes, size_t count) {
    lastNmea.assign(reinterpret_cast<const char*>(bytes), count);
    return true;
}
}
std::string nmea(const std::string& body) {
    uint8_t sum = 0;
    for (char byte : body) sum ^= byte;
    char checksum[8];
    std::snprintf(checksum, sizeof(checksum), "*%02X\r\n", sum);
    return "$" + body + checksum;
}
void feed(const std::string& body) {
    for (char byte : nmea(body)) gps::parser << byte;
}

void checkPid() {
    Storage settings;
    Setup setup;
    struct Case { float error; int16_t expected; };
    const Case cases[] = {{0,0},{0.1f,11},{0.5f,18},{1,26},{2,43},{3,60},{8,60},{-0.1f,-11},{-1,-26},{-8,-60}};
    for (const auto& test : cases) assert(calcSteeringPID(test.error, settings, setup) == test.expected);
    setup.MotorDriveDirection = 1;
    assert(calcSteeringPID(1, settings, setup) == -26);
    setup.MotorDriveDirection = 0; setup.IsDanfoss = 1;
    assert(calcSteeringPID(0, settings, setup) == 128);
    assert(calcSteeringPID(8, settings, setup) == 143);
    assert(calcSteeringPID(-8, settings, setup) == 113);
}

void checkProtocol() {
    uint8_t packet[] = {0x80,0x81,0x7F,254,8,123,0,1,0x2E,0xFB,3,4,5,0};
    pgn::checksum(packet, sizeof(packet));
    GuidanceCommand command;
    assert(pgn::decodeGuidance(packet, sizeof(packet), command));
    assert(std::fabs(command.gpsSpeed - 12.3f) < 0.001f);
    assert(std::fabs(command.steerAngleSetPoint + 12.34f) < 0.001f);
    assert(command.tram == 3 && command.relay == 4 && command.relayHi == 5);
    for (size_t count = 0; count < sizeof(packet); ++count) assert(!pgn::decodeGuidance(packet, count, command));
    packet[8] ^= 1;
    assert(!pgn::decodeGuidance(packet, sizeof(packet), command));
    packet[3] = 252; packet[5] = 40; packet[6] = 60; packet[8] = 9; packet[9] = 30;
    packet[10] = 0xD4; packet[11] = 0xFE; packet[12] = 100;
    pgn::checksum(packet, sizeof(packet));
    Storage settings;
    assert(pgn::decodeSettings(packet, sizeof(packet), settings));
    assert(settings.lowPWM == 10 && settings.wasOffset == -300 && settings.AckermanFix == 1);
    packet[9] = 0; pgn::checksum(packet, sizeof(packet));
    assert(!pgn::decodeSettings(packet, sizeof(packet), settings));
    command.valid = true; command.receivedMs = UINT32_MAX - 100;
    assert(safety::commandFresh(100, command));
    assert(!safety::commandFresh(2399, command));
}

void checkNavigation() {
    feed("GNGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,1.2,0000");
    feed("GNVTG,054.7,T,034.4,M,005.5,N,010.2,K");
    feed("GNHPR,123519,90.0,-2.5,0.0,4");
    assert(gps::context.dualReadyGGA && gps::context.dualReadyRelPos);
    gps::imuHandler(); gps::BuildNmea();
    const std::string prefix = userSettings.makeOGI ? "PAOGI" : "PANDA";
    const std::string position = ",123519,4807.038,N,01131.000,E,4,08,0.9,545.4,1.2,005.5,";
    const std::string dualOnly = userSettings.makeOGI ? "90.00,-2.50,0.0000,0.0" : "900,-25,0.0000,0";
    assert(lastNmea == nmea(prefix + position + dualOnly));
    gps::imu.useBNO08x = true;
    gps::imu.yaw = 900; gps::imu.roll = -30; gps::imu.pitch = 20; gps::imu.gyroZ = 5.2;
    gps::imu.correctionHeading = 90 * DEG_TO_RAD;
    gps::imuHandler(); gps::BuildNmea();
    const std::string imuFields = (userSettings.makeOGI || userSettings.baseLineCheck)
        ? "90.0,-3.0,2.0,5.2" : "900,-30,20,52";
    assert(lastNmea == nmea(prefix + position + imuFields));
    gps::imu = ImuState{};

    NMEAParser<1> parser;
    boundedParser = &parser;
    parser.addHandler("G-GGA", [] {
        boundedCalled = true;
        boundedResult = boundedParser->getArg(0, guard.field);
    });
    for (char byte : nmea("GNGGA,oversized")) parser << byte;
    assert(boundedCalled && !boundedResult);
    assert(guard.field[0] == 0 && guard.canary == 0x12345678);
}

void checkNavigationTask() {
    gps::context.dualReadyGGA = false;
    gps::context.dualReadyRelPos = false;
    std::string bytes = nmea("GNGGA,123520,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,1.2,0000") +
        nmea("GNVTG,054.7,T,034.4,M,005.5,N,010.2,K") + nmea("GNHPR,123520,90.0,-2.5,0.0,4");
    for (uint8_t byte : bytes) fakeUartRx[2].push_back(byte);
    std::array<uint8_t, 600> correction;
    for (size_t i = 0; i < correction.size(); ++i) correction[i] = i;
    assert(rtcm::enqueue(correction.data(), correction.size()));
    for (uint8_t i = 0; i < 5; ++i) { ++fakeMillis; gps::poll(); }
    assert(fakeUartRx[2].empty() && rtcm::available() == 0);
    assert(fakeUartTx[2].size() == correction.size());
    assert(std::equal(correction.begin(), correction.end(), fakeUartTx[2].begin()));
    assert(lastNmea.find(",123520,") != std::string::npos);
}

void checkNvsCompatibility() {
    ConfigSnapshot initial;
    assert(persist::load(initial));
    assert(fakeNvs.size() == 24);
    const char* byteKeys[] = {
        "st_kp", "st_lowpwm", "st_minpwm", "st_highpwm", "sc_invwas", "sc_relayah",
        "sc_motordir", "sc_singlewas", "sc_cytron", "sc_steersw", "sc_steerbtn", "sc_shaftenc",
        "sc_pressens", "sc_currsens", "sc_pulsemax", "sc_danfoss", "sc_useyaxis", "ip_one", "ip_two", "ip_three"
    };
    for (const char* key : byteKeys) assert(fakeNvs.at(key).size() == 1);
    assert(fakeNvs.at("ident").size() == 4);
    assert(fakeNvs.at("st_kp").size() == 1 && fakeNvs.at("st_wasoff").size() == 2);
    assert(fakeNvs.at("st_senscnt").size() == 4 && fakeNvs.at("st_ackfix").size() == 4);
    initial.steerSettings.Kp = 93;
    initial.steerSettings.wasOffset = -2048;
    initial.steerSettings.steerSensorCounts = 42.5f;
    initial.steerConfig.IsUseY_Axis = 1;
    initial.networkAddress = {10,20,30};
    assert(persist::save(initial, pending::Settings | pending::Config | pending::Network));
    ConfigSnapshot restored;
    assert(persist::load(restored));
    assert(restored.steerSettings.Kp == 93 && restored.steerSettings.wasOffset == -2048);
    assert(restored.steerSettings.steerSensorCounts == 42.5f);
    assert(restored.steerConfig.IsUseY_Axis == 1 && restored.networkAddress.ipTwo == 20);
    fakeNvsFail = true;
    assert(!persist::save(initial, pending::Settings));
    fakeNvsFail = false;
}

void checkRtcm() {
    std::array<uint8_t, 1460> input;
    for (size_t i = 0; i < input.size(); ++i) input[i] = i;
    assert(rtcm::enqueue(input.data(), input.size()));
    assert(!rtcm::enqueue(input.data(), input.size()));
    assert(rtcm::droppedPackets() == 1 && rtcm::available() == input.size());
    std::array<uint8_t, 1460> output{};
    assert(rtcm::read(output.data(), 1000) == 1000);
    assert(rtcm::enqueue(input.data(), 1000));
    assert(rtcm::read(output.data() + 1000, 460) == 460);
    assert(output == input);
    assert(rtcm::read(output.data(), output.size()) == 1000);
    for (size_t i = 0; i < 1000; ++i) assert(output[i] == input[i]);
    assert(rtcm::available() == 0);
}

void checkSnapshots() {
    struct Pair { uint32_t left = 0, right = UINT32_MAX; };
    SharedValue<Pair> shared;
    std::atomic<bool> done{false};
    std::thread producer([&] {
        for (uint32_t i = 0; i < 50000; ++i) shared.publish({i, ~i});
        done = true;
    });
    while (!done.load()) {
        Pair value = shared.snapshot();
        assert(value.right == ~value.left);
    }
    producer.join();
}

void checkMotorAndAdc() {
    assert(motor::setup());
    Setup setup;
    uint32_t token = motor::epoch();
    assert(motor::drive(40, setup, token) == 40 && fakePwm[PWM1_LPWM] == 40);
    motor::stop();
    assert(motor::drive(40, setup, token) == 0 && fakePwm[PWM1_LPWM] == 0);
    setup.CytronDriver = 0;
    assert(motor::drive(-30, setup, motor::epoch()) == 30);
    assert(fakePwm[PWM1_LPWM] == 0 && fakePwm[PWM2_RPWM] == 30);
    motor::stop();
    ADS1115_lite adc;
    int16_t result = 0;
    assert(adc.getConversion(result) && result == 13610);
    Wire.count = 1;
    assert(!adc.getConversion(result));
    Wire.count = 2; Wire.error = 2;
    assert(!adc.getConversion(result) && !adc.triggerConversion());
    Wire.error = 0;
}

void checkControlAndPersistence() {
    config::publish(ConfigSnapshot{});
    assert(autosteer::setup(true));
    GuidanceCommand command;
    command.valid = true; command.gpsSpeed = 5; command.guidanceStatus = 1;
    command.steerAngleSetPoint = 1; command.receivedMs = 25;
    state::publishGuidance(command); fakeMillis = 25; autosteer::poll();
    command.receivedMs = 50; state::publishGuidance(command); fakeMillis = 50; autosteer::poll();
    assert(state::steerSnapshot().pwmDisplay == 26);
    assert(fakePwm[PWM1_LPWM] == 26);
    fakeMillis = 2550; safety::poll();
    assert(fakePwm[PWM1_LPWM] == 0);
    Storage settings; settings.Kp = 80;
    pending::stageSettings(settings); fakeMillis = 2575; autosteer::poll();
    assert(pending::maintenanceActive() && fakePwm[PWM1_LPWM] == 0);
    pending::Update update;
    assert(pending::takePersistence(update));
    assert(update.mask == pending::Settings && update.values.steerSettings.Kp == 80);
    settings.Kp = 90; pending::stageSettings(settings);
    assert(!pending::take(update));
    pending::finishPersistence();
    assert(pending::take(update) && update.values.steerSettings.Kp == 90);
}

void checkRvc() {
    ::imu::setup();
    ImuState sample;
    Setup setup;
    auto feedPacket = [&](uint8_t index, int16_t yaw, bool corrupt = false) {
        uint8_t bytes[19] = {0xAA,0xAA,index};
        bytes[3] = yaw; bytes[4] = yaw >> 8;
        bytes[5] = 0x2C; bytes[6] = 1; // pitch = 3 grados.
        bytes[7] = 0x38; bytes[8] = 0xFF; // roll = -2 grados.
        for (uint8_t i = 2; i < 18; ++i) bytes[18] += bytes[i];
        if (corrupt) bytes[18] ^= 1;
        for (uint8_t byte : bytes) fakeUartRx[1].push_back(byte);
        ::imu::poll(sample, setup);
    };
    feedPacket(1, 17900); feedPacket(2, 17900); feedPacket(3, -17900);
    assert(sample.useBNO08x && sample.rvcPacketCount == 3);
    assert(sample.roll == -30 && sample.pitch == -20);
    assert(sample.gyroZ == 40); // Diferencia de 2 grados cruzando +/-180.
    feedPacket(4, -17800, true);
    assert(sample.rvcPacketCount == 3);
}

int main() {
    gps::setup();
    checkPid(); checkProtocol(); checkNavigation(); checkRtcm(); checkSnapshots();
    checkNavigationTask();
    checkMotorAndAdc(); checkControlAndPersistence(); checkRvc(); checkNvsCompatibility();
    std::cout << "PASS: PID, PGNs, NMEA/PANDA, RTCM, snapshots, motor, ADS1115, controle, staging, RVC e NVS\n";
}
