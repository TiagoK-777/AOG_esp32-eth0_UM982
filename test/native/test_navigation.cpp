// SPDX-License-Identifier: GPL-3.0-or-later
// Regressões do código de produção; UART, relógio e saída UDP simulados.
#include <Arduino.h>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "config/settings.h"
#include "gps/context.h"
#include "gps/gps.h"
#include "network/udp_tx.h"

namespace {
int option(const char* name) {
    const char* value = std::getenv(name);
    return value ? std::atoi(value) : 0;
}
}

// Cada processo recebe configurações imutáveis, como no firmware real.
const UserSettings userSettings = [] {
    UserSettings value;
    value.makeOGI = option("NAV_OGI");
    value.baseLineCheck = option("NAV_MODE") != 0;
    value.headingcorr = option("NAV_MOUNT");
    value.useYawRate = option("NAV_YAW_RATE");
    value.invertRoll = option("NAV_INVERT_ROLL");
    return value;
}();

namespace {
std::vector<std::string> messages;
const std::string gga = "GNGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,1.2,0000";
const std::string position = ",123519,4807.038,N,01131.000,E,4,08,0.9,545.4,1.2,005.5,";
bool single() { return option("NAV_MODE") == 0; }
bool fused() { return option("NAV_MODE") == 2; }
bool lateral() { return userSettings.headingcorr == 900; }

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, const std::string& message, double tolerance = 1e-6) {
    require(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance,
        message + ": esperado " + std::to_string(expected) + ", recebido " + std::to_string(actual));
}
std::string sentence(const std::string& body) {
    uint8_t sum = 0;
    for (char byte : body) sum ^= byte;
    char tail[8];
    std::snprintf(tail, sizeof(tail), "*%02X\r\n", sum);
    return "$" + body + tail;
}
void feed(const std::string& body) {
    for (char byte : sentence(body)) gps::parser << byte;
}
void hpr(double heading, double tilt, int quality) {
    char body[96];
    std::snprintf(body, sizeof(body), "GNHPR,123519,%.4f,%.4f,0.0,%d", heading, tilt, quality);
    feed(body);
}
void reset() {
    gps::context = gps::Context{};
    gps::imu = ImuState{};
    gps::parser = NMEAParser<4>{};
    gps::rollFilter = SimpleKalmanFilter(1, 1, 0.01f);
    gps::headingFilter = SimpleKalmanFilter(1, 1, 0.01f);
    for (auto& rx : fakeUartRx) rx.clear();
    for (auto& tx : fakeUartTx) tx.clear();
    fakeMillis = 100;
    config::publish(ConfigSnapshot{});
    messages.clear();
    gps::setup();
}
void setImu() {
    gps::imu.useBNO08x = true;
    gps::imu.yaw = 1234;
    gps::imu.roll = -30;
    gps::imu.pitch = 20;
    gps::imu.gyroZ = -5.2;
    gps::imu.correctionHeading = 20 * DEG_TO_RAD;
}
void sendUart(const std::string& bytes) {
    for (uint8_t byte : bytes) fakeUartRx[2].push_back(byte);
    const size_t passes = (bytes.size() + 63) / 64;
    for (size_t i = 0; i < passes; ++i) { ++fakeMillis; gps::poll(); }
    require(fakeUartRx[2].empty(), "orçamento de leitura GNSS");
}
void checkMessage(const std::string& fields) {
    require(!messages.empty(), "faltou mensagem UDP");
    const std::string expected = sentence((userSettings.makeOGI ? "PAOGI" : "PANDA") + position + fields);
    require(messages.back() == expected, "sentença/unidades/checksum: esperado " + expected + "recebido " + messages.back());
}
void checkFormatter() {
    feed(gga);
    feed("GNVTG,054.7,T,034.4,M,005.5,N,010.2,K");
    if (single() || fused()) setImu();
    gps::context.heading = 87.5;
    gps::context.rollDual = -6.5;
    gps::context.pitchDual = 4.5;
    gps::context.imuGPS_Offset = 10 * DEG_TO_RAD;
    gps::context.rollDeltaSmooth = 1.5;
    gps::context.pitchDeltaSmooth = -1.5;
    gps::imuHandler();
    gps::BuildNmea();
    std::string fields;
    if (single()) fields = userSettings.makeOGI ? "123.4,-3.0,2.0," : "1234,-30,20,";
    else if (fused()) {
        if (userSettings.makeOGI) fields = lateral() ? "30.0,-1.5,2.0," : "30.0,-3.0,0.5,";
        else fields = lateral() ? "300,-15,20," : "300,-30,5,";
    } else fields = userSettings.makeOGI ? "87.50,-6.50,4.5000," : "875,-65,45,";
    fields += (single() || fused()) && userSettings.useYawRate
        ? (userSettings.makeOGI ? "-5.2" : "-52") : (userSettings.makeOGI ? "0.0" : "0");
    checkMessage(fields);
}
void checkEpochs() {
    if (single() || fused()) setImu();
    sendUart(sentence("GNVTG,054.7,T,034.4,M,005.5,N,010.2,K"));
    sendUart(sentence(gga));
    require(messages.size() == (single() ? 1u : 0u), "gate GGA sem HPR");
    sendUart(sentence("GNHPR,123519,80.0,-6.0,0.0,4"));
    require(messages.size() == 1, "época GGA/HPR deve sair uma vez");
    require(!gps::context.dualReadyGGA, "GGA consumido");
    for (int i = 0; i < 10; ++i) gps::poll();
    require(messages.size() == 1, "poll sem dados não pode repetir mensagem");
    sendUart(sentence(gga));
    require(messages.size() == (single() ? 2u : 1u), "HPR consumido não pode liberar nova época dual");
    sendUart(sentence("GNHPR,123519,81.0,-5.0,0.0,4"));
    require(messages.size() == 2, "segunda época deve recuperar");
    reset();
    if (single() || fused()) setImu();
    sendUart(sentence("GNHPR,123519,80.0,-6.0,0.0,4"));
    require(messages.empty(), "HPR sem GGA não envia");
    sendUart(sentence(gga));
    require(messages.size() == 1, "ordem HPR antes de GGA");
}
void checkFullPrecisionHpr() {
    if (single() || fused()) setImu();
    struct Case { const char* heading; const char* tilt; };
    const Case cases[] = {
        {"320.9610", "-66.1712"}, {"000.0000", "-00.0873"},
        {"359.9999", "090.0000"}, {"100.0000", "-090.0000"}
    };
    // Vinte épocas a cada 100 ms, alternando a ordem GGA/HPR na UART.
    for (unsigned epoch = 0; epoch < 20; ++epoch) {
        const Case& sample = cases[epoch % 4];
        char fixTime[12];
        std::snprintf(fixTime, sizeof(fixTime), "1235%02u.%u0", epoch / 10, epoch % 10);
        std::string epochGga = gga;
        epochGga.replace(6, 6, fixTime);
        const std::string epochHpr = std::string("GNHPR,") + fixTime + "," + sample.heading + "," +
            sample.tilt + ",000.0000,4,47,0.00,0999";
        fakeMillis = 100 + epoch * 100;
        if (epoch % 2 == 0) {
            sendUart(sentence(epochGga));
            sendUart(sentence(epochHpr));
        } else {
            sendUart(sentence(epochHpr));
            sendUart(sentence(epochGga));
        }
        require(messages.size() == epoch + 1, "uma saída por época com HPR de quatro casas");
        require(messages.back().find(std::string(",") + fixTime + ",") != std::string::npos,
            "saída deve corresponder ao GGA da época");
        require(std::string(gps::context.umHeading) == sample.heading, "heading HPR sem truncamento");
        require(std::string(gps::context.umRoll) == sample.tilt, "inclinação HPR sem truncamento");
        near(gps::context.heading, std::atof(sample.heading), "heading HPR de quatro casas");
        near(gps::context.rollDual, lateral() ? std::atof(sample.tilt) : 0, "roll HPR de quatro casas");
        near(gps::context.pitchDual, lateral() ? 0 : std::atof(sample.tilt), "pitch HPR de quatro casas");
    }
}
void checkMount() {
    if (single() || fused()) setImu();
    hpr(80, -6, 4);
    near(gps::context.rollDual, lateral() ? -6 : 0, "inclinação dual no roll");
    near(gps::context.pitchDual, lateral() ? 0 : -6, "inclinação dual no pitch");
    gps::imuHandler();
    const double scale = userSettings.makeOGI ? 1 : 10;
    if (fused()) {
        near(std::atof(gps::context.imuRoll), (lateral() ? -3.9 : -3) * scale, "roll fundido", 0.11);
        near(std::atof(gps::context.imuPitch), (lateral() ? 2 : -0.4) * scale, "pitch fundido", 0.11);
    } else if (!single()) {
        near(std::atof(gps::context.imuRoll), (lateral() ? -6 : 0) * scale, "roll dual direto");
        near(std::atof(gps::context.imuPitch), (lateral() ? 0 : -6) * scale, "pitch dual direto");
    }
}
void checkQuality() {
    if (single() || fused()) setImu();
    for (int quality : {0, 1, 2, 3, 4, 5}) {
        gps::context.imuGPS_Offset = 0;
        gps::context.rollDeltaSmooth = 0;
        gps::context.pitchDeltaSmooth = 0;
        hpr(80, -6, quality);
        const bool update = fused() && quality >= 4;
        near(gps::context.imuGPS_Offset, update ? 12 * DEG_TO_RAD : 0, "offset/qualidade " + std::to_string(quality));
        near(gps::context.rollDeltaSmooth, update && lateral() ? -0.9 : 0, "correção roll/qualidade");
        near(gps::context.pitchDeltaSmooth, update && !lateral() ? -2.4 : 0, "correção pitch/qualidade");
    }
    // GGA de qualidade alta não autoriza corrigir com HPR de qualidade baixa.
    feed(gga);
    const double previous = gps::context.imuGPS_Offset;
    hpr(140, 12, 1);
    near(gps::context.imuGPS_Offset, previous, "qualidade usada deve ser a do HPR");
}
void checkHoldoverAndConvergence() {
    if (!fused()) return;
    setImu();
    for (int i = 0; i < 120; ++i) hpr(80, -6, 4);
    near(gps::context.imuGPS_Offset * RAD_TO_DEG, 60, "convergência heading", 0.001);
    near(lateral() ? gps::context.rollDeltaSmooth : gps::context.pitchDeltaSmooth,
        lateral() ? -3 : -8, "convergência da inclinação", 0.001);
    const double offset = gps::context.imuGPS_Offset;
    const double roll = gps::context.rollDeltaSmooth, pitch = gps::context.pitchDeltaSmooth;
    gps::imu.correctionHeading = 30 * DEG_TO_RAD;
    hpr(200, 15, 3);
    near(gps::context.imuGPS_Offset, offset, "sinal fraco mantém offset");
    near(gps::context.rollDeltaSmooth, roll, "sinal fraco mantém correção roll");
    near(gps::context.pitchDeltaSmooth, pitch, "sinal fraco mantém correção pitch");
    feed(gga);
    gps::poll();
    require(messages.size() == 1, "sinal fraco continua liberando época com IMU");
    near(std::atof(gps::context.imuHeading), userSettings.makeOGI ? 90 : 900, "holdover acompanha IMU", 0.11);
    hpr(100, -4, 4);
    require(gps::context.imuGPS_Offset > offset, "sinal bom volta a corrigir offset");
}
void checkCircularHeading() {
    if (!fused()) return;
    struct Case { double imu, dual, correction; };
    const Case cases[] = {{359,1,0.2},{1,359,-0.2},{120,20,-20},{20,120,20},{5,355,-1},{-20,340,0},{20,25,0.5}};
    setImu();
    for (const auto& c : cases) {
        gps::imu.correctionHeading = c.imu * DEG_TO_RAD;
        gps::context.imuGPS_Offset = 0;
        hpr(c.dual, 0, 4);
        near(gps::context.imuGPS_Offset * RAD_TO_DEG, c.correction, "menor caminho circular");
    }
    for (double degrees : {-720.0, -360.0, 0.0, 360.0, 720.0}) {
        gps::imu.correctionHeading = degrees * DEG_TO_RAD;
        gps::context.imuGPS_Offset = 0;
        gps::fuseIMU();
        near(gps::context.imuCorrected, 0, "heading normalizado em [0,360)");
    }
}
void checkBadChecksum() {
    if (single() || fused()) setImu();
    std::string bad = sentence("GNHPR,123519,80.0,-6.0,0.0,4");
    bad[bad.find('*') + 1] = 'Z';
    sendUart(bad);
    require(!gps::context.dualReadyRelPos, "checksum inválido não libera HPR");
    near(gps::context.imuGPS_Offset, 0, "checksum inválido não corrige");
    sendUart(sentence("GNHPR,123519,80.0,-6.0,0.0,4"));
    require(gps::context.dualReadyRelPos, "parser recupera após checksum inválido");
}
void checkInvalidHpr() {
    if (single() || fused()) setImu();
    const char* bad[] = {
        "GNHPR,123519,80,-6,0", "GNHPR,123519,,-6,0,4", "GNHPR,123519,no,-6,0,4",
        "GNHPR,123519,nan,-6,0,4", "GNHPR,123519,inf,-6,0,4", "GNHPR,123519,361,-6,0,4",
        "GNHPR,123519,80,nan,0,4", "GNHPR,123519,80,181,0,4", "GNHPR,123519,80,-6,0,4x",
        "GNHPR,123519,123456789,-6,0,4", "GNHPR,123519,80,,0,4",
        "GNHPR,123519,0000000000000001,-6,0,4", "GNHPR,123519,80,-0000000000000001,0,4"
    };
    for (const char* body : bad) {
        hpr(80, -6, 4);
        const double offset = gps::context.imuGPS_Offset;
        const double heading = gps::context.heading;
        const double roll = gps::context.rollDeltaSmooth, pitch = gps::context.pitchDeltaSmooth;
        feed(body);
        require(!gps::context.dualReadyRelPos, std::string("HPR inválido deve cancelar prontidão: ") + body);
        near(gps::context.imuGPS_Offset, offset, "HPR inválido preserva offset");
        near(gps::context.heading, heading, "HPR inválido preserva heading");
        near(gps::context.rollDeltaSmooth, roll, "HPR inválido preserva roll");
        near(gps::context.pitchDeltaSmooth, pitch, "HPR inválido preserva pitch");
    }
    hpr(80, -6, 4);
    require(gps::context.dualReadyRelPos, "recuperação após HPR inválido");
}
void checkRvcToOutput() {
    if (!single() && !fused()) return;
    uint8_t index = 0;
    for (bool yAxis : {false, true}) {
        ConfigSnapshot config;
        config.steerConfig.IsUseY_Axis = yAxis;
        config::publish(config);
        uint8_t packet[19] = {0xAA, 0xAA, ++index};
        const int16_t values[] = {-12340, 300, -200};
        for (size_t i = 0; i < 3; ++i) {
            packet[3 + i * 2] = values[i] & 0xFF;
            packet[4 + i * 2] = static_cast<uint16_t>(values[i]) >> 8;
        }
        for (size_t i = 2; i < 18; ++i) packet[18] += packet[i];
        for (uint8_t byte : packet) fakeUartRx[1].push_back(byte);
        gps::poll();
        require(gps::imu.useBNO08x, "RVC válido detecta IMU");
        const double sign = userSettings.invertRoll ? -1 : 1;
        near(gps::imu.roll, (yAxis ? -20 : 30) * sign, "mapeamento e inversão RVC roll");
        near(gps::imu.pitch, yAxis ? 30 : -20, "mapeamento RVC pitch");
        near(gps::imu.yaw, 2366, "yaw RVC negativo convertido");
        near(gps::imu.correctionHeading * RAD_TO_DEG, 123.4, "heading RVC para fusão");
        if (fused()) hpr(123.4, -6, 4);
        feed(gga);
        const size_t before = messages.size();
        gps::poll();
        require(messages.size() == before + 1, "RVC até UDP");
        const double heading = std::atof(gps::context.imuHeading);
        near(heading, fused() ? (userSettings.makeOGI ? 123.4 : 1234)
            : (userSettings.makeOGI ? 236.6 : 2366), "heading final RVC", 0.11);
    }
}
}

namespace udp_tx {
bool enqueue(Channel channel, const uint8_t* bytes, size_t count) {
    require(channel == Channel::GPS, "canal UDP GPS");
    messages.emplace_back(reinterpret_cast<const char*>(bytes), count);
    return true;
}
}

int main() {
    struct Test { const char* name; std::function<void()> run; };
    const Test tests[] = {
        {"formato/unidades/checksum", checkFormatter}, {"épocas/ordem/sem repetição", checkEpochs},
        {"HPR quatro casas/UART/10 Hz", checkFullPrecisionHpr},
        {"roll/pitch conforme montagem", checkMount}, {"qualidade HPR 0..5", checkQuality},
        {"convergência/perda/recuperação", checkHoldoverAndConvergence}, {"heading circular", checkCircularHeading},
        {"checksum/recuperação", checkBadChecksum}, {"HPR inválido/atômico", checkInvalidHpr},
        {"RVC/eixos/inversão/UDP", checkRvcToOutput}
    };
    int failures = 0;
    for (const auto& test : tests) {
        reset();
        try { test.run(); }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
        }
    }
    if (!failures) std::cout << "PASS: navegação (10 grupos)\n";
    return failures ? 1 : 0;
}
