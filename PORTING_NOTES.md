# Notas de Portagem: Teensy 4.1 → ESP32 WT32-ETH01

## Resumo
Firmware originalmente desenvolvido para Teensy 4.1 foi portado para ESP32 WT32-ETH01 mantendo funcionalidade de comunicação UDP via Ethernet para o sistema AgOpenGPS.

## Alterações Principais

### 1. Platform & Build (platformio.ini)
- ✅ Adicionada biblioteca `SimpleKalmanFilter`
- ✅ Configurados pinos Ethernet do WT32-ETH01 (LAN8720)
- ✅ Definidos build flags para PHY Ethernet

### 2. Ethernet
- ✅ `NativeEthernet.h` → `ETH.h`
- ✅ `EthernetUDP` → `WiFiUDP`
- ✅ `Ethernet.begin()` → `ETH.begin()` + `ETH.config()`
- ✅ `Ethernet.linkStatus()` → `ETH.linkUp()`
- ✅ Função `EthernetStart()` reescrita para ESP32

### 3. Serial Ports (UART)
**Teensy → ESP32:**
- `Serial7` → `Serial2` (GPIO 5/RX, 17/TX) - GPS UM982
- `Serial3` → `Serial1` (GPIO 4/RX, 2/TX) - RTK Radio
- `Serial` → `Serial` (USB) - AgIO

**Configuração:**
```cpp
SerialGPS->begin(baudGPS, SERIAL_8N1, 5, 17);
SerialRTK.begin(baudRTK, SERIAL_8N1, 4, 2);
```

### 4. I2C (Wire)
- ✅ `Wire1` → `Wire`
- ✅ Pinos I2C definidos: SDA=GPIO32, SCL=GPIO33
- ✅ Ajustado em: `zADS1115.cpp`, `Autosteer.ino`
- ✅ Configuração: `Wire.begin(32, 33)`

### 5. GPIO/Pinos

#### LEDs
| Função | Teensy | ESP32 |
|--------|--------|-------|
| GGA Received | 13 | 2 (LED onboard) |
| Power On (Red) | 5 | 14 |
| Ethernet Active (Green) | 6 | 15 |
| GPS Red | 9 | 12 |
| GPS Green | 10 | 13 |
| Autosteer Standby (Red) | 11 | 32 |
| Autosteer Active (Green) | 12 | 33 |

#### Motor & Sensores
| Função | Teensy | ESP32 |
|--------|--------|-------|
| DIR1_RL_ENABLE | 4 | 25 |
| PWM1_LPWM | 2 | 26 |
| PWM2_RPWM | 3 | 27 |
| STEERSW_PIN | 32 | 34 |
| WORKSW_PIN | 34 | 35 |
| REMOTE_PIN | 37 | 36 |
| CURRENT_SENSOR | A17 | 39 |
| PRESSURE_SENSOR | A10 | 36 |

### 6. PWM (Motor Control)
- ✅ Substituído `analogWriteFrequency()` por `ledcSetup()`
- ✅ Substituído `analogWrite()` por `ledcWrite()`
- ✅ Configurados 2 canais LEDC (0 e 1)

**Configuração:**
```cpp
ledcSetup(0, 490, 8);  // Canal 0, 490Hz, 8-bit
ledcSetup(1, 490, 8);  // Canal 1, 490Hz, 8-bit
ledcAttachPin(PWM1_LPWM, 0);
ledcAttachPin(PWM2_RPWM, 1);
```

**Uso:**
```cpp
ledcWrite(0, pwmDrive);  // Escrever PWM no canal 0
```

### 7. Manipulação de Portas
- ✅ `bitSet(PORTD, 4)` → `digitalWrite(DIR1_RL_ENABLE, HIGH)`
- ✅ `bitClear(PORTD, 4)` → `digitalWrite(DIR1_RL_ENABLE, LOW)`

### 8. Timers
- ✅ `systick_millis_count` → `millis()`
- ✅ `elapsedMillis` → `uint32_t` + `millis()`

### 9. APIs Específicas Removidas
- ❌ `set_arm_clock()` - CPU clock específico do Teensy
- ❌ `addMemoryForRead()/addMemoryForWrite()` - Buffer serial do Teensy
- ❌ `F_CPU_ACTUAL` - Constante Teensy
- ❌ `#ifdef ARDUINO_TEENSY41` - Condicionais removidas

### 10. EEPROM
- ✅ Mantida biblioteca `EEPROM.h` (ESP32 emula via flash)
- ⚠️ Considere usar `Preferences.h` para storage mais robusto

## Pinout WT32-ETH01

### Ethernet (LAN8720)
- MDC: GPIO23
- MDIO: GPIO18
- PHY Power: GPIO16
- CLK Mode: GPIO0 (input)

### Disponíveis para uso
- GPIO 2, 4, 5, 12, 13, 14, 15, 17, 25, 26, 27, 32, 33, 34, 35, 36, 39

### Reservados (Ethernet/Internal)
- GPIO 0, 16, 18, 19, 21, 22, 23 (Ethernet PHY)
- GPIO 1, 3 (USB/Serial)

## Compilação

```bash
# Via PlatformIO
pio run -e wt32-eth01

# Upload
pio run -e wt32-eth01 -t upload

# Monitor serial
pio device monitor
```

## Testes Recomendados

### Hardware
1. ✅ Verificar conexão Ethernet (LED verde deve acender)
2. ⚠️ Testar comunicação Serial2 com GPS UM982
3. ⚠️ Testar comunicação Serial1 com rádio RTK
4. ⚠️ Verificar I2C: BNO08x IMU e ADS1115 ADC
5. ⚠️ Testar PWM motor (canais LEDC)
6. ⚠️ Verificar LEDs de status

### Software
1. ⚠️ Confirmar recepção UDP NTRIP (porta 2233)
2. ⚠️ Confirmar envio/recepção UDP Autosteer (porta 8888)
3. ⚠️ Verificar envio dados GPS para AgOpenGPS (porta 9999)
4. ⚠️ Testar parsing NMEA (GGA, VTG, HPR)
5. ⚠️ Validar controle PID autosteer

## Limitações Conhecidas

1. **Buffers Serial**: ESP32 usa buffers internos (128-256 bytes padrão) vs Teensy com buffers customizáveis
2. **ADC Resolution**: ESP32 ADC é 12-bit vs Teensy 4.1 16-bit (já usando ADS1115 externo, não afeta)
3. **GPIO Input Modes**: ESP32 não possui `INPUT_DISABLE`, use `INPUT` ou `INPUT_PULLUP`
4. **EEPROM Size**: ESP32 emula EEPROM em flash (512 bytes padrão), Teensy tem EEPROM real

## Próximos Passos

1. ⚠️ Compilar e resolver warnings/erros
2. ⚠️ Upload no WT32-ETH01
3. ⚠️ Testes de hardware (sensores, motor, Ethernet)
4. ⚠️ Calibração e validação em campo
5. ⚠️ Considerar migrar EEPROM para `Preferences.h`
6. ⚠️ Otimizar buffers serial se necessário

## Configuração AgOpenGPS

Manter mesma configuração de rede:
- IP Module: 192.168.137.126 (com autosteer) ou .120 (GPS only)
- Porta GPS: 5120
- Porta NTRIP: 2233
- Porta Autosteer: 8888
- Porta AOG: 9999

## Referências

- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
- [WT32-ETH01 Pinout](https://github.com/ldab/wt32-eth01)
- [ESP32 LEDC (PWM)](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [AgOpenGPS](https://github.com/farmerbriantee/AgOpenGPS)
