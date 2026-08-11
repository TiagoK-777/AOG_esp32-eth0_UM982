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
- `Serial7` → `Serial2` (GPIO 5/RX, 17/TX) - GPS UM982 @ 460800
- `Serial3` → `Serial1` (GPIO 2/RX) - **BNO085 UART-RVC** (RX only, TX = -1)
- `Serial` → `Serial` (USB) - AgIO

> ⚠️ **Rádio RTK desabilitado:** no port atual o passthrough RTK Radio→GPS foi desativado
> porque os pinos GPIO 4/2 do `Serial1` colidem com o **BNO085 UART-RVC** (usa GPIO2).
> A correção RTK entra somente por **UDP** (porta 2233) e é repassada ao GPS via
> `SerialGPS->write()`.

**Configuração (em `setup()` de `AOG_Esp32_UM982.cpp`):**
```cpp
SerialGPS->begin(baudGPS, SERIAL_8N1, 5, 17);   // GPS UM982 (RX=5, TX=17) @ 460800
//SerialRTK.begin(baudRTK, SERIAL_8N1, 4, 2);   // DESABILITADO (colide com BNO085 RVC)
SerialRVC.begin(115200, SERIAL_8N1, 2, -1);     // BNO085 UART-RVC (RX=GPIO2, TX não usado)
```

### 4. I2C (Wire)
- ✅ `Wire1` → `Wire`
- ✅ Pinos I2C definidos: **SDA=GPIO33, SCL=GPIO32** (WT32-ETH01)
- ✅ Ajustado em: `Autosteer.cpp` (`Wire.begin(33, 32)`) e `AOG_Esp32_UM982.cpp` (`#define ImuWire Wire`)
- ✅ Configuração: `Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32`
- Nota: apenas o **ADS1115** (ADC) usa a I2C — o BNO085 foi migrado para UART-RVC (GPIO2)

### 5. GPIO/Pinos

#### LEDs
> ⚠️ **Todos os LEDs estão DESABILITADOS** no firmware atual (definições comentadas em
> `GlobalVariables.h` para liberar GPIOs). A tabela abaixo lista o mapeamento original:

| Função | Teensy | ESP32 (desabilitado) |
|--------|--------|-------|
| GGA Received | 13 | 2 — ⚠️ USADO pelo BNO085 RVC |
| Power On (Red) | 5 | 14 |
| Ethernet Active (Green) | 6 | 15 |
| GPS Red | 9 | 12 |
| GPS Green | 10 | 13 |
| Autosteer Standby (Red) | 11 | 32 |
| Autosteer Active (Green) | 12 | 33 |

> ⚠️ **Não reativar LEDs nestes pinos:** GPIO2 pertence ao `SerialRVC` (BNO085 UART-RVC),
> GPIO32/33 à **I2C** (ADS1115) e GPIO14 é o DIR do motor.

#### Motor & Sensores
> ⚠️ Atualizado conforme os valores reais em `GlobalVariables.h`:

| Função | Teensy | ESP32 |
|--------|--------|-------|
| DIR1_RL_ENABLE | 4 | **14** |
| PWM1_LPWM | 2 | **4** |
| PWM2_RPWM | 3 | **12** |
| STEERSW_PIN | 32 | **36** (Input Only, pull-up ext. 10k) |
| WORKSW_PIN | 34 | **15** |
| REMOTE_PIN | 37 | **39** (Input Only, pull-up ext. 10k) |
| CURRENT_SENSOR | A17 | **35** (ADC1) |
| PRESSURE_SENSOR | A10 | — (Removido, não usado) |

### 6. PWM (Motor Control)
- ✅ Substituídos `analogWriteFrequency()`/`analogWrite()` do Teensy pela API LEDC do ESP32
- ✅ Usa a **API nova (Arduino-ESP32 Core 3.x)**: `ledcAttach(pin, freq, res)` + `ledcWrite(pin, duty)`
  (a API antiga `ledcSetup()`/`ledcAttachPin()`/canal foi removida no Core 3.x)
- ✅ Canais LEDC são atribuídos automaticamente por `ledcAttach()`

**Configuração (em `autosteerSetup()` de `Autosteer.cpp`):**
```cpp
// PWM_Frequency: 0 = 490Hz (padrão), 1 = 10kHz, 2 = 15kHz
ledcAttach(PWM1_LPWM, 490, 8);   // Pin-based, canal atribuído automaticamente
ledcAttach(PWM2_RPWM, 490, 8);
ledcWrite(PWM1_LPWM, 0);         // Garante motor desligado no boot
ledcWrite(PWM2_RPWM, 0);
```

**Uso (em `motorDrive()` de `AutosteerPID.cpp`):**
```cpp
ledcWrite(PWM1_LPWM, pwmDrive);  // Escrita por PINO (não por canal)
ledcWrite(PWM2_RPWM, pwmDrive);
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

### 10. Storage Persistente
- ✅ Migrado de `EEPROM.h` (deprecated) para `Preferences.h` (NVS nativo)
- NVS: chave-valor tipado, thread-safe, sem risco de corrupção em operação prolongada

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

### Upload via ESP Flash Download Tool

1. Baixar [Flash Download Tools](https://www.espressif.com/en/support/download/other-tools)
2. Selecionar **ESP32** e **Develop**
3. Carregar binários de `.pio\build\wt32-eth01\`:
   - `bootloader.bin` @ `0x1000`
   - `partitions.bin` @ `0x8000`
   - `firmware.bin` @ `0x10000`
4. Config: **SPI 40MHz, DIO**
5. Conectar via USB e clicar **START**

## Testes Recomendados

### Hardware
1. ✅ Verificar conexão Ethernet (link no módulo WT32-ETH01)
2. ✅ Testar comunicação Serial2 com GPS UM982 (@ 460800)
3. ✅  Testar Serial1 com BNO085 UART-RVC (GPIO2, 115200) — rádio RTK desabilitado
4. ✅ Verificar I2C: ADS1115 ADC (SDA=33, SCL=32)
5. ✅ Testar PWM motor (ledcWrite para PWM1_LPWM / PWM2_RPWM)
6. ⚠️ LEDs de status: nenhum — desabilitados para liberar pinos

### Software
1. ✅ Confirmar recepção UDP NTRIP (porta 2233)
2. ✅ Confirmar envio/recepção UDP Autosteer (porta 8888)
3. ✅ Verificar envio dados GPS para AgOpenGPS (porta 9999)
4. ✅ Testar parsing NMEA (GGA, VTG, HPR)
5. ✅ Validar controle PID autosteer

## Limitações Conhecidas

1. **Buffers Serial**: ESP32 usa buffers internos (128-256 bytes padrão) vs Teensy com buffers customizáveis
2. **ADC Resolution**: ESP32 ADC é 12-bit vs Teensy 4.1 16-bit (já usando ADS1115 externo, não afeta)
3. **GPIO Input Modes**: ESP32 não possui `INPUT_DISABLE`, use `INPUT` ou `INPUT_PULLUP`
4. **Persistent Storage**: `Preferences.h` (NVS) — campos tipados chave-valor, thread-safe (vs Teensy com EEPROM real)

## Próximos Passos

1. ✅ Compilar e resolver warnings/erros
2. ✅ Upload no WT32-ETH01
3. ✅ Testes de hardware (sensores, motor, Ethernet)
4. ✅ Calibração e validação em campo
5. ✅ Migrado EEPROM para `Preferences.h` (NVS)
6. ✅ Otimizar buffers serial se necessário

## Configuração AgOpenGPS

Manter mesma configuração de rede:
- IP Module: 192.168.137.126 (com autosteer) ou .120 (GPS only)
- Porta GPS: 5120
- Porta NTRIP: 2233
- Porta Autosteer: 8888
- Porta AOG: 9999

## Uso Rápido

1. **Conectar hardware:**
   - GPS UM982 → `Serial2` (RX=5, TX=17) @ 460800
   - BNO085 → `Serial1` UART-RVC (RX=GPIO2, sem TX) @ 115200
   - ADS1115 → I2C (SDA=33, SCL=32, endereço 0x48)
   - Motor → GPIO14 (DIR), GPIO4 (PWM1), GPIO12 (PWM2)
   - Rádio RTK → desabilitado (correção via UDP, porta 2233)
   - Cabo Ethernet (LAN8720)

2. **Configurar AgOpenGPS:** Settings → GPS → UDP → IP `192.168.137.126`, porta `9999`
3. **Configurar NTRIP** em AgOpenGPS (envia para porta 2233)
4. **Serial monitor** (115200) deve mostrar:
   ```text
   Ethernet status OK
   IP set Manually: 192.168.137.126
   ```

## Solução de Problemas

- **Ethernet não conecta:** verificar cabo RJ45, `ping 192.168.137.126`, PC na rede 192.168.137.x
- **GPS sem correção RTK:** porta 2233 liberada no firewall; AgOpenGPS enviando NTRIP
- **Motor não responde:** conferir GPIO14 (DIR), GPIO4 (PWM1), GPIO12 (PWM2); o monitor mostra `pwmDisplay`
- **BNO085 sem resposta:** verificar fiação UART-RVC no GPIO2 @ 115200; firmware loga `No RVC packets detected!`
- **ADS1115 não encontrado:** I2C SDA=33 / SCL=32, endereço 0x48; firmware loga `ADC Connecton FAILED!`
- **Resets constantes:** fonte insuficiente (mín. 1A @ 5V); desconectar periféricos e testar isoladamente

## Referências

- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
- [WT32-ETH01 Pinout](https://github.com/ldab/wt32-eth01)
- [ESP32 LEDC (PWM)](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [AgOpenGPS](https://github.com/farmerbriantee/AgOpenGPS)
