# Instruções para Agentes de IA - Firmware AgOpenGPS ESP32

## Visão Geral do Projeto

Este é um firmware de **autosteer GPS para agricultura de precisão** (AgOpenGPS), portado de **Teensy 4.1 para ESP32 WT32-ETH01**. O sistema processa dados GNSS RTK (UM982), controla direção automática via PWM, e comunica via **UDP sobre Ethernet** com o software AgOpenGPS.

**Arquitetura:** Firmware bare-metal Arduino/PlatformIO com loop principal que gerencia:
- Recepção NMEA (GGA/VTG/HPR) do GPS UM982
- Controle PID de autosteer baseado em ângulo de direção
- Comunicação UDP bidirecional com AgOpenGPS (portas 5120/8888/2233)
- IMU BNO085 via I2C ou UART-RVC para fusão de dados
- ADC ADS1115 para sensores de WAS (Wheel Angle Sensor)

**Regras de Código:** Podem ser encontrados em:
- [.github\instructions\firmware.instructions.md](.github\instructions\firmware.instructions.md)

**Código Base (teensy) para o port:**
- [.teensy_firmware](.teensy_firmware)
- Sempre compare com o código original para garantir compatibilidade. E quando tiver algum bug relatado nesse port.

## Estrutura do Código

### Arquitetura Modular

**Arquivo principal:** [src/AOG_Esp32_UM982.cpp](src/AOG_Esp32_UM982.cpp)
- `setup()`: Inicializa Ethernet, Serial, I2C, timers
- `loop()`: Processa NMEA, UDP, autosteer, constrói mensagens PANDA/PAOGI

**Módulos segregados:**
- [src/Autosteer.cpp](src/Autosteer.cpp) - Controle PID, PWM motor, leitura WAS/ADS1115
- [src/AutosteerPID.cpp](src/AutosteerPID.cpp) - Algoritmo PID de direção
- [src/zHandlers.cpp](src/zHandlers.cpp) - Parsers NMEA (GGA/VTG/HPR), construção mensagens
- [src/zEthernet.cpp](src/zEthernet.cpp) - Inicialização Ethernet, configuração IP estática
- [src/zUdpNtrip.cpp](src/zUdpNtrip.cpp) - Passthrough NTRIP para GPS
- [src/zADS1115.cpp/.h](src/zADS1115.cpp) - Driver ADC 16-bit para sensores analógicos

**Variáveis globais:** [src/GlobalVariables.h](src/GlobalVariables.h)
- Define structs `ConfigIP`, `Storage`, `Setup`
- Declara `extern` para compartilhamento entre arquivos
- **Regra:** Definir variáveis em `.cpp`, declarar `extern` no `.h`

### Padrão de Parser NMEA

[src/zNMEAParser.h](src/zNMEAParser.h) - Parser state-machine customizado:
```cpp
if (parser.decode(incomingChar)) {
  if (strcmp(parser.thisSentence, "GGA") == 0) GGA_Handler();
  if (strcmp(parser.thisSentence, "VTG") == 0) VTG_Handler();
  if (strcmp(parser.thisSentence, "HPR") == 0) HPR_Handler();
}
```

**Importante:** UM982 envia `$GNGGA`, `$GNVTG`, `$GNHPR` (não `$GPGGA`). Código ignora prefixo `$G*`.

## Configurações de Build e Deploy

### PlatformIO Build

[platformio.ini](platformio.ini) configurado para **WT32-ETH01**:
```ini
[env:wt32-eth01]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = denyssene/SimpleKalmanFilter@^0.1.0
build_flags = 
    -D ETH_PHY_TYPE=ETH_PHY_LAN8720
    -D ETH_PHY_ADDR=1
    -D ETH_PHY_MDC=23
    -D ETH_PHY_MDIO=18
    -D ETH_PHY_POWER=16
    -D ETH_CLK_MODE=ETH_CLOCK_GPIO0_IN
```

**Comandos essenciais:**
```bash
pio run -e wt32-eth01              # Compilar
pio run -e wt32-eth01 -t upload    # Upload via USB
pio device monitor                 # Monitor serial 115200
```

### Mapeamento de Hardware ESP32

**Referência crítica:** [PORTING_NOTES.md](PORTING_NOTES.md) e [README_ESP32.md](README_ESP32.md)

**Serial/UART:**
- `Serial2` (GPIO5/RX, GPIO17/TX) @ 460800 → GPS UM982
- `Serial1` (-1/RX, GPIO2/TX) @ 115200 → BNO085 UART-RVC
- `Serial` (USB) @ 115200 → Debug/AgIO (opcional)

**I2C (Wire):**
- SDA=GPIO33, SCL=GPIO32 → ADS1115 ADC
- Inicializado em `autosteerSetup()`: `Wire.begin(33, 32)`

**PWM Motor (LEDC):**
- GPIO14 → DIR1_RL_ENABLE
- GPIO4 → PWM1_LPWM (LEDC canal 0)
- GPIO12 → PWM2_RPWM (LEDC canal 1)

**Switches/Sensores:**
- GPIO36 → STEERSW_PIN (Input Only, pull-up 10k)
- GPIO39 → REMOTE_PIN (Input Only)
- GPIO35 → CURRENT_SENSOR_PIN (ADC1_CH7)

## Convenções e Padrões

### Porting Teensy → ESP32

**SEMPRE consultar** [PORTING_NOTES.md](PORTING_NOTES.md) antes de modificar:
- ❌ `NativeEthernet.h` → ✅ `ETH.h` (ESP32 Ethernet)
- ❌ `EthernetUDP` → ✅ `WiFiUDP` (UDP class)
- ❌ `Ethernet.begin()` → ✅ `ETH.begin()` + `ETH.config()`
- ❌ `Wire1` → ✅ `Wire` (apenas um I2C disponível)
- ❌ `analogWrite()` → ✅ `ledcWrite()` (LEDC PWM)

### Controle de Fluxo UDP

**Padrão de comunicação bidirecional:**

1. **Recepção (AgOpenGPS → ESP32):**
   - Porta 8888 → comandos autosteer (PGN 253/254)
   - Porta 2233 → dados NTRIP RTK
   - Processar em `ReceiveUdp()` no loop principal

2. **Envio (ESP32 → AgOpenGPS):**
   - Porta 5120 → mensagens PANDA/PAOGI (GPS + IMU)
   - Porta 8888 → status autosteer (PGN 253/250)
   - IP destino: broadcast `192.168.137.255`

**Exemplo construção mensagem:**
```cpp
BuildNmea();  // Monta nmea[] global
SendUdp((uint8_t*)nmea, strlen(nmea), Eth_ipDestination, portDestination);
```

### Configurações de Usuário

**SEMPRE modificar no topo de** [src/AOG_Esp32_UM982.cpp](src/AOG_Esp32_UM982.cpp):

```cpp
bool udpPassthrough = false;  // UM982 send full NMEA vs KSXT only
bool makeOGI = false;         // PAOGI (true) vs PANDA (false)
bool baseLineCheck = false;   // Dual antenna IMU fusion
double headingcorr = 0;       // Heading correction (0.1° units)
bool filterRoll = false;      // Kalman filter for roll
```

**Não hardcode:** IP/MAC são configurados via AgOpenGPS UDP (porta 8888).

## Debugging e Troubleshooting

### Watchdog e Timeouts

- **GGA Timeout:** `gpsReadyTime` em [src/AOG_Esp32_UM982.cpp](src/AOG_Esp32_UM982.cpp#L170) - 10s sem GGA desliga LEDs
- **Autosteer Watchdog:** `watchdogTimer` em [src/Autosteer.cpp](src/Autosteer.cpp#L35) - desabilita motor se UDP falha
- **RVC Timeout:** `lastRvcTime` para IMU UART-RVC (19 bytes packets @ 0xAA header)

### Serial Monitor Output

**Sempre habilitar em produção:**
```cpp
Serial.begin(115200);  // USB Debug
Serial.println("Ethernet IP: " + ETH.localIP());
```

**Verificar:**
- "Ethernet status OK" → ETH inicializado
- "GGA received" → GPS ativo
- "IMU data available" → BNO08x respondendo

### Teste de Hardware

**Checklist [README_ESP32.md](README_ESP32.md#L99):**
1. Testar Ethernet: `ping 192.168.137.126`
2. GPS NMEA: Monitor Serial2 @ 460800
3. I2C scan: `Wire.beginTransmission(0x4A)` para BNO08x
4. ADC: `adc.getConversion()` com +5V no ADS1115

## Integrações Externas

### AgOpenGPS (Desktop App)

- **Protocolo:** UDP customizado com PGN (Parameter Group Number)
- **IP padrão:** 192.168.137.255 (broadcast)
- **Configuração:** AgOpenGPS → Network → Set IP → 192.168.137.126

### UM982 GNSS Receiver

- **Baud:** 460800 (configurado via `unichome` utility)
- **Mensagens requeridas:** GGA (10Hz), VTG (10Hz), HPR (10Hz)
- **Dual antenna:** HPR contém roll ($GNHPR,t,h,r,p,qual)

### BNO085 IMU

2. **UART-RVC (simplificado):**
   - Serial1 GPIO2/RX @ 115200
   - Packet: 0xAA + 18 bytes (yaw, pitch, roll, quaternion)
   - Ver [src/AOG_Esp32_UM982.cpp](src/AOG_Esp32_UM982.cpp#L153) `SerialRVC`

## Tarefas Comuns

### Adicionar Novo Sensor Analógico

1. Definir GPIO em [src/GlobalVariables.h](src/GlobalVariables.h)
2. Configurar canal ADC em [src/zADS1115.cpp](src/zADS1115.cpp)
3. Ler em `autosteerLoop()`: `adc.getConversion(ADS1115_REG_CONFIG_MUX_SINGLE_0)`
4. Enviar em PGN_250 byte 5-6

### Alterar Porta UDP

1. Modificar `portMy`/`AOGAutoSteerPort` em [src/AOG_Esp32_UM982.cpp](src/AOG_Esp32_UM982.cpp#L117)
2. Atualizar `Eth_udpPAOGI.begin(portMy)` em [src/zEthernet.cpp](src/zEthernet.cpp)
3. Sincronizar com AgOpenGPS settings

### Ajustar Ganhos PID

1. Editar struct `Storage` em [src/GlobalVariables.h](src/GlobalVariables.h#L32)
2. Valores em `steerSettings`: Kp, Ki, Kd, minPWM, highPWM
3. Salvar em EEPROM via comando UDP (PGN 200/201)

---

**Última atualização:** Firmware v1.0 ESP32 WT32-ETH01 - Fevereiro 2025
**Licença:** GNU GPL v3 - Ver [COPYING](COPYING)
