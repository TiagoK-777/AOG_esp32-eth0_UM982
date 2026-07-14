# Firmware AgOpenGPS para WT32-ETH01 (ESP32)

## ✅ Status: Compilação Bem-Sucedida

Firmware portado com sucesso do Teensy 4.1 para ESP32 WT32-ETH01.

## Características

- **Hardware:** ESP32 WT32-ETH01 com Ethernet LAN8720
- **Comunicação:** UDP via RJ45 (porta Ethernet)
- **GPS:** UM982 via UART (Serial2) @ 115200 baud
- **RTK:** Rádio correção via UART (Serial1) @ 9600 baud
- **IMU:** BNO08x via I2C (GPIO33 SDA, GPIO32 SCL)
- **ADC:** ADS1115 16-bit via I2C (GPIO32 SDA, GPIO33 SCL)
- **Controle:** Motor Cytron/IBT2 via PWM (LEDC)

## Pinout WT32-ETH01

### Ethernet (Interno - LAN8720)
- MDC: GPIO23
- MDIO: GPIO18
- Power: GPIO16
- CLK: GPIO0

### Serial/UART
| Periférico | RX | TX | Velocidade |
|------------|----|----|------------|
| USB (AgIO) | 3 | 1 | 115200 |
| GPS UM982  | 5 | 17 | 115200 |
| RTK Radio  | 4 | 2 | 9600 |

### I2C
- SDA: GPIO33
- SCL: GPIO32
- Dispositivos: BNO08x IMU, ADS1115 ADC

### Motor & Controle
| Função | GPIO | Descrição |
|--------|------|-----------|
| DIR1_RL_ENABLE | 14 | Direção motor |
| PWM1_LPWM | 4 | PWM esquerda (LEDC canal 0) |
| PWM2_RPWM | 2 | PWM direita (LEDC canal 1) |

### Switches/Sensores
| Função | GPIO | Descrição |
|--------|------|-----------|
| STEERSW_PIN | 36 | Switch autosteer (Input Only - Pull-up 10k) |
| WORKSW_PIN | 15 | Switch trabalho |
| REMOTE_PIN | 39 | Remote (Input Only - Pull-up 10k) |
| CURRENT_SENSOR_PIN | 35 | Sensor corrente motor |
| PRESSURE_SENSOR | 36 | Sensor pressão (não usado) |

## Configuração de Rede

O módulo aceita configuração de IP via AgOpenGPS (UDP porta 8888).

**Padrão:**
- IP: 192.168.137.126 (com autosteer) ou .120 (GPS only)
- Máscara: 255.255.255.0
- Gateway: 192.168.137.1

**Portas UDP:**
- 5120: Saída dados GPS (PANDA/PAOGI)
- 2233: Entrada NTRIP (correção RTK)
- 8888: Entrada/Saída Autosteer
- 9999: Destino AgOpenGPS

## Upload do Firmware

### Via PlatformIO

```bash
# Conectar ESP32 via USB
# Upload automático
pio run -e wt32-eth01 -t upload

# Monitor serial após upload
pio device monitor
```

### Via ESP Flash Download Tool

1. Abrir [Flash Download Tools](https://www.espressif.com/en/support/download/other-tools)
2. Selecionar **ESP32** e **Develop**
3. Carregar arquivos:
   - `bootloader.bin` @ 0x1000
   - `partitions.bin` @ 0x8000
   - `firmware.bin` @ 0x10000
4. Config: **SPI 40MHz, DIO**
5. Conectar USB e clicar **START**

Binários em: `.pio\build\wt32-eth01\`

## Primeiro Uso

1. **Conectar hardware:**
   - GPS UM982 em RX=5, TX=17
   - Rádio RTK em RX=4, TX=2
   - IMU BNO08x em SDA=33, SCL=32
   - ADC ADS1115 em SDA=33, SCL=32
   - Motor em GPIO 14 (DIR), GPIO4 (PWM1), GPIO2 (PWM2)
   - Cabo Ethernet

2. **Ligar módulo** — LED verde Ethernet deve acender

3. **Abrir AgOpenGPS:**
   - Settings → GPS → UDP
   - IP: 192.168.137.126
   - Porta: 9999

4. **Configurar NTRIP** em AgOpenGPS (envia para porta 2233)

5. **Verificar Serial Monitor:**
   ```
   Start setup
   SerialAOG, SerialRTK, SerialGPS initialized
   Starting AutoSteer...
   Starting Ethernet...
   Ethernet status OK
   IP set Manually: 192.168.137.126
   ```

## Calibração

### IMU BNO08x
- Deixe em superfície plana por 30s após ligar
- Evite vibração excessiva durante calibração
- LED verde GPS indica heading dual antenna OK

### Sensor WAS (Wheel Angle Sensor)
Configure no AgOpenGPS:
- Settings → Steer Settings → Stanley Gain
- Ajuste offset, ganho proporcional (Kp)
- PWM mínimo/máximo conforme motor

### Baseline Dual Antenna
Se usar dual antenna UM982:
- Ajuste `baseLineCheck = true` no código
- `baseLineLimit = 5` (cm máximo diferença)

## Solução de Problemas

### LED Ethernet não acende
- Verificar cabo RJ45
- Testar ping 192.168.137.126
- Verificar se PC está em 192.168.137.x

### GPS não recebe correção
- Confirmar porta 2233 aberta no firewall
- Verificar AgOpenGPS enviando NTRIP
- Monitor serial: deve aparecer dados RTK

### Motor não responde
- Verificar conexões GPIO 14 (DIR), GPIO4 (PWM1), GPIO2 (PWM2)
- Testar PWM: Monitor serial mostra pwmDisplay
- Ajustar settings: lowPWM, highPWM, minPWM

### IMU não detectado
- I2C scan: código mostra "BNO08X Ok" ou "not Found"
- Verificar SDA=33, SCL=32
- Endereço padrão: 0x4A ou 0x4B

### Reset constante
- Fonte alimentação insuficiente (mín. 1A @ 5V)
- Desconectar periféricos e testar isoladamente
- Verificar logs serial antes do reset

## Desenvolvimento

### Estrutura do Código
```
src/
  AOG_Teensy_UM982.ino  - Setup, loop, GPS parsing
  Autosteer.ino         - Controle autosteer
  AutosteerPID.ino      - Algoritmo PID motor
  BNO08x_AOG.cpp/h      - Driver IMU
  zADS1115.cpp/h        - Driver ADC
  zEthernet.ino         - Configuração Ethernet
  zHandlers.ino         - Parsers NMEA (GGA, VTG, HPR)
  zUdpNtrip.ino         - Recepção NTRIP
  zNMEAParser.h         - Parser NMEA genérico
```

### Modificar Configurações

Edite `AOG_Teensy_UM982.ino`:
```cpp
// Correção heading
double headingcorr = 0; // em décimos de grau

// Kalman filter
bool filterRoll = false;
bool filterHeading = false;

// Passthrough GPS
bool udpPassthrough = false; // false = parse NMEA
```

### Rebuild
```bash
# Limpar build anterior
pio run -e wt32-eth01 -t clean

# Recompilar
pio run -e wt32-eth01

# Upload + Monitor
pio run -e wt32-eth01 -t upload && pio device monitor
```

## Diferenças vs Teensy 4.1

### Hardware
- ✅ CPU: 240MHz (vs Teensy 600MHz) — suficiente
- ✅ RAM: 320KB (vs Teensy 1MB) — suficiente
- ⚠️ ADC: 12-bit (vs 16-bit) — usa ADS1115 externo
- ⚠️ UART: 2 HW (vs 8 no Teensy) — suficiente

### Software
- ✅ PWM via LEDC (vs FlexPWM)
- ✅ Preferences.h / NVS para storage persistente (vs EEPROM real)
- ✅ Ethernet nativa LAN8720 (vs PHY externo)
- ⚠️ Buffers serial menores (configurável)

## Referências

- [Documentação PORTING_NOTES.md](PORTING_NOTES.md) - Detalhes técnicos da portagem
- [AgOpenGPS](https://github.com/farmerbriantee/AgOpenGPS)
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/)
- [WT32-ETH01 Hardware](https://github.com/ldab/wt32-eth01)
- [Unicore UM982 Manual](https://en.unicorecomm.com/products/detail/24)

## Suporte

Para problemas específicos ao ESP32:
- Abrir issue no repositório
- Discord AgOpenGPS: canal #hardware

---

**Versão:** 1.1 - Pinagem Corrigida  
**Data:** Dezembro 2025  
**Licença:** GPL v3
