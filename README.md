# Firmware AgOpenGPS ESP32 (WT32-ETH01 + UM982)

Este firmware é uma adaptação do sistema de autosteer para agricultura de precisão (AgOpenGPS), originalmente desenvolvido para Teensy 4.1, portado para o **ESP32 WT32-ETH01**. Ele gerencia dados GNSS RTK do receptor **UM982**, processa fusão de dados de IMU (BNO085) e controla o motor de direção via PWM.

## 🚀 Funcionalidades Principais

- **Comunicação Ethernet:** UDP nativo via LAN8720 (WT32-ETH01).
- **Suporte UM982:** Processamento de mensagens NMEA (GGA, VTG, HPR) e suporte a antena dupla.
- **Fusão de Dados IMU:** Suporte ao BNO085 via I2C ou UART-RVC.
- **Controle de Direção:** Algoritmo PID não bloqueante para motores DC (Cytron ou IBT-2).
- **Configuração via AgIO:** Suporte total aos protocolos PGN do AgOpenGPS.

---

## 🛠️ Requisitos de Hardware

- **Microcontrolador:** WT32-ETH01 (ESP32 com Ethernet).
- **GNSS:** Unicore Communications UM982 (RTK).
- **IMU:** BNO085 (recomendado modo UART-RVC).
- **ADC:** ADS1115 (para leitura do sensor de ângulo de roda - WAS).
- **Driver de Motor:** Cytron MD30C ou IBT-2.

### Pinagem (Mapeamento WT32-ETH01)

| Função | Pino ESP32 | Notas |
| :--- | :--- | :--- |
| **GPS RX (Serial2)** | GPIO 5 | Conectar ao TX do UM982 |
| **GPS TX (Serial2)** | GPIO 17 | Conectar ao RX do UM982 |
| **IMU RX (Serial1)** | GPIO 2 | Modo UART-RVC (BNO085) |
| **I2C SDA** | GPIO 32 | ADS1115 / BNO085 (Se usar I2C) |
| **I2C SCL** | GPIO 33 | ADS1115 / BNO085 (Se usar I2C) |
| **PWM Motor 1** | GPIO 4 | Direção Esquerda / PWM Cytron |
| **PWM Motor 2** | GPIO 12 | Direção Direita |
| **Direção (DIR)** | GPIO 14 | Enable/Direction para Cytron |
| **Sensor WAS** | ADS1115 | Via I2C (Endereço 0x48) |
| **Interruptor Direção**| GPIO 36 | Entrada (Requer resistor Pull-up 10k) |
| **Sensor de Trabalho** | GPIO 15 | Input |

---

## 💻 Como Compilar e Carregar

Este projeto utiliza o **PlatformIO**.

1. **Instale o VS Code** e a extensão **PlatformIO IDE**.
2. **Clone este repositório** ou baixe os arquivos.
3. **Abra a pasta do projeto** no VS Code.
4. **Dependências:** O PlatformIO baixará automaticamente:
    - `SimpleKalmanFilter`
5. **Configuração de Build:** O arquivo `platformio.ini` já está pré-configurado para o ambiente `wt32-eth01`.
6. **Compilar:** Clique no ícone de "Check" (✔) na barra inferior do PlatformIO.
7. **Upload:** Conecte o ESP32 via USB (usando um adaptador Serial-USB, já que o WT32-ETH01 não possui USB nativo para programação) e clique na seta para a direita (➡).

> **Atenção:** Para colocar o WT32-ETH01 em modo de bootloader, conecte o **GPIO 0 ao GND** durante a inicialização.

---

## ⚙️ Configuração do Firmware

As configurações principais podem ser ajustadas no topo do arquivo `src/AOG_Esp32_UM982.cpp`:

- `udpPassthrough`: Define se o GPS envia NMEA completo ou apenas KSXT.
- `makeOGI`: `true` para mensagens PAOGI, `false` para PANDA.
- `baseLineCheck`: Habilita fusão de antena dupla se estiver usando UM982 com duas antenas.
- `filterRoll` / `filterHeading`: Habilita filtros de Kalman para suavização.

### Configuração de Rede (UDP)

O firmware utiliza IPs padrão do AgOpenGPS:
- **IP do Módulo:** Configurado via AgIO (Padrão geralmente `192.168.137.126`).
- **IP Destino (Broadcast):** `192.168.137.255`.
- **Portas:** 5120 (PAOGI), 8888 (Autosteer), 2233 (NTRIP).

---

## 🔍 Depuração

Monitore a saída serial em **115200 baud**.
- Se a Ethernet iniciar corretamente, você verá o IP local no monitor.
- Se o GPS estiver conectado, mensagens NMEA processadas aparecerão conforme a configuração.

---

## 📜 Licença

Distribuído sob a licença GNU GPL v3. Veja o arquivo `LICENSE` para mais detalhes.

---
*Desenvolvido para uso em agricultura de precisão com AgOpenGPS.*
