# Instruções para agentes — AgOpenGPS WT32-ETH01

Firmware Arduino/PlatformIO para ESP32 WT32-ETH01, UM982, BNO085 UART-RVC, ADS1115 e motor PWM Cytron/IBT-2. Organização baseada no projeto ESP32-S3 de referência, mantendo somente os recursos desta placa.

Leia [.github/instructions/firmware.instructions.md](.github/instructions/firmware.instructions.md), [PORTING_NOTES.md](PORTING_NOTES.md) e [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) antes de alterar hardware, controle ou concorrência.

## Organização

- `src/main.cpp`: opções do usuário, inicialização e manutenção.
- `src/tasks.cpp`: criação estática das tasks, afinidade, watchdog e diagnóstico.
- `src/config/pinout.h`: fonte da verdade dos pinos. `defines.h`: constantes e portas. `settings.h/.cpp`: tipos e configurações ativas.
- `src/state/`: comandos e status tipados; snapshots protegidos e configurações pendentes.
- `src/autosteer/`: leitura de entradas/WAS, controle proporcional e supervisor.
- `src/gps/`: NMEA, fusão e PANDA/PAOGI. `context.h` é privado ao domínio GPS.
- `src/imu/`: parser UART-RVC de uma BNO085.
- `src/network/`: LAN8720, sockets UDP, PGNs, fila TX e FIFO RTK.
- `src/hardware/`: ADS1115 e driver PWM.
- `src/persist/`: NVS, namespace `aog`, chaves existentes e tipos preservados.

O cabeçalho central de variáveis globais foi removido. Não recriar um agregador de `extern` ou um cabeçalho que exponha detalhes de todos os módulos. Estado interno deve ficar no `.cpp` ou em um contexto privado ao domínio; APIs públicas são pequenas e específicas.

## Concorrência

- Controle: núcleo 1, prioridade 5, passagem a cada 1 ms e gate de controle de 25 ms. A leitura de encoder fora do gate precisa continuar rápida.
- Navegação: núcleo 0, prioridade 3, passagem a cada 1 ms; único dono das UARTs GPS e IMU e da fusão.
- Rede: núcleo 0, prioridade 2; único dono dos sockets. Todo TX passa pela fila, todo RTK pelo FIFO.
- Supervisor: núcleo 1, prioridade 6, período 20 ms. Controle e supervisor usam o driver do motor com mutex e invalidação por geração.
- `loop()`: manutenção a cada 10 ms e diagnóstico a cada 5 s.

Snapshots precisam da mesma proteção no produtor e consumidor. Desabilitar interrupções locais ou usar apenas `volatile` não protege os dois núcleos. Nunca salvar NVS ou reiniciar a CPU no caminho de RX. Controle confirma o corte antes de entregar a persistência ao `loop()`.

As esperas FreeRTOS liberam a CPU; não usar `delay()`, espera ocupada ou loops sem prazo. Manter orçamento por passagem em UART/UDP e contadores de descarte.

## Hardware e protocolo

| Função | GPIO |
|---|---|
| UM982 RX/TX, 460800 baud | 5 / 17 |
| BNO085 RVC RX, 115200 baud | 2 (TX não utilizado) |
| I2C SDA/SCL ADS1115 | 33 / 32 |
| DIR/enable; PWM1; PWM2 | 14; 4; 12 |
| Steer; remote; work; corrente | 36; 39; 15; 35 |

GPIO36/39 precisam de pull-up externo. Não reativar LEDs nos pinos ocupados. Rádio RTK permanece desativado; correções entram por UDP 2233.

- LAN8720: usar `ETH.h` e `WiFiUDP`, sem `NativeEthernet`.
- LEDC Arduino-ESP32 3.x: `ledcAttach(pin, freq, res)` e `ledcWrite(pin, duty)` por pino.
- Portas: GPS origem 5120, autosteer 8888, RTK 2233, destino AOG 9999. IP/subnet configurados pelo AgIO.
- UM982: `$GNGGA`, `$GNVTG`, `$GNHPR`; preservar handlers `G-GGA`, `G-VTG`, `G-HPR`.
- Configurações de usuário são definidas no topo de `src/main.cpp`; não adicionar Web UI, CAN/Keya, WAS por duas IMUs, WiFi, W5500 ou PSRAM.
- Preservar nomes/tipos das chaves NVS (`st_*`, `sc_*`, `ip_*`, `ident=2400`) para manter calibrações.

## Compatibilidade e validação

A referência original Teensy pode estar em `.teensy_firmware` ou no projeto vizinho `../AOG_esp32-s3_ETH_UM982_task/.AOG_Teensy_UM982/src/`. Compare a lógica de direção e o protocolo ao original ao alterar o port. Não editar ou compilar pastas de referência.

```sh
pio run -e wt32-eth01
python test/run_native.py
pio run -e wt32-eth01 -t upload
pio device monitor
```

Compilar e executar regressões adequadas à mudança. Tests nativos usam simuladores e ASan/UBSan; não confirmam hardware ou prazos reais. Mudanças na direção exigem validação em bancada com AgOpenGPS antes do campo. Atualizar pinout/documentação juntos quando houver alteração de hardware.

Manter licença GNU GPL v3 e cabeçalhos de autoria. Serial de diagnóstico: 115200 baud. O controle é proporcional, apesar do nome histórico PID; não alterar ganhos, frequências ou lei de controle como parte de uma simples reorganização.
