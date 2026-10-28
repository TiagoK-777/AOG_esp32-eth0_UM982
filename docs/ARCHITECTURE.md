# Arquitetura modular WT32-ETH01

O firmware usa a organização do projeto ESP32-S3 de referência, com somente os recursos do WT32-ETH01: LAN8720, UM982, BNO085 UART-RVC, ADS1115 e motor PWM Cytron/IBT-2. A entrada principal é `src/main.cpp`; o cabeçalho central de variáveis globais foi removido.

## Módulos e propriedade

| Módulo | Responsabilidade | Dono durante execução |
|---|---|---|
| `config/` | Pinos, constantes, opções de usuário e cópia protegida das configurações ativas | Controle publica; demais módulos leem cópias |
| `state/` | Comando de guidance, status da direção/IMU, alterações pendentes | Cada estado tem um produtor; snapshots usam a mesma trava na leitura e escrita |
| `gps/` | Parser GGA/VTG/HPR, fusão e montagem PANDA/PAOGI | Navegação |
| `imu/` | Parser UART-RVC de uma BNO085 | Navegação |
| `autosteer/` | Entradas, WAS, controle proporcional e supervisão | Controle; supervisor verifica os snapshots |
| `hardware/` | ADS1115 e saída LEDC | Controle acessa I2C; controle e supervisor serializam a saída do motor |
| `network/` | Ethernet, PGNs, UDP e FIFO RTK | Rede possui todos os sockets; navegação possui RX/TX da UART GPS |
| `persist/` | Preferences/NVS, namespace e chaves anteriores | Inicialização e manutenção no `loop()` |

`gps/context.h` é privado ao domínio GPS. Não deve ser incluído por rede, direção ou persistência. Os campos NMEA e a fusão não cruzam tasks. A IMU é decodificada pela mesma task que realiza a fusão; o snapshot publicado é usado para diagnóstico e identificação via AgIO.

## Tasks

| Task própria | Núcleo | Prioridade | Ativação |
|---|---|---|---|
| `autosteer` | 1 | 5 | Passagem a cada 1 ms; bloco de controle com gate de 25 ms (40 Hz) |
| `navigation` | 1 | 3 | Passagem a cada 1 ms; RX GNSS limitado a 64 bytes, RVC a 76 bytes, RTK TX a 128 bytes |
| `network` | 0 | 2 | Passagem a cada 1 ms; até um datagrama por porta e dois envios por passagem |
| `safety` | 1 | 6 | A cada 20 ms |
| `loopTask` do Arduino | 1 | 1 | Manutenção a cada 10 ms; diagnóstico a cada 5 s |

A navegação compartilha o núcleo 1 com controle e supervisão. Essa afinidade foi escolhida pelo usuário após observar spikes no ESP32-S3 quando navegação e rede compartilhavam o núcleo 0; o efeito no WT32-ETH01 ainda precisa ser medido. Dentro do núcleo 1, as prioridades permanecem supervisor (6), controle (5), navegação (3) e manutenção (1). A task de rede permanece no núcleo 0.

As esperas usam o FreeRTOS e liberam a CPU. O gate de 25 ms e a leitura rápida do encoder foram mantidos separados para preservar a lógica de entradas do firmware anterior. As frequências reais e os picos de duração devem ser medidos em bancada; o agendador não garante ausência de atraso. O campo `peak` mede o tempo decorrido dentro da chamada ao handler, incluindo preempções durante a chamada; não mede a espera antes de começar a executá-lo.

Stacks das quatro tasks próprias são estáticas: 4096, 4096, 4096 e 2048 bytes. A fila UDP possui oito mensagens de até 256 bytes e o FIFO RTK possui 2048 bytes. Bibliotecas Arduino/rede ainda podem alocar memória internamente. Não há dependência de PSRAM.

## Fluxos

1. AgIO/AOG envia PGNs na porta 8888. Rede valida cabeçalho, tamanho declarado e checksum. O PGN 254 publica um comando completo, com instante de recepção e sequência; mensagens inválidas não renovam o prazo da direção.
2. Controle lê a última cópia do comando e mantém o cálculo proporcional, mínimo PWM, limites, Ackerman, inversões e modo Danfoss. Publica o status depois de concluir o ciclo.
3. GPS/IMU produzem uma mensagem completa PANDA/PAOGI. A fila copia seus bytes; somente rede faz o envio UDP, sem compartilhar sockets.
4. RTK recebido na porta 2233 entra no FIFO em ordem. Navegação transmite somente até o espaço informado pelo buffer TX da UART. Datagrama que não cabe é descartado inteiro e contabilizado; não há truncamento deliberado em 512 bytes.
5. PGNs 252/251/201 criam configurações pendentes. Controle aplica entre passagens, corta o motor e entrega uma cópia à manutenção. O motor permanece bloqueado durante a gravação da NVS. Uma alteração desengaja a direção; após concluir, reativar pela GUI/chave. O PGN 201 reinicia somente depois de persistir o subnet.

Portas mantidas: GPS origem 5120, autosteer origem/recepção 8888, RTK recepção 2233, destino AOG 9999. O IP permanece configurável pelo AgIO; final .126 para direção disponível ou .120 para GPS somente.

## Proteções e limites

- Prazo de comando: 2500 ms desde o último PGN 254 válido, equivalente nominal ao contador anterior de 100 ciclos de 25 ms; subtrações sem sinal toleram o wrap de `millis()`.
- Supervisor: motor desligado com direção indisponível, chave desligada, guidance desligado, velocidade fora da faixa ou ciclo concluído há mais de 100 ms.
- Saída do motor: um mutex serializa corte e PWM; cada corte incrementa uma geração. Um cálculo anterior ao corte não pode reaplicar PWM com a geração antiga.
- ADS1115: timeout I2C de 3 ms por transação; erros de transmissão e respostas incompletas desengajam a direção. Conversão é iniciada para a leitura seguinte.
- Watchdog FreeRTOS: quatro tasks registradas, timeout de 2 s com panic/reset. Gravação de flash pode suspender ambas as CPUs; o corte acontece antes de iniciar a persistência.
- Falha de carga NVS impede engajamento; falha de gravação mantém a direção bloqueada até reiniciar. Chaves `st_*`, `sc_*`, `ip_*` e `ident=2400` permanecem iguais às anteriores, inclusive seus tipos.
- RVC: detecção passa a ser por pacote válido, sem espera de 2 s no boot. A UART continua sendo atendida para recuperar uma IMU conectada depois. O primeiro pacote válido já publica orientação. O delta de yaw usa intermediário de 32 bits ao cruzar +/-180 graus.
- Montagem NMEA tem limite de tamanho e checksum. Extração de campos do parser usa capacidade do array, sem `String`.

## Verificação

```sh
pio run -e wt32-eth01
python test/run_native.py
```

Os testes nativos compilam código de produção com interfaces simuladas de Arduino, I2C, UART e travas. ASan/UBSan verificam PID, PGNs truncados/checksum, NMEA/PANDA/PAOGI (com e sem fusão dual), FIFO RTK com wrap/overflow, snapshots concorrentes, invalidação de PWM após corte, falhas ADC, direção, staging, RVC e compatibilidade NVS. Não simulam o escalonador, o PHY, os tempos reais do barramento ou o circuito do motor.

A suíte de navegação cobre 48 perfis de single+IMU, dual e dual+IMU, cruzando PANDA/PAOGI, montagem da baseline, yaw rate e inversão de roll. Pode ser executada separadamente com `python test/run_native.py --suite navigation`; cenários e requisitos estão em [test/README](../test/README).

Com `headingcorr=900`, a inclinação do HPR representa roll; com `headingcorr=0`, representa pitch. Sem IMU, o eixo não medido fica em zero. Com IMU e `baseLineCheck=true`, heading e o eixo medido recebem correção suavizada somente com qualidade HPR >= 4; o outro eixo continua vindo da IMU. HPR de qualidade inferior mantém os offsets e libera a época GGA/HPR para holdover. HPR inválido não publica valores parciais nem reutiliza a qualidade anterior. A fusão usa a menor diferença circular; PANDA mantém inteiros x10 também neste modo.

Antes do uso em campo, validar em bancada: pinos, Ethernet com link tardio/reconexão, tráfego GNSS/RVC/RTK simultâneo, falha I2C, desligamento ao perder comandos, chaves/encoder, Cytron/IBT-2, configuração NVS e reboot PGN 201. Conferir os picos de duração e o mínimo de stack informados por `TASK`/`CONTROL`.

A referência Teensy utilizada para comparação está no projeto vizinho, em `AOG_esp32-s3_ETH_UM982_task/.AOG_Teensy_UM982/src/`. Ela não é uma dependência de compilação deste firmware.

Formato de PGNs conferido na [especificação oficial AgOpenGPS](https://github.com/AgOpenGPS-Official/Boards/blob/main/PGN.md). Respostas hello agora calculam seu checksum, preservando identificação e payload.
