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
| `navigation` | 0 | 3 | Passagem a cada 1 ms; RX GNSS limitado a 64 bytes, RVC a 76 bytes, RTK TX a 128 bytes |
| `network` | 0 | 2 | Passagem a cada 1 ms; até um datagrama por porta e dois envios por passagem |
| `safety` | 1 | 6 | A cada 20 ms |
| `loopTask` do Arduino | 1 | 1 | Manutenção a cada 10 ms; diagnóstico a cada 5 s |

A navegação compartilha o núcleo 0 com a rede, conforme decisão atual do usuário. Nesse núcleo, navegação tem prioridade 3 e rede tem prioridade 2. No núcleo 1 ficam supervisor (6), controle (5) e manutenção (1). Os efeitos dessa afinidade nos picos de duração ainda precisam ser medidos no WT32-ETH01.

As esperas usam o FreeRTOS e liberam a CPU. O gate de 25 ms e a leitura rápida do encoder foram mantidos separados para preservar a lógica de entradas do firmware anterior. As frequências reais e os picos de duração devem ser medidos em bancada; o agendador não garante ausência de atraso. O campo `peak` mede o tempo decorrido dentro da chamada ao handler, incluindo preempções durante a chamada; não mede a espera antes de começar a executá-lo.

Stacks das quatro tasks próprias são estáticas: 4096, 4096, 4096 e 2048 bytes. A fila UDP possui oito mensagens de até 256 bytes e o FIFO RTK possui 2048 bytes. Bibliotecas Arduino/rede ainda podem alocar memória internamente. Não há dependência de PSRAM.

### Diagnóstico de temporização

O `loop()` imprime um bloco a cada 5 s, iniciado por `DIAG uptime=... window=...` em milissegundos. Os acumuladores são retirados e zerados com proteção entre núcleos antes de imprimir. Assim, os picos cobrem todas as passagens desde a coleta anterior, sem perder as janelas intermediárias de 1 s. A primeira janela começa na criação das tasks; o primeiro intervalo de cada medidor é ignorado. Um intervalo que cruza a coleta pertence à janela em que termina. As coletas dos módulos são sequenciais, não um snapshot atômico de todo o sistema.

| Linha/campo | Significado |
|---|---|
| `TASK ... Hz` | Média de passagens na janela `window`, não garantia de espaçamento uniforme |
| `TASK peak` | Maior duração do handler, em us, incluindo esperas e preempções |
| `gapMax` | Maior intervalo entre inícios de passagens/ciclos, em us; revela pausas escondidas pela média em Hz |
| `TASK lateMax` / `late` | Maior atraso em ms e quantidade de inícios atrasados pelo menos um tick em relação ao tick programado por `vTaskDelayUntil`; resolução de 1 ms no SDK atual |
| `TASK over` | Passagens cujo handler durou mais que o período nominal (1 ms ou 20 ms) |
| `CYCLE n/peak/gapMax` | Quantidade, maior duração e maior intervalo entre inícios dos ciclos efetivos de controle de 25 ms |
| `CYCLE lateMax/late` | Excesso sobre o gate de 25 ms, em ms, e quantidade de ciclos com excesso; o primeiro ciclo é ignorado para atraso |
| `CYCLE over25ms` | Ciclos com duração maior que 25 ms |
| `ADS n/peak` | Quantidade e maior duração do bloco de leitura e disparo da próxima conversão do ADS1115 dentro do ciclo |
| `MOTOR stop/control`, `stop/safety`, `stop/other` | Cortes separados por origem; `other` inclui falhas de inicialização/watchdog |
| `MOTOR drive`, `epoch` | Escrita de PWM (inclui rejeições por geração antiga) e leitura da geração |
| `MOTOR n/totalMax/waitMax/bodyMax` | Quantidade e máximos em us: da entrada até antes de liberar o mutex; da entrada até adquirir o mutex; e da aquisição até antes de liberá-lo |

Os máximos de espera, corpo e total podem vir de chamadas diferentes: não devem ser somados. O corpo de `stop` contém GPIO e, quando necessário, as duas chamadas LEDC; `waitMax` inclui tanto a espera pela trava quanto eventuais preempções antes de adquiri-la. Todos os tempos são de relógio, não consumo exclusivo de CPU. A instrumentação acrescenta leituras de relógio e seções críticas curtas; nenhuma impressão ocorre nas tasks medidas nem segurando o mutex do motor. O diagnóstico não altera prioridades, períodos, PWM ou lei de controle.

`CONTROL cycles/peak`, contadores de perdas e mínimo de stack continuam acumulados desde a inicialização. Os contadores `TASK`, `CYCLE`, `ADS` e `MOTOR` são por janela.

As linhas `GPS` mostram contadores acumulados e as taxas médias na janela `DIAG`: `gga` conta GGA aceitas pelo parser; `hpr`, HPR aceitos pelo handler; `panda`, PANDA/PAOGI montadas; `queued`, as montadas que entraram na fila; `tx`, datagramas GPS cujo envio foi aceito pela pilha UDP. Esse último valor não confirma recepção no PC e, em passthrough, conta sentenças raw. Não há temporizador artificial para produzir 10 Hz: a saída continua dirigida pelas sentenças de entrada. A média de uma janela curta pode oscilar por eventos próximos às bordas; comparar também os totais ao longo de várias janelas.

`gapGga`, `gapPanda` e `gapTx` são os maiores intervalos entre eventos em us, acumulados desde o boot, ignorando o primeiro. `GPS RX` traz bytes lidos e maior backlog observado no buffer de software, erros do parser (`nmeaErr`, incluindo `crcErr`), overflow da FIFO física (`fifoOvf`), buffer RX cheio (`rxFull`), erros de enquadramento/paridade (`frameErr`, `parityErr`), falhas de montagem (`buildFail`) e falhas de envio GPS (`txFail`). Eventos UART são contabilizados por callback com atômicos; contadores de navegação e rede usam snapshots protegidos de seus respectivos produtores. A fila de eventos UART é finita: zero erros registrados não é prova absoluta de ausência de perda. As contagens não identificam épocas UTC distintas ou duplicadas.

Para o simulador de 10 Hz, comparar GGA/PANDA/TX e seus intervalos. GGA abaixo de 10 Hz com erros aponta para recepção/parser; GGA correta com PANDA menor pede verificar gate/montagem; PANDA correta com `queued` menor indica fila; `tx` menor pede conferir fila/envio. Se as três taxas estiverem corretas, investigar o percurso até AgIO/AOG e sua janela de cálculo. Os snapshots de cada estágio são sequenciais e podem diferir transitoriamente por mensagens em trânsito.

Para investigar os atrasos, capturar blocos completos com GPS/IMU/RTK ativos: primeiro parado com PWM zero; depois em bancada com Free Drive e variações de ângulo; por fim desengajado. Anotar as transições. Comparar `stop/safety waitMax` com `stop/control bodyMax`, `CYCLE peak` com `ADS peak` e `TASK lateMax/gapMax` nos dois núcleos. Os logs localizam o caminho que demora, mas não identificam sozinhos uma ISR ou task interna que tenha causado a preempção.

## Fluxos

1. AgIO/AOG envia PGNs na porta 8888. Rede valida cabeçalho, tamanho declarado e checksum. O PGN 254 publica um comando completo, com instante de recepção e sequência; mensagens inválidas não renovam o prazo da direção.
2. Controle lê a última cópia do comando e mantém o cálculo proporcional, mínimo PWM, limites, Ackerman, inversões e modo Danfoss. Publica o status depois de concluir o ciclo, incluindo a sequência de transições de guidance consumida. A rede só responde ao PGN 254 depois de um novo ciclo e com essa sequência atualizada, para não devolver ao AOG o estado da chave anterior à transição. Comandos recebidos antes dessa resposta compartilham o retorno mais recente; duplicatas não reiniciam a espera. O atraso nominal é de até um gate de 25 ms, acrescido do escalonamento e da fila TX, sem espera bloqueante na rede.
3. GPS/IMU produzem uma mensagem completa PANDA/PAOGI. A fila copia seus bytes; somente rede faz o envio UDP, sem compartilhar sockets.
4. RTK recebido na porta 2233 entra no FIFO em ordem. Navegação transmite somente até o espaço informado pelo buffer TX da UART. Datagrama que não cabe é descartado inteiro e contabilizado; não há truncamento deliberado em 512 bytes.
5. PGNs 252/251/201 criam configurações pendentes. Controle aplica entre passagens, corta o motor e entrega uma cópia à manutenção. O motor permanece bloqueado durante a gravação da NVS. Uma alteração desengaja a direção; após concluir, reativar pela GUI/chave. O PGN 201 reinicia somente depois de persistir o subnet.

Portas mantidas: GPS origem 5120, autosteer origem/recepção 8888, RTK recepção 2233, destino AOG 9999. O IP permanece configurável pelo AgIO; final .126 para direção disponível ou .120 para GPS somente.

## Proteções e limites

- Prazo de comando: 2500 ms desde o último PGN 254 válido, equivalente nominal ao contador anterior de 100 ciclos de 25 ms; subtrações sem sinal toleram o wrap de `millis()`.
- Supervisor: motor desligado com direção indisponível, direção desengajada, chave desligada, guidance desligado, velocidade fora da faixa de PWM ou ciclo concluído há mais de 100 ms.
- Engajamento e PWM são independentes: com guidance válido e entrada habilitada, o piloto pode ficar engajado parado; o PWM fica zerado abaixo de 1,5 km/h. Controle e supervisor usam a mesma faixa de PWM (1,5 a 25 km/h). O limite superior de 25 km/h também desengaja a entrada, como antes.
- Saída do motor: um mutex serializa corte e PWM; cada corte incrementa uma geração. Um cálculo anterior ao corte não pode reaplicar PWM com a geração antiga. O corte sempre força DIR/enable em LOW. As duas escritas LEDC de zero só são omitidas se o corte anterior teve sucesso em ambas e nenhum `drive` válido ocorreu desde então; falhas exigem nova tentativa. Isso evita reprogramar repetidamente duty zero, que no ESP32 clássico pode esperar pela atualização anterior dentro da seção crítica do driver LEDC. O cache é privado e protegido pelo mesmo mutex. Frequência e lei de controle permanecem iguais.
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

O hello legado enviado pelo [AgIO](https://github.com/AgOpenGPS-Official/AgOpenGPS/blob/master/SourceCode/AgIO/Source/Forms/UDP.designer.cs) é `80 81 7F C8 03 38 00 00 47`: usa checksum fixo `0x47`, embora a soma seja `0x82`. A validação aceita exatamente esse pacote além das mensagens com checksum correto; a exceção não se aplica a comandos ou configurações. Ao receber o hello, a rede responde com PGN 126 se a direção está disponível e PGN 121 se a IMU já recebeu RVC válido. São essas respostas que renovam os indicadores do AgIO; a orientação continua no PANDA/PAOGI e o verde da direção não indica engajamento. `useBNO08x` indica detecção e permanece ativo após a primeira leitura, sem timeout de presença nesta versão.

## Engajamento e teste parado

`None` usa o botão da tela do AOG. `Switch` e `Button` selecionam, respectivamente, chave e botão momentâneo físicos no GPIO36; precisam de pull-up externo e acionamento em LOW. Essas entradas continuam sendo necessárias nos seus respectivos modos. O engajamento é avaliado depois das entradas e do WAS do próprio ciclo, sem depender da chegada de outro PGN para reconhecer uma mudança da entrada.

O estado anunciado nasce desengajado (`steerSwitch=1`, bit 1 de `switchByte` em 1). A resposta PGN 253 usa o snapshot após o controle consumir a transição, inclusive em falhas de ADC. Ao aplicar configurações, o corte atualiza imediatamente tanto `steerSwitch` quanto `switchByte`, mesmo fora do gate. Isso evita anunciar uma chave ligada com a direção já cortada. A detecção de transições preservada entre tasks impede que comandos repetidos apaguem uma mudança no modo `None`.

O [AOG, em `Position.designer.cs`](https://github.com/AgOpenGPS-Official/AgOpenGPS/blob/master/SourceCode/GPS/Forms/Position.designer.cs), transmite o Free Drive pelo PGN 254 com status 1, velocidade de teste 80 (8,0 km/h) e o ângulo solicitado. Não existe indicação separada de Free Drive nesse PGN: o firmware usa a velocidade recebida, sem tentar identificar o modo pelo valor 8. Ao sair do teste e receber novamente velocidade zero, corta o PWM mesmo que o piloto permaneça engajado. Free Drive não contorna entrada física, timeout, falha do ADC, limites do WAS ou bloqueio de persistência.

O diagnóstico `STEER`, a cada 5 s, mostra disponibilidade, engajamento lógico, modo da entrada, `switch` (0 habilitado, 1 desligado), guidance, velocidade recebida do AOG e PWM do último ciclo. Por exemplo, `engaged=1 speed=0.0 pwm=0` representa piloto engajado com motor cortado por baixa velocidade. Os snapshots de direção, comando e configuração são lidos separadamente; a linha é diagnóstico periódico, não um registro atômico dos três domínios.

A comparação usou as referências locais `.AOG_Teensy_UM982`, `.AIO_v4_FirmwareWASBNOkorektaBNO`, `.AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025` e `.ESP32-AIO-AG`. Elas diferem nos limites de baixa velocidade e na combinação de GUI/entrada física; nenhuma foi editada ou compilada. O comportamento de piloto engajado parado com PWM cortado segue a decisão do usuário para este port.
