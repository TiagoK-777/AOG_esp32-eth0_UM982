# Plano: Migrar EEPROM.h → Preferences.h (NVS)

## Objetivo
Substituir a biblioteca `EEPROM.h` (deprecated, emulação flash via blob monolítico não thread-safe) por `Preferences.h` (NVS nativo, chave-valor, thread-safe) no firmware AOG ESP32 WT32-ETH01, eliminando risco de corrupção silenciosa em operação prolongada no trator.

## Decisões de Design (confirmadas com o usuário)
1. **Serialização**: campos individuais tipados (`putX/getX`) — um key NVS por membro de struct (~23 chaves). Robusto a reorder do struct, default por campo.
2. **Dados legados**: **reset para defaults**. Não migrar blob EEPROM existente. Se ident ≠ `EEP_Ident` (ou ausente), grava defaults dos structs em NVS.
3. **Concorrência**: abrir/fechar `Preferences` por operação (`prefs.begin(...)/.../prefs.end()`), sem instância global persistente.
   - **Justificativa específica do firmware**: este firmware usa **Ethernet (`ETH.h`), não WiFi** — o stack ETH roda no core 0 mas não toca NVS, então o pânico `Core 1 panic'ed (LoadStoreError)` (originado por `EEPROM.h` + WiFi dual-core) não se aplica. Há **um único escritor**: `autosteerLoop()` → `ReceiveUdp()` → handlers PGN 252/251/201 (loop Arduino, core 1, sem `xTaskCreate` concorrente). Escritas são **raríssimas** (disparadas pelo operador via AgOpenGPS, não no hot-path do loop), então o overhead de `begin/end` (~µs) é desprezível frente ao `commit` síncrono na flash (ms). A escolha open/close é defensiva: protege contra futuras tasks em outro core e garante flush limpo — crítico no PGN 201 antes do `ESP.restart()`.

## Escopo
- **Alvo único**: `src/Autosteer.cpp` (único arquivo que usa `EEPROM.h`).
- **Structs in-memory preservados** (`Storage`, `Setup`, `ConfigIP` em `GlobalVariables.h`): a camada de persistência muda, a representação em RAM não.
- `EEP_Ident` (2400) e `EEread` mantêm função de magic-number.

## Namespace e mapeamento de chaves (namespace `aog`, ≤15 chars)

### Magic
| Key | Tipo API | Default | Origem |
|---|---|---|---|
| `ident` | `putInt`/`getInt` | 0 | compara com `EEP_Ident` (2400) |

### steerSettings (`Storage`) — prefixo `st_`
| Campo (tipo C) | Key | API | Default struct |
|---|---|---|---|
| Kp (uint8_t) | `st_kp` | putUChar/getUChar | 40 |
| lowPWM (uint8_t) | `st_lowpwm` | UChar | 10 |
| wasOffset (int16_t) | `st_wasoff` | putShort/getShort | 0 |
| minPWM (uint8_t) | `st_minpwm` | UChar | 9 |
| highPWM (uint8_t) | `st_highpwm` | UChar | 60 |
| steerSensorCounts (float) | `st_senscnt` | putFloat/getFloat | 30 |
| AckermanFix (float) | `st_ackfix` | Float | 1 |

### steerConfig (`Setup`) — prefixo `sc_`
| Campo | Key | Default struct |
|---|---|---|
| InvertWAS | `sc_invwas` | 0 |
| IsRelayActiveHigh | `sc_relayah` | 0 |
| MotorDriveDirection | `sc_motordir` | 0 |
| SingleInputWAS | `sc_singlewas` | 1 |
| CytronDriver | `sc_cytron` | 1 |
| SteerSwitch | `sc_steersw` | 0 |
| SteerButton | `sc_steerbtn` | 0 |
| ShaftEncoder | `sc_shaftenc` | 0 |
| PressureSensor | `sc_pressens` | 0 |
| CurrentSensor | `sc_currsens` | 0 |
| PulseCountMax | `sc_pulsemax` | 5 |
| IsDanfoss | `sc_danfoss` | 0 |
| IsUseY_Axis | `sc_useyaxis` | 0 |

Todos `uint8_t` → `putUChar`/`getUChar`.

### networkAddress (`ConfigIP`) — prefixo `ip_`
| Campo | Key | Default struct |
|---|---|---|
| ipOne (uint8_t) | `ip_one` | 192 |
| ipTwo (uint8_t) | `ip_two` | 168 |
| ipThree (uint8_t) | `ip_three` | 137 |

Todos `putUChar`/`getUChar`.

## Tarefas de Implementação

### 1. `src/Autosteer.cpp` — includes e globais
- Substituir `#include <EEPROM.h>` por `#include <Preferences.h>`.
- `int16_t EEread = 0;` pode ser removido (não usado fora do bloco de init) — confirmar sem outros usos antes de apagar.
- Adicionar helper local (static no mesmo .cpp, ou novo `src/zPreferences.cpp/.h` se preferir segregação — ver Abordagem de organização abaixo).

### 2. Novo módulo helper (recomendado `src/zPreferences.cpp` + declaração em `GlobalVariables.h`)
Criar 6 funções, cada uma abre/fecha `Preferences` por operação:

```cpp
// protótipos (em GlobalVariables.h, após fwd declarations existentes)
void nvsLoadAll();          // chamado em autosteerSetup()
void nvsSaveSettings();     // PGN 252
void nvsSaveConfig();       // PGN 251
void nvsSaveNetIP();        // PGN 201
```

Implementação `zPreferences.cpp`:

> **Restrição crítica de `Preferences`**: apenas **um namespace pode estar aberto por vez** por instância/processo. `nvsLoadAll()` **não pode** chamar `nvsSave*()` (que fariam novo `begin()`) dentro da sua própria sessão aberta — causaria conflito/fechamento prematuro. A inicialização de defaults deve ser feita **inline** na mesma sessão. Os helpers `nvsSave*()` existem exclusivamente para uso externo (PGN 252/251/201), onde cada um abre, escreve e fecha isoladamente.

- `nvsLoadAll()`:
  - `Preferences prefs; prefs.begin("aog", false);`
  - **Detecção de primeira inicialização via `isKey`** (semanticamente mais claro que `getInt(default)`): `bool firstRun = !prefs.isKey("ident");`
  - `if (firstRun)`: gravar **inline** (sem chamar helpers) todos os defaults dos structs — que já estão populados pelos initializers em `GlobalVariables.h` — usando `putX` direto na mesma sessão (`prefs.putUChar("st_kp", steerSettings.Kp);` ... para os 23 campos), depois `prefs.putInt("ident", EEP_Ident);`. Não atribuir a `EEread` neste ramo (mantém 0 ou seta para `EEP_Ident` conforme preferir; o valor em RAM não é usado depois).
  - `else`: popular structs a partir das chaves (`getX` com defaults iguais aos do struct, por segurança). Ex.: `steerSettings.Kp = prefs.getUChar("st_kp", 40);`
  - `prefs.end();`
- `nvsSaveSettings()`: `begin("aog", false)` → 7 `putX` (mesmos do ramo firstRun, espelhados) → `end()`.
- `nvsSaveConfig()`: 13 `putUChar` → `end()`.
- `nvsSaveNetIP()`: 3 `putUChar` → `end()`.

> **Duplicação de chaves entre `nvsLoadAll()` (firstRun inline) e `nvsSave*()`**: aceitável e intencional — os helpers são chamados em runtime isolado; o bloco inline evita re-entrância no boot. Para reduzir boilerplate, pode-se extrair funções `static` internas (`static void putSettings(Preferences& p)`, etc.) que recebem a `Preferences&` já aberta e escrevem os campos, chamadas tanto por `nvsLoadAll()` (firstRun) quanto por `nvsSave*()`. Recomendado para evitar drift entre os dois caminhos.

### 3. Substituir pontos de uso em `Autosteer.cpp`
- **`autosteerSetup()` linhas 188–204** (bloco EEPROM begin/get/put): substituir por `nvsLoadAll();`. Manter `steerSettingsInit(); steerConfigInit();` logo após (linhas 206–207) — inalterado.
- **PGN 252** (~linha 655–657): trocar `EEPROM.put(10,...); EEPROM.commit();` por `nvsSaveSettings();`.
- **PGN 251** (~linha 691–692): trocar por `nvsSaveConfig();`.
- **PGN 201** (~linha 729–730): trocar por `nvsSaveNetIP();` antes do `ESP.restart();`.

### 4. Limpeza
- Remover `#include <EEPROM.h>` (apenas em Autosteer.cpp — confirmar via grep que não há outros includes).
- Confirmar `EEread` sem outros usos antes de remover a definição; se mantida por segurança, apenas populada por `prefs.getInt`.

### 5. Atualização de documentação
- `PORTING_NOTES.md` linha 157 (item "Considerar migrar EEPROM para Preferences.h"): marcar como **feito**.
- `PORTING_NOTES.md` linhas 95–96 e 149: atualizar descrição.
- `README_ESP32.md` linha 221: trocar "EEPROM emulado em flash" por "Preferences.h / NVS".
- `AGENTS.md` linha 219 e `.github/copilot-instructions.md` linha 218: trocar "Salvar em EEPROM" por "Salvar em NVS via Preferences".

## Mapeamento de structs para helpers (resumo de dados)
Os defaults de cada `getX` **devem ser idênticos** aos defaults declarados nos structs em `GlobalVariables.h` (linhas 31–63), garantindo consistência entre "primeira inicialização" e "campo ausente isolado".

## Riscos e Mitigações
- **Perda de calibrações em campo**: aceita pelo usuário (reset para defaults). Documentar no README que este firmware resetará PID/config/IP ao atualizar desta versão — operador deve reenviar config via AgOpenGPS após o primeiro boot.
- **Concorrência cross-core**: mitigada por open/close por operação. `Preferences`/NVS é thread-safe no ESP-IDF; o `begin`/`end` por chamada evita estado compartilhado entre cores.
- **Re-entrância de namespace (CRÍTICO)**: `Preferences` permite apenas **um namespace aberto por vez**. `nvsLoadAll()` **não pode** invocar `nvsSave*()` (que fariam novo `begin`) dentro da sua sessão. Mitigação: gravar defaults **inline** na mesma sessão, ou via helpers `static` que recebem `Preferences&` já aberta. Validar no code review que nenhum `nvsSave*()` é chamado enquanto `nvsLoadAll()` tem a sessão aberta.
- **Tamanho de chave NVS ≤15 chars**: todas as chaves acima têm ≤15 chars. Validar contagem antes de commit.
- **Detecção de primeira inicialização**: usar `isKey("ident")` (semântico, não depende de default coincidir com valor válido) em vez de `getInt("ident", 0)` + comparação. O ramo firstRun grava todas as chaves + `ident`, garantindo estado NVS completo após o boot inicial.
- **`Preferences` não persiste default implícito**: cada `putX` grava; `getX(key, default)` só retorna default se chave inexistente. No caminho firstRun garantimos gravar todas as chaves, então estado NVS fica completo.
- **Write-on-restart (PGN 201)**: `nvsSaveNetIP()` deve `end()` antes de `ESP.restart()` para garantir flush. Confirmar ordem.

## Validação
1. `pio run -e wt32-eth01` — compila sem erros/warnings de EEPROM.
2. Grep pós-build: `rg -n "EEPROM" src/` → deve retornar apenas referências em comentários/docs atualizados (nenhum uso funcional).
3. Flash em HW: primeiro boot deve imprimir defaults (PID Kp=40, etc.) — confirmar via Serial.
4. Enviar PGN 252/251/201 via AgOpenGPS → power cycle → confirmar valores persistidos (não voltaram aos defaults).
5. Forçar power-fail imediato após PGN 252 → no próximo boot, valor deve estar persistido (NVS é flash, commit é síncrono no `end()`).
6. Operação prolongada (teste de stress, dias) sem `Guru Meditation Error` / `Core 1 panic'ed`.
7. **Code review de re-entrância**: confirmar que `nvsLoadAll()` não chama nenhum `nvsSave*()` (que fariam `begin` aninhado). Defaults gravados inline ou via helper `static` recebendo `Preferences&` aberta.

## Fora de escopo
- Migração automática de dados legados da EEPROM emulada (decisão: reset para defaults).
- Persistência de outras variáveis (GPS/calibrações não estruturadas) — apenas os 3 structs + ident já cobertos.
- Mudança nos structs em `GlobalVariables.h` (campos e tipos permanecem).
