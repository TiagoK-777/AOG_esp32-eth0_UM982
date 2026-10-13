# Instruções do projeto

Siga [AGENTS.md](../AGENTS.md), [as regras de firmware](instructions/firmware.instructions.md), [PORTING_NOTES.md](../PORTING_NOTES.md) e [a arquitetura](../docs/ARCHITECTURE.md).

O firmware é modular, com tasks FreeRTOS e estado tipado por domínio. A entrada principal é `src/main.cpp`; pinagem em `src/config/pinout.h`. Não recriar cabeçalho central de variáveis globais, copiar recursos extras do ESP32-S3 ou modificar a lei de controle durante reorganizações.
