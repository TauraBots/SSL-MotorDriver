# Perfil experimental de validação a 120 Hz

Este perfil é opt-in. Os perfis normais continuam com seus limites anteriores e o
perfil padrão do ESP32 permanece `TAURA_PROFILE_SAFE_9600`.

## ESP32 host

Compile e grave o ambiente dedicado:

```text
cd esp32-host
platformio run -e esp32dev-validation120
platformio run -e esp32dev-validation120 -t upload
```

Esse ambiente seleciona UART a 921600 baud, D0 a 120 Hz, E1 FAST a 120 Hz,
E1 FULL a 10 Hz e telemetria Web a 120 Hz.

## Quad-MD STM32

Adicione o símbolo abaixo à configuração experimental do compilador no
STM32CubeIDE e recompile o firmware:

```text
TAURA_SERIAL_PROFILE=TAURA_SERIAL_PROFILE_VALIDATION_120
```

Isso reduz somente nesse build o intervalo mínimo de telemetria para 8 ms e o
turnaround para 1 ms. Sem esse símbolo, os valores normais continuam em 100 ms e
3 ms.

Os dois firmwares precisam usar o perfil experimental durante o ensaio. Não use
o ESP32 a 120 Hz com um Quad-MD compilado no perfil normal.

## Validação em bancada

Durante alguns minutos, acompanhe o resumo de um segundo no serial do ESP32:

```text
Telemetry RX: ... Hz | Web TX: ... Hz | Parser errors: ... | Dropped: ... | UART errors: ...
```

Com um navegador conectado, o esperado em regime é aproximadamente 120 Hz em
Telemetry RX e Web TX, sem crescimento contínuo dos contadores de erro ou queda.
Abra dois clientes para conferir que ambos recebem telemetria e que somente um
mantém o controle.
