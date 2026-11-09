# SSL-MotorDriver

Firmware para uma placa controladora com `STM32F103RCTx` focada em 4 motores DC com encoder para base omni.

O projeto usa STM32CubeIDE/HAL e implementa:

- controle fechado de velocidade (PI + compensacao de atrito) para 4 rodas a 1 kHz
- leitura de encoder em hardware
- PWM de 30 kHz para ponte H
- cinematica omni com limitador de aceleracao e de velocidade
- comunicacao por `USART2` nos modos bench (AirPort) e match (AirPort Team ou CRSF)
- identidade persistente por placa (ID `A..Z` gravado na Flash)
- acionamento da placa do kicker via `I2C2`
- leitura da tensao de bateria por `ADC1`, com corte por subtensao
- watchdog de comunicacao e watchdog independente (IWDG)

Hardware configurado no CubeMX:

- `STM32F103RCTx` em `LQFP64`
- `HSE 8 MHz` com `SYSCLK 72 MHz`
- `USART2` em `PA2/PA3`, 8N1, TX+RX e sem flow control
- o `.ioc` mantem `USART2` em `9600` baud como base; apos a inicializacao
  gerada pelo CubeMX, `ApplyCommModeUartBaud()` mantem `9600` no AirPort Team
  ou aplica `420000` nos backends CRSF
- `TIM1/TIM8` para PWM
- `TIM2/TIM3/TIM4/TIM5` para encoder
- `TIM6` como base de tempo do controle em 1 kHz
- `I2C2` em `PB10/PB11`, 100 kHz, para a placa do kicker

Documentacao:

- [Visao geral da placa](docs/board-overview.md)
- [Protocolo serial](docs/serial-protocol.md)
- [Controle omnidirecional e validacao](docs/omni-motion-validation.md)
- [ELRS multi-robot match mode](docs/elrs-multi-robot-match-mode.md)
- [Validacao AirPort em alta taxa](docs/airport-high-rate-validation.md)
- [Validacao 120 Hz](docs/validation-120hz.md)
- [Investigacao da rotacao angular](docs/yaw-investigation.md) (registro historico)

Arquivos principais:

- [main.c](Core/Src/main.c): inicializacao do CubeMX e laco principal
- [app_c_api.cpp](Core/Src/app_c_api.cpp): ponte C/C++, tick de 1 kHz, estado seguro
- [app.cpp](Core/Src/app.cpp): PID das rodas, bateria e protecoes
- [omni_kinematics.cpp](Core/Src/omni_kinematics.cpp): robo <-> rodas
- [acceleration_limiter.cpp](Core/Src/acceleration_limiter.cpp): rampas de velocidade
- [motor.cpp](Core/Src/motor.cpp) e [encoder.cpp](Core/Src/encoder.cpp): acesso aos timers
- [serial_service.c](Core/Src/serial_service.c): parser, comandos e telemetria
- [kicker_board.c](Core/Src/kicker_board.c): protocolo I2C da placa do kicker
- [robot_config.h](Core/Inc/robot_config.h): geometria e limites de movimento

## Build

Abra o `.ioc` ou o projeto no STM32CubeIDE e compile a configuracao `Debug`.

Perfis de firmware:

- `quadmd_bench`: build padrao, `TAURA_COMM_MODE=COMM_MODE_BENCH`, USART2 `9600`, AirPort, `D0/E0/E1`.
- `quadmd_match_airport`: adicionar `TAURA_COMM_MODE=COMM_MODE_MATCH`; o
  transporte default e `MATCH_TRANSPORT_AIRPORT_TEAM`, USART2 `9600`, D1
  conjunto para A/B/C e uplink solicitado E1/E3/F2/F3/F5. A configuracao
  F0--F5 e destinada a service com apenas o robo/RX alvo ativo.
- `quadmd_match_crsf`: adicionar `TAURA_COMM_MODE=COMM_MODE_MATCH` e
  `TAURA_MATCH_TRANSPORT=MATCH_TRANSPORT_CHANNELS`, USART2 `420000`, CRSF.

A selecao de baud fica em blocos `USER CODE` de `Core/Src/main.c`. Portanto,
regenerar o projeto pelo CubeMX pode recriar a atribuicao base de `9600` em
`MX_USART2_UART_Init()`, mas nao remove a aplicacao posterior do baud de match.
Para conferir o valor ativo no firmware, inspecione `huart2.Init.BaudRate` no
debugger depois de `ApplyCommModeUartBaud()`: `9600` no bench/AirPort Team e
`420000` no match CRSF.

Detalhes do modo de partida: [ELRS multi-robot match mode](docs/elrs-multi-robot-match-mode.md).

## SSL Configurator

Instale as dependências e abra a interface Qt 6:

```powershell
python -m pip install -r configurator/requirements.txt
python configurator/ssl-configurator.py
```

Na tela de controle, `TESTAR KICKER` permite escolher a potência e disparar
um pulso no robô selecionado via rádio AirPort. O teste para o movimento,
solicita freio e envia a sequência segura `kick=0`, pulso e `kick=0`.

O mesmo teste pode ser executado pelo terminal:

```powershell
python configurator/ssl-configurator.py kick-test --port COM4 --robot-id A --kick-power 20
```

Os comandos de terminal (`drive`, `kick-test`, `discover` e `set-id`) continuam
disponíveis.

## Git

Repositorio esperado:

`git@github.com:TauraBots/SSL-MotorDriver.git`
