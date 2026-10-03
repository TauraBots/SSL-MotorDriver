# SSL-MotorDriver

Firmware para uma placa controladora com `STM32F103RCTx` focada em 4 motores DC com encoder para base omni.

O projeto usa STM32CubeIDE/HAL e implementa:

- controle fechado de velocidade para 4 rodas
- leitura de encoder em hardware
- PWM de 30 kHz para ponte H
- telemetria binaria via `USART2` no modo bench
- conversao de velocidades do robo para velocidades das rodas
- leitura da tensao de bateria por `ADC1`

Hardware configurado no CubeMX:

- `STM32F103RCTx` em `LQFP64`
- `HSE 8 MHz` com `SYSCLK 72 MHz`
- `USART2` em `PA2/PA3`, 8N1, TX+RX e sem flow control
- o `.ioc` mantem `USART2` em `9600` baud como base; apos a inicializacao
  gerada pelo CubeMX, `ApplyCommModeUartBaud()` aplica `420000` no build match
- `TIM1/TIM8` para PWM
- `TIM2/TIM3/TIM4/TIM5` para encoder

Documentacao:

- [Visao geral da placa](docs/board-overview.md)
- [Protocolo serial](docs/serial-protocol.md)

Arquivos principais:

- [main.c](/c:/Users/Thassio/STM32CubeIDE/workspace_1.19.0/YahBoom-4ch/Core/Src/main.c)
- [app.cpp](/c:/Users/Thassio/STM32CubeIDE/workspace_1.19.0/YahBoom-4ch/Core/Src/app.cpp)
- [motor.cpp](/c:/Users/Thassio/STM32CubeIDE/workspace_1.19.0/YahBoom-4ch/Core/Src/motor.cpp)
- [encoder.cpp](/c:/Users/Thassio/STM32CubeIDE/workspace_1.19.0/YahBoom-4ch/Core/Src/encoder.cpp)
- [omni_kinematics.cpp](/c:/Users/Thassio/STM32CubeIDE/workspace_1.19.0/YahBoom-4ch/Core/Src/omni_kinematics.cpp)

## Build

Abra o `.ioc` ou o projeto no STM32CubeIDE e compile a configuracao `Debug`.

Perfis de firmware:

- `quadmd_bench`: build padrao, `TAURA_COMM_MODE=COMM_MODE_BENCH`, USART2 `9600`, AirPort, `D0/E0/E1`.
- `quadmd_match`: adicionar os simbolos de compilacao
  `TAURA_COMM_MODE=COMM_MODE_MATCH` e
  `TAURA_MATCH_TRANSPORT=MATCH_TRANSPORT_CHANNELS`, USART2 `420000`, CRSF.

A selecao de baud fica em blocos `USER CODE` de `Core/Src/main.c`. Portanto,
regenerar o projeto pelo CubeMX pode recriar a atribuicao base de `9600` em
`MX_USART2_UART_Init()`, mas nao remove a aplicacao posterior do baud de match.
Para conferir o valor ativo no firmware, inspecione `huart2.Init.BaudRate` no
debugger depois de `ApplyCommModeUartBaud()`: `9600` no bench e `420000` no
match.

Detalhes do modo de partida: [ELRS multi-robot match mode](docs/elrs-multi-robot-match-mode.md).

## SSL Configurator

Instale as dependências e abra a interface Qt 6:

```powershell
python -m pip install -r scripts/requirements.txt
python scripts/ssl-configurator.py
```

Os comandos de terminal (`drive`, `discover` e `set-id`) continuam disponíveis.

## Git

Repositorio esperado:

`git@github.com:TauraBots/SSL-MotorDriver.git`
