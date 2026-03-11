# SSL-MotorDriver

Firmware para uma placa controladora com `STM32F103RCTx` focada em 4 motores DC com encoder para base omni.

O projeto usa STM32CubeIDE/HAL e implementa:

- controle fechado de velocidade para 4 rodas
- leitura de encoder em hardware
- PWM de 30 kHz para ponte H
- telemetria binaria via `USART2`
- conversao de velocidades do robo para velocidades das rodas
- leitura da tensao de bateria por `ADC1`

Hardware configurado no CubeMX:

- `STM32F103RCTx` em `LQFP64`
- `HSE 8 MHz` com `SYSCLK 72 MHz`
- `USART2` em `1 Mbps`
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

## Git

Repositorio esperado:

`git@github.com:TauraBots/SSL-MotorDriver.git`
