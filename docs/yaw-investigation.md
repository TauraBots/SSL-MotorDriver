# Investigacao da rotacao angular

## Resultado

A limitacao aparente em aproximadamente `150 RPM` nao e causada pelo PID nem
por uma saturacao de rodas nesse valor. Ela e explicada por dois dados atuais:

1. O SSL Configurator envia `omega = 5 rad/s` nas teclas `Q/E`.
2. O firmware esta construido com `r = 0.030 m` e `L = 0.090 m`.

Para yaw puro, a implementacao produz `wheel_rad_s = omega * L / r`. Logo, o
coeficiente compilado e `3.0`, e `5 * 3 * 9.5493 = 143.24 RPM`, coerente com a
observacao de aproximadamente 150 RPM.

A nota tecnica usa `r = 0.03175 m` e `L = 0.07715 m`, coeficiente `2.4299`.
Esses valores nao correspondem aos valores compilados em `app_c_api.cpp`.

## Caminho do comando

1. `serial_service.c::Serial_ProcessRobotVelocityPacket` le `omega_mrad_s`,
   multiplica por `0.001` e entrega `rad/s` a `AppC_SetRobotVelocity`.
2. `app_c_api.cpp::AppC_SetRobotVelocity` armazena o comando em `rad/s`.
3. `app_c_api.cpp::AppC_FastTick1kHz` chama `AccelerationLimiter::Update`.
4. `OmniKinematics::RobotToWheels` converte `(vx, vy, omega)` para `rad/s` das
   rodas e aplica saturacao vetorial em `52.36 rad/s` (`500 RPM`).
5. `app_c_api.cpp` multiplica por `9.5492966` para obter RPM e limita a
   referencia em `+/-530 RPM`.
6. `App::ApplyMotors` usa a referencia em RPM e o feedback dos encoders em RPM
   no PID. A saida do PID e limitada a `+/-2399`, unidade de PWM, e depois
   aplicada por `Motor::ApplySigned`.

Nao existe conversao para graus por segundo nesse caminho.

## Instrumentacao temporaria

Adicionar os seguintes simbolos a janela Live Expressions/Expressions do
STM32CubeIDE:

- `dbg_yaw_omega_cmd_rad_s`
- `dbg_yaw_omega_limited_rad_s`
- `dbg_yaw_wheel_raw_rpm`
- `dbg_yaw_wheel_limited_rpm`
- `dbg_yaw_pid_reference_rpm`
- `dbg_yaw_feedback_rpm`

Os arrays seguem a ordem `M1, M2, M3, M4`. A instrumentacao nao participa das
decisoes do controlador e nao altera protocolo ou telemetria.

## Limites encontrados

| Limite | Valor | Local | Momento |
| --- | ---: | --- | --- |
| comando Qt Q/E | `5 rad/s` | `ssl_configurator_qt.py::motion_command` | antes da serial |
| comando CLI padrao | `5 rad/s` | argumento `--angular` | antes da serial |
| campo serial | `int16`, escala `0.001 rad/s` | `encode_robot_velocity_packet` | empacotamento |
| aceleracao angular | configuravel, default `10 rad/s2` | `AccelerationLimiter` | antes da cinematica |
| roda cinematica | `500 RPM` | `omni_kinematics.cpp` | em `rad/s`, antes de RPM |
| referencia PID | `530 RPM` | `app_c_api.cpp::ClampSetpoint` | depois da cinematica |
| saida PID | `+/-2399` | `app.cpp::ComputePid` | PWM, nao RPM |
| duty PWM | `0..2399` | `motor.cpp::ClampDuty` | atuacao final |

## Simulacao de yaw puro

| omega | nota tecnica | firmware antes do limite | firmware apos limite |
| ---: | ---: | ---: | ---: |
| `1 rad/s` | `23.20 RPM` | `28.65 RPM` | `28.65 RPM` |
| `5 rad/s` | `116.02 RPM` | `143.24 RPM` | `143.24 RPM` |
| `10 rad/s` | `232.04 RPM` | `286.48 RPM` | `286.48 RPM` |
| `20 rad/s` | `464.08 RPM` | `572.96 RPM` | `500.00 RPM` |

O teste correspondente esta em `tests/test_yaw_kinematics.cpp`.

## Recomendacao

Para confirmar na bancada, comandar yaw puro e observar os seis grupos de
debug. Se `omega_cmd` estabilizar em `5`, a referencia ficar proxima de `143`
RPM e o feedback proximo de `150`, o firmware esta obedecendo ao comando.

A primeira correcao recomendada e tornar a velocidade angular desejada
configuravel no SSL Configurator. Isso nao exige alterar o firmware. A
divergencia entre a geometria real e `0.030/0.090 m` deve ser resolvida em uma
tarefa separada, depois de medir o robo; este diagnostico nao altera a matriz.
