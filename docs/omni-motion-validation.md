# Controle omnidirecional: correção e validação — 2026-11-03

As correções estão implementadas e os testes de host e a compilação STM32 passaram.
Base atualizada para `origin/feature/erls`, commit `d0823e3`, após `git fetch origin`.
A branch local `codex/omni-motion-fix` consolida o histórico da `main`, as
atualizações da `feature/erls` e as correções descritas neste relatório.
O comportamento no robô ainda precisa ser validado. Nenhum firmware foi gravado.

## Causas comprovadas no software

1. A geometria usada era r=0,03 m e L=0,09 m. Agora os valores efetivos
   r=0,03175 m e L=0,07715 m estão centralizados em `robot_config.h`, inclusive
   nos argumentos padrão e fallbacks de `OmniKinematics`. Antes, o coeficiente
   angular L/r era 3 em vez de aproximadamente 2,42992: a referência de roda
   para rotação era cerca de 23,46% maior. O erro de raio também afetava a
   escala translacional em 5,83%. Isso prova erro de escala, não sua contribuição
   exata para as curvas observadas no campo.
2. O limitador subtraía componentes de vetores em referenciais diferentes.
   Para v global=1 m/s e yaw real=5 rad/s, a variação puramente geométrica
   exige 5 m/s² em coordenadas locais, acima dos 4 m/s² disponíveis. A simulação
   reproduz erro persistente com esse algoritmo e o elimina com transporte de
   referencial, sem aumentar a aceleração permitida.
3. O anti-windup considerava apenas a saída PID antes da compensação estática.
   Por exemplo, I=2380, erro=1 RPM, medição=100 RPM: o PID sozinho está abaixo
   de 2399, mas os 40 de compensação fazem o PWM final saturar. Antes o
   integrador crescia; agora a decisão inclui a compensação. O teste também
   verifica o caso negativo. Ganhos P/I/D mantidos.
4. `maxLinearSpeed` e `maxAngularSpeed` eram apenas declarados; agora o limitador
   aplica 2,5 m/s à norma translacional e ±8 rad/s ao comando angular. Entradas
   NaN/Inf são rejeitadas na API com parada segura; configuração inválida não
   altera a configuração nem o estado anteriores. Overflow na cinemática
   também produz saída zero, em vez de NaN.

## Referencial e estimativa de rotação

Para x=direita, y=frente e yaw positivo anti-horário, a atualização é:

```text
vx_transp = cos(dtheta)*vx_prev + sin(dtheta)*vy_prev
vy_transp = -sin(dtheta)*vx_prev + cos(dtheta)*vy_prev
delta_v = desired_body - v_transp
```

A norma de delta_v é limitada a 4*dt (ou 7*dt com brake explícito).
A rampa angular continua limitada a 10 rad/s². Comando zero usa a rampa normal,
salvo brake explícito; watchdog e disable continuam parando imediatamente.

`AppC_FastTick1kHz` reconstrói omega com `WheelsToRobot` usando RPMs dos quatro
encoders **antes do filtro biquad**. Usa a última janela de contagem de 10 ms
disponível, lida antes da atualização dos encoders desse tick:
`dtheta = omega_encoder * 0,001`. Portanto, não usa `omega_cmd` nem
`omega_limited` como medição. A janela representa aproximadamente 6–15 ms de
atraso efetivo, dependendo da fase. O biquad do PID acrescentaria aproximadamente
11 ms em baixas frequências, por isso ele não é usado para transportar o vetor.

A estimativa só é usada sem falhas declaradas de bateria/encoder e com quatro
RPMs finitas abaixo de aproximadamente 1000 RPM em módulo (verificação de
plausibilidade, não um novo limite de referência). Na indisponibilidade, dtheta=0
e o limitador opera localmente; não há fallback para o comando. O estado da
rampa é reinicializado quando a proteção do App reporta falha, evitando acumular
uma referência alta enquanto a saída está bloqueada. O timeout existente de
150 ms também é verificado na interrupção de controle; não depende apenas do
foreground. As funções do kicker chamadas na parada apenas cancelam/enfileiram
estado, sem I2C bloqueante nesse caminho.

Isso é odometria, não orientação absoluta. Escorregamento, roda suspensa,
encoder com sinal errado, quantização e falha ainda não detectada podem causar
erro de yaw. O teste com yaw senoidal independente do comando e 15 ms de atraso
passou, mas não demonstra precisão em qualquer aceleração ou piso. Compare
omega_encoder com visão externa na bancada/campo antes de considerar a estimativa
confiável para as condições reais.

Os comandos recebidos continuam sendo locais. Quem envia deve atualizar
`v_body=R(-theta)*v_global` usando orientação atual. O firmware não recebe
orientação nem um identificador de comando global, portanto não pode recuperar
uma direção global de pacotes locais atrasados ou mantidos constantes. Não foi
introduzida extrapolação desses pacotes, que mudaria sua semântica. Latência,
frequência dos comandos e orientação do TAURA-Framework permanecem hipóteses
de campo, sem alteração do Framework nesta tarefa.

## Cinemática e saturação

As linhas da matriz são `[-sin(phi), cos(phi), L]`, divididas por r, com
phi=[30°,150°,225°,315°] na ordem M1–M4. A contribuição angular é positiva nas
quatro rodas. A inversa usa `(TᵀT)⁻¹Tᵀ`; os testes confirmam ida/volta.
Não foi encontrado defeito de sinal ou ordem nessa matriz.

A saturação continua aplicando **um único fator**
`s=min(1,500/max_abs_raw_rpm)` às quatro rodas. Consequentemente, a inversa
recupera `s*[vx,vy,omega]`: direção translacional e proporção entre translação e
rotação são preservadas. Nenhuma prioridade angular foi introduzida. O clamp
posterior de 530 RPM não atua no caminho cartesiano, limitado a 500 RPM.
O modo direto de bancada mantém seus clamps individuais existentes de 530 RPM;
ele não oferece a mesma garantia de preservação de um twist cartesiano.

500 RPM fica abaixo dos 530 RPM sem carga, mas acima dos 407 RPM nominais com
carga. Foi mantido como teto cinemático existente, não como promessa de resposta
sob carga; 407 RPM não virou limite rígido. Se motores distintos saturarem em
PWM ou responderem diferentemente, a proporção **real** entre rodas poderá ser
distorcida. Além disso, s varia com a orientação e pode modular a velocidade ao
girar. O limitador guarda o vetor anterior à saturação; não foi introduzida
realimentação do fator de saturação no estado da rampa. Os limites globais
medidos nos testes D/E referem-se à saída do limitador sem saturação, não à
aceleração real de um robô com aderência e torque limitados.

## Cadeia de rodas/PID inspecionada

- Encoder mantido em 11 PPR ×4 ×18,8 = 827,2 contagens por volta de saída.
  Janela de 10 ms: aproximadamente 7,253 RPM por contagem. Contadores do firmware
  são de 16 bits; testes cobrem overflow nos dois sentidos.
- Inversão de encoder de M3 e saídas complementares de M4 mantidas. O teste
  de integração verifica a reconstrução de yaw a partir dos quatro sinais reais
  de contador simulados, independentemente do comando angular.
- Referências com módulo menor que 3 RPM zeram PID/integrador. Esse limiar,
  quantização e atrito podem distorcer referências de uma roda ao cruzar zero
  durante movimento combinado. Não foram removidos sem evidência de bancada.
- Compensação de partida 425 e de movimento 40 PWM mantidas nos dois sentidos;
  elas usam sinal da referência. Na inversão rápida, a medição filtrada pode
  permanecer com o sinal antigo por algum tempo. O teste verifica torque de
  sinal contrário em uma inversão; não modela a dinâmica elétrica do motor.
- Integral e PWM continuam limitados a ±2399; derivada sobre medição mantida.
  Anti-windup corrigido para considerar a compensação estática final.
- Filtro de RPM, ganhos PID e proteção de encoder (300 ticks suspeitos,
  PWM >=650 e medição <=2 RPM) mantidos. A proteção existente não detecta
  todos os tipos de falha de encoder, nem garante proporção entre rodas quando
  apenas uma saída é cortada. Verificar isso em ensaio contido.

## Instrumentação sem alteração de protocolo

As variáveis são `volatile`, observáveis por debugger; não há printf no controle.
Leituras de vários campos no debugger não são um snapshot atômico.

| Etapa | Variáveis | Unidade |
|---|---|---|
| Comando recebido | `dbg_motion_desired_xy`, `dbg_yaw_omega_cmd_rad_s` | m/s, rad/s |
| Saída do limitador | `dbg_motion_limited_xy`, `dbg_yaw_omega_limited_rad_s` | m/s, rad/s |
| Rotação medida/incremento | `dbg_motion_measured_omega`, `dbg_motion_delta_theta` | rad/s, rad |
| Origem da rotação | `dbg_motion_rotation_source` (0 indisponível, 1 encoder) | enum |
| Rodas antes/depois da saturação | `dbg_yaw_wheel_raw_rpm`, `dbg_yaw_wheel_limited_rpm` | RPM |
| Fator comum | `dbg_motion_saturation_scale` | 0–1 |
| Referências/feedback PID | `dbg_yaw_pid_reference_rpm`, `dbg_yaw_feedback_rpm` | RPM |
| PWM após proteção/antes dela | `cmd_m1..4`, `dbg_prefault_m1..4` | PWM assinado |
| Falhas | `dbg_fault_m1..4`, `AppC_GetTelemetry().fault_status` | contagem/bits |
| Watchdog | `live_comm_state`, `live_comm_age_ms`, `match_dbg_watchdog_trips` | estado, ms, contagem |

`live_comm_age_ms` é atualizado no foreground; `live_comm_state` é zerado também
pela verificação de timeout de 1 kHz. Campos desired/limited/raw descrevem o
caminho cartesiano; no modo direto, use setpoints, referências PID e PWM.
AirPort, CRSF e o formato das mensagens/telemetria permanecem iguais.

## Reprodução e resultados automatizados

Requer Python e um compilador C++14 para host (GCC/MinGW ou equivalente).
O runner cria e remove os executáveis em diretório temporário e não acessa serial:

```powershell
py tests/run_motion_tests.py --cxx C:/caminho/para/g++.exe
```

Executado com GCC/MinGW 16.2.0, `-O2 -Wall -Wextra -Werror`, assertions ativas.
Cinco executáveis passaram: `test_acceleration_limiter`, `test_yaw_kinematics`,
`test_omni_motion`, `test_motion_pipeline`, `test_pid_autotune`.

| Teste | Resultado medido |
|---|---|
| A/H | 360 direções de translação de 0,6 m/s com ida/volta e yaw zero passaram |
| B | omega=5 rad/s → 116,020180 RPM em cada roda, sem saturação |
| C | (0,4; −0,3; 2) → [−91,886070; 64,396149; 195,279266; 67,675392] RPM |
| D | v global=1 m/s, yaw real=5, comando angular=8, 12 s: erro máximo após 3 s = 0,00000027 m/s; sem transporte = 0,598269 m/s |
| D/E atraso | yaw real=5*sin(t), comando angular=8, estimativa atrasada 15 ms: erro máximo após 3 s = 0,00000030 m/s |
| E | pico da aceleração global do limitador = 4,000066 m/s² (arredondamento float); rampa angular <=10 rad/s² |
| F | (2,5; −1,7; 8) → fator 0,463452, twist reconstruído (1,158631; −0,787869; 3,707620), roda máxima 500 RPM |
| G | zero, brake a 7 m/s², inversão, reset, NaN, limites extremos e configuração inválida passaram |
| G integração | timeout real da API em 150 ms, sequência congelada, nova época após timeout, primeiro passo de retomada 0,004 m/s, subtensão/recuperação sem rampa acumulada, sinais/overflow de encoder, anti-windup ±, proteção encoder e limites PWM passaram |

D/E simulam execução ideal das referências de roda, sem escorregamento, atraso
de motor, quantização de encoder ou saturação. A variante atrasada inclui apenas
atraso na estimativa angular; não é simulação do conjunto eletromecânico completo.
G compila os fontes reais App/API/PID/Motor/Encoder com uma HAL mínima simulada:
valida decisões e valores, não concorrência de interrupções ou temporização STM32.

A suíte Python existente teve 124 testes aprovados. Uma tentativa incluindo
`test_autotune_panel` falhou na importação porque `configurator/ui/autotune_panel.py`
não existe neste checkout; os 124 restantes passaram separadamente. O script
`tests/test_serial.py` abre COM4 ao importar e foi excluído deliberadamente.

Build completo executado em `Debug` com `make -B -j4 all`, GNU Arm 13.3.rel1 e
flags existentes STM32F103xE/Cortex-M3/gnu++14/soft-float, incluindo link e geração
de ELF/HEX. Resultado: text=61956, data=104, bss=6136 bytes. Sem warnings no log
do build completo. Não há novas dependências de runtime. `git diff --check` passou.

## Arquivos alterados

- `robot_config.h`: geometria centralizada e limites de velocidade efetivos.
- `acceleration_limiter.hpp/.cpp`: incremento angular explícito, transporte,
  validação e aplicação dos limites, preservando aceleração/frenagem.
- `omni_kinematics.hpp/.cpp`: geometria padrão/fallback correta, teto exato
  de 500 RPM e rejeição de resultados não finitos; matriz/ordem mantidas.
- `app.hpp/.cpp`: acesso à RPM de encoder sem biquad e anti-windup final.
- `app_c_api.h/.cpp`: odometria angular, instrumentação, rejeição de entradas
  inválidas, timeout em 1 kHz e reset da rampa sob proteção.
- `tests/test_yaw_kinematics.cpp`: expectativa da geometria corrigida.
- `tests/test_omni_motion.cpp`, `tests/test_motion_pipeline.cpp`,
  `tests/host/stm32f1xx_hal.h`, `tests/run_motion_tests.py`: regressões reproduzíveis.

## Roteiro progressivo de validação física (pendente)

1. **Bancada contida, rodas suspensas, chute desabilitado.** Gravação manual
   apenas após revisão. Verificar a polaridade de cada roda com ±30, ±60 e
   ±120 RPM: referência, contador, feedback e PWM devem ter sinais coerentes.
   Observar cruzamentos por zero e comparar resposta positiva/negativa. Não
   inferir o comportamento no piso de rodas suspensas.
2. **Segurança.** Com baixa referência, cortar comunicação e repetir pacotes
   com sequência congelada. Confirmar parada/freio em até 150 ms, somados ao
   próximo tick, e retomada com rampa desde zero. Testar disable e brake. Em
   ensaio mecanicamente contido, verificar proteção de encoder e subtensão,
   confirmando que a rampa não fica acumulada durante bloqueio.
3. **Tempo real.** Medir duração/pior caso da interrupção TIM6 com DWT ou GPIO,
   inclusive durante saturação, falhas e tráfego AirPort/CRSF. Exigir folga no
   período de 1 ms e ausência de perda de ticks. O build não mede esse prazo.
4. **Piso, translação pura.** Rodar retas em ±x, ±y e diagonais a 0,2, 0,5 e
   1,0 m/s, registrando visão externa. Confirmar regressão contra a referência
   anterior e coerência de escala com o raio corrigido.
5. **Rotação pura.** Ensaiar ±0,5, ±1, ±2 e ±5 rad/s. Comparar yaw de visão com
   `dbg_motion_measured_omega`, inclusive nas rampas. Em 5 rad/s a referência
   estacionária deve ser cerca de 116 RPM por roda. Avaliar atraso, erro
   sistemático e ruído antes de depender da estimativa no movimento combinado.
6. **Translação global + rotação, sem saturação.** Começar a 0,2 m/s e
   |omega|=0,5; subir para 0,5 m/s com 1 e 2 rad/s, depois 1 m/s com 5 rad/s.
   Repetir ambos os sentidos de yaw e todas as direções globais. O emissor deve
   usar orientação atual para os componentes locais. Registrar também idade e
   frequência dos pacotes. Separar transitório de partida do trecho constante.
   Confirmar fator=1, comparar trajetória/velocidade global pela visão e medir
   RMS/máximo do erro lateral, yaw e erro por roda contra o mesmo ensaio anterior.
7. **Próximo da capacidade.** Aumentar velocidade gradualmente até s<1,
   registrando RPM raw/saturada/medida e PWM por roda. Conferir o fator comum
   nas referências. Se uma roda ficar em ±2399 sem seguir referência, reduzir
   comando e medir a capacidade com carga/bateria reais; não aumentar o teto
   nem escolher 407 RPM como limite universal sem essa medição.
8. **Retângulos e lookahead.** Repetir as condições que produziam meias-luas,
   primeiro lentas e depois até o envelope validado. Comparar erro lateral por
   lado, erro nos cantos, tempo saturado, assimetria de roda e latência de comando.
   Se curvas persistirem com referências proporcionais corretas, investigar
   escorregamento, compensação estática, atraso PID/encoder, transmissão e
   orientação externa com esses registros.

Definir com a equipe a tolerância de trajetória antes do ensaio (por exemplo,
erro lateral máximo admissível por metro), e comparar a mesma condição de piso,
bateria e carga. Interromper aumentos se houver falha, perda de deadline,
divergência de sinais, saturação persistente de PWM ou deriva crescente. O
resultado físico só pode ser declarado após esses registros de bancada/campo.
