# ELRS multi-robot match mode

Este firmware mantém dois modos de comunicação:

```text
BENCH MODE
PC -> ELRS AirPort -> RX -> STM32
```

Bench mode usa o protocolo Quad-MD existente em `USART2` para `D0`, `E0`,
`E1`, discovery, configuracao e telemetria.

```text
MATCH MODE
PC -> USB -> Jumper AION Nano TX
              AirPort
                 |
                 +-- RX A -> STM32 A
                 +-- RX B -> STM32 B
                 +-- RX C -> STM32 C
```

O transporte principal de match usa AirPort como UART transparente. Um unico
pacote Quad-MD D1 carrega comandos diferentes para A, B e C; todos os RX recebem
os mesmos bytes e cada STM32 escolhe somente o slot do seu `robot_id` persistente.
O uplink e estritamente solicitado: E0 pode produzir E1 somente no robo
enderecado, E2 pode produzir E3 em um slot de discovery e operacoes de service
F0/F1/F4 podem produzir F2/F3/F5. Qualquer transmissao espontanea continua
bloqueada nesse modo.
O backend CRSF normal continua disponivel como alternativa experimental.

## Configuracao

O padrao de build permanece em bench:

```c
#define TAURA_COMM_MODE COMM_MODE_BENCH
```

Para firmware de partida, compilar com:

```c
#define TAURA_COMM_MODE COMM_MODE_MATCH
#define TAURA_MATCH_TRANSPORT MATCH_TRANSPORT_AIRPORT_TEAM
```

`MATCH_TRANSPORT_AIRPORT_TEAM` e o default quando
`TAURA_MATCH_TRANSPORT` nao e definido. Os bauds sao escolhidos por modo e
transporte:

```text
BENCH                         USART2 = 9600 baud, 8N1
MATCH + AIRPORT_TEAM         USART2 = 9600 baud, 8N1
MATCH + CHANNELS/TEAMFRAME   USART2 = 420000 baud, 8N1
```

Para o enlace AirPort de partida, configure as tres pontas em `9600`:

```text
TX AirPort baud = 9600
RX AirPort baud = 9600
STM32 USART2    = 9600
```

O arquivo `YahBoom-4ch.ioc` permanece com `USART2.BaudRate=9600`. Esse e o
valor base do CubeMX e faz com que `MX_USART2_UART_Init()` continue sendo codigo
gerado normal e compativel com o perfil bench. Depois que todos os perifericos
sao inicializados, `main()` chama `ApplyCommModeUartBaud()` dentro de um bloco
`USER CODE`. A funcao tambem esta em bloco protegido e:

- mantem `9600` sem reinicializacao no bench e no match AirPort Team;
- troca para `420000` e chama `HAL_UART_Init()` novamente apenas nos transportes
  match baseados em CRSF, antes de `SerialService_Init()` iniciar DMA/recepcao.

Assim, uma nova geracao pode reescrever a linha de baud dentro de
`MX_USART2_UART_Init()` para `9600`, conforme o `.ioc`, sem perder a selecao de
match. Nao mova a selecao dinamica de volta para essa linha gerada.

Para verificar em firmware, coloque um breakpoint depois de
`ApplyCommModeUartBaud()` e inspecione `huart2.Init.BaudRate`. O valor deve ser
`9600` no bench/AirPort Team e `420000` no match CRSF. No AirPort Team, `PA2`
permanece configurado como TX por compatibilidade com CubeMX e transmite apenas
respostas E1/E3/F2/F3/F5 solicitadas; a recepcao pode ser verificada em `PA3` com
analisador logico 8N1.

Nao houve alteracao de pinagem. A placa documentada possui `USART2` em
`PA2/PA3` com DMA; nenhum segundo UART foi configurado sem conflito no `.ioc`.
Assim, bench e match sao modos alternativos sobre a UART existente.

## Build profiles

O codigo nao exige editar `main.c` entre bench e match. Use dois perfis no
STM32CubeIDE, ou equivalentes no sistema de build:

```text
quadmd_bench
  symbols:
    USE_HAL_DRIVER
    STM32F103xE
  optional/default:
    TAURA_COMM_MODE=COMM_MODE_BENCH

quadmd_match_airport
  symbols:
    USE_HAL_DRIVER
    STM32F103xE
    TAURA_COMM_MODE=COMM_MODE_MATCH
    TAURA_MATCH_TRANSPORT=MATCH_TRANSPORT_AIRPORT_TEAM  (opcional/default)

quadmd_match_crsf
  symbols:
    USE_HAL_DRIVER
    STM32F103xE
    TAURA_COMM_MODE=COMM_MODE_MATCH
    TAURA_MATCH_TRANSPORT=MATCH_TRANSPORT_CHANNELS
```

Em STM32CubeIDE: Project Properties -> C/C++ Build -> Settings -> MCU GCC
Compiler -> Preprocessor, duplique a configuracao existente e adicione os
simbolos acima ao perfil escolhido. Repita tambem no MCU G++ Compiler para
os arquivos C++.

## D1 AirPort Team

O frame e atomico, possui 32 bytes e usa CRC16-CCITT-FALSE, o mesmo algoritmo
do protocolo Quad-MD existente. O valor numerico do SOF e `0xAA55`; na linha,
os bytes aparecem como `55 AA`.

```text
offset  tamanho  campo
0       2        SOF = 55 AA
2       1        TYPE = D1
3       1        VERSION = 1
4       2        SEQUENCE uint16 little-endian
6       8        Robot A: vx, vy, omega, kick_power, flags
14      8        Robot B: vx, vy, omega, kick_power, flags
22      8        Robot C: vx, vy, omega, kick_power, flags
30      2        CRC16 little-endian sobre bytes 0..29
```

Cada slot de robo usa:

```text
+0  vx          int16 little-endian, m/s x 1000
+2  vy          int16 little-endian, m/s x 1000
+4  omega       int16 little-endian, rad/s x 1000
+6  kick_power  uint8, limitado a 0..100 pelo caminho comum
+7  flags       uint8
```

Flags:

```text
bit 0 kick
bit 1 chip
bit 2 brake
bit 3 dribbler
bit 4 enabled
bits 5..7 reservados
```

`enabled=0` aplica safe state somente ao robo daquele slot. O comando so e
aplicado depois de SOF, tamanho, tipo, versao, CRC e sequence validos. Sequence
duplicada ou antiga nao atualiza o comando nem repete kick. Kick e gerado apenas
na borda `0 -> 1`; o primeiro frame observado estabelece o baseline.

O watchdog AirPort usa `MATCH_AIRPORT_COMMAND_TIMEOUT_MS=120`. Inicializacao,
ausencia de frame e timeout aplicam safe state. Os contadores ficam em
`MatchControlStats`: `team_frames_ok`, `team_frames_bad_crc`,
`team_frames_bad_version`, `team_frames_duplicate`, `team_frames_old`,
`team_frames_missed` e `team_frames_timeout`. No STM32CubeIDE, os mesmos eventos
podem ser acompanhados diretamente pelas variaveis `match_dbg_team_frames_ok`,
`match_dbg_team_frames_bad_crc`, `match_dbg_team_frames_duplicate`,
`match_dbg_team_frames_old`, `match_dbg_team_frames_missed`,
`match_dbg_watchdog_trips`, `match_dbg_team_timeouts`,
`match_dbg_last_interframe_ms`, `match_dbg_max_interframe_ms` e
`match_dbg_last_sequence`.

Um frame de 32 bytes ocupa aproximadamente `33,3 ms` em 9600 baud/8N1. Assim,
50 Hz nao e fisicamente possivel nesse baud; para o primeiro teste, use 20 Hz.
O timeout de 120 ms admite um frame de 50 ms perdido e pequena margem de jitter,
sem enfraquecer o failsafe para centenas de milissegundos. O watchdog interno
do AppC continua em 150 ms; o watchdog MATCH normalmente atua primeiro.

No Configurator, D1 nao usa a janela de guarda de telemetria: o deadline de
comando permanece acumulativo em 50 ms e E0/E1 e best-effort. Em conflito, D1
e escrito primeiro e uma resposta E1 pode ser perdida. Para diagnostico, o host
mantem `last_command_tx_ns`, `max_command_tx_gap_ms` e `command_frames_sent`.
Compare `max_command_tx_gap_ms` com `match_dbg_max_interframe_ms`: gap alto nos
dois lados indica atraso no host; gap normal no host e alto no STM32 indica
perda ou atraso no AirPort/RF. CRC crescente indica corrupcao, enquanto salto
de sequence aumenta `match_dbg_team_frames_missed`.

Zero normal e um comando habilitado sem brake: a rampa usa `maxLinearAccel` e,
ao chegar a zero, o motor fica em coast. Um brake explicito usa
`maxBrakeAccel`; emergency stop, slot desabilitado e watchdog mantem
`enabled=false`, `brake=true` e forcam o estado seguro/freio eletrico.

## Uplink solicitado no AirPort Team

O parser de match aceita `D1`, `E0`, `E2`, `F0`, `F1` e `F4`. `D0` e demais
tipos continuam ignorados. O envio possui uma barreira independente: em
`MATCH_TRANSPORT_AIRPORT_TEAM`, a fila DMA aceita somente `E1`, `E3`, `F2`,
`F3` e `F5`.

- `E0 TELEMETRY_REQUEST` sempre deve conter um `robot_id` A/B/C. Somente o
  STM32 com ID configurado igual responde, nunca ha resposta a broadcast, e E1
  e enfileirado apos `SERIAL_TELEMETRY_TURNAROUND_MS`. Nao existe telemetria
  espontanea e so uma resposta de telemetria pode ficar pendente.
- `E2 DISCOVERY_REQUEST` agenda E3 nos slots A = 20 ms, B = 90 ms e C = 160 ms.
  O espacamento de 70 ms e maior que os aproximadamente 28,1 ms necessarios para
  transmitir os 27 bytes de E3 em 9600 baud/8N1.
- Um dispositivo ainda sem ID usa UID e o nonce de E2 para escolher um dos
  slots. Alterar o nonce pode alterar o slot e reduzir colisoes entre tentativas.

O agendamento usa `HAL_GetTick()` e o envio usa DMA, sem espera bloqueante. A
recepcao de D1 e `MatchControl_Task()` continuam executando antes das tarefas de
uplink, portanto o watchdog de comando permanece ativo durante a espera de E1/E3.
Os contadores de diagnostico sao `match_telemetry_requests`,
`match_telemetry_responses`, `match_discovery_requests`,
`match_discovery_responses` e `match_uplink_dropped_busy`.

### Configuracao de service F0--F5

O mesmo firmware MATCH pode executar discovery de placas por UID (`F0/F2`),
alterar o Robot ID (`F1/F3`) e gravar limites de movimento (`F4/F5`). As
respostas usam a fila TX por DMA; D1 e o watchdog continuam sendo processados
enquanto uma resposta aguarda. F1 e F4 exigem CRC, UID exato e a chave de
configuracao existente, e forcam safe state antes da tentativa de gravacao.

F0 conserva o atraso derivado de nonce + UID para reduzir colisoes. Entretanto,
os testes fisicos mostraram que o uplink AirPort com varios RX nao funciona como
um barramento bidirecional confiavel. Discovery de placas pode retornar apenas
uma parte dos RX ativos. Use como procedimento suportado de manutencao:

1. mantenha somente o robo/RX alvo ligado;
2. conecte o Configurator em 9600 baud;
3. use `CONFIGURACAO > DESCOBRIR PLACAS` e confirme o UID;
4. grave ID ou limites, se necessario;
5. desligue esse robo antes de repetir no proximo.

Os contadores adicionais observaveis no debugger sao
`match_config_discover_requests/responses`,
`match_config_set_id_requests/responses` e
`match_config_motion_requests/responses`.

### Modelo do Configurator para multi-RX

Os testes fisicos confirmaram broadcast de downlink para varios RX, mas somente
um RX entrega uplink AirPort ao TX/PC por vez. Por isso, o Configurator separa:

- `registered`: robo conhecido que pode continuar recebendo D1;
- `telemetry online`: robo com E1/E3 recente;
- `active_robot`: alvo escolhido pelo operador para controle;
- `uplink_robot`: RX que atualmente consegue responder ao PC.

Quando existe `uplink_robot`, E0 e enviado somente para ele. Apos 1,5 s sem
resposta, o robo permanece registrado, passa para `NO TELEMETRY`, o uplink e
limpo e inicia um probe lento entre os IDs registrados A/B/C. Timeouts desse
probe sao contabilizados separadamente e nao degradam a estatistica principal
do link. Uma resposta E1 ou E3 valida fixa imediatamente o novo uplink.

## Backend alternativo: CRSF 333 Full / 16ch Rate/2

O primeiro backend usa os 16 canais CRSF diretamente. O frame CRSF aceito e o
padrao `RC_CHANNELS_PACKED` (`0x16`) com CRC8 DVB-S2 valido.

Constantes de escala:

```text
CRSF min    = 172
CRSF center = 992
CRSF max    = 1811
MAX_VX      = 2.5 m/s
MAX_VY      = 2.5 m/s
MAX_OMEGA   = 8.0 rad/s
```

Os canais sao quantizados/dequantizados por funcoes explicitas. O ELRS 333 Hz
Full Resolution oferece aproximadamente 10 bits efetivos; o empacotamento CRSF
continua usando os 11 bits do frame padrao.

## Mapeamento

```text
CH1  Robot A vx
CH2  Robot A vy
CH3  Robot A omega
CH4  Robot A action

CH5  Robot B vx
CH6  Robot B vy
CH7  Robot B omega
CH8  Robot B action

CH9  Robot C vx
CH10 Robot C vy
CH11 Robot C omega
CH12 Robot C action

CH13 Robot A kick_power
CH14 Robot B kick_power
CH15 Robot C kick_power
CH16 global enable / emergency stop
```

`A`, `B` e `C` sao IDs logicos do STM32 gravados pela infraestrutura existente
de identidade. Binding phrase diferente por robo nao deve ser usada.

## Action

O canal de action codifica 4 bits no range inteiro `0..15`:

```text
bit 0 = kick
bit 1 = chip
bit 2 = brake
bit 3 = dribbler enable
```

`kick_power` fica em `CH13/14/15`, no range `0..100`. O firmware gera `kick`
somente na borda de subida do bit de kick depois de ja ter observado um baseline
anterior. Manter o bit alto, religar com o bit alto ou alternar `CH16` com o bit
alto nao gera chutes repetidos/acidentais. Como o firmware atual ainda nao
aciona kicker fisico, o evento fica exposto como `kick_power` no caminho de
comando/telemetria.

## CH16 e failsafe

Se `CH16 <= center`, o comando aplicado e seguro:

```text
vx = 0
vy = 0
omega = 0
kick = 0
dribbler = 0
brake = true
```

Esse estado tem prioridade sobre todos os outros canais.

O match mode tambem possui watchdog independente:

```c
#define MATCH_COMMAND_TIMEOUT_MS 50U
```

Se nenhum CRSF valido chegar dentro desse intervalo, o firmware chama o estado
seguro e nao mantem o ultimo comando indefinidamente. AirPort Team usa o timeout
separado de 100 ms descrito acima.

## CRSF UART

O parser CRSF valida:

```text
address/sync
length
type
payload
CRC8 DVB-S2
```

Frames invalidos ou truncados sao descartados sem atualizar comandos
parcialmente. Bytes aleatorios antes do frame sao ignorados por ressincronizacao
baseada no campo de tamanho.

Enderecos/sync usados:

```text
HOST bridge -> ELRS TX module:
  first byte = 0xEE (CRSF transmitter address)
  baud inicial = 921600, configuravel no host com --baud

ELRS RX -> STM32:
  first byte/sync = 0xC8
  baud = 420000 em quadmd_match
```

O parser do STM32 aceita `RC_CHANNELS_PACKED` vindo do RX ExpressLRS pela UART
CRSF. O host gera `RC_CHANNELS_PACKED` para o modulo TX usando o endereco de TX.

## TeamFrame experimental

A arquitetura ja reserva o backend:

```text
MATCH_TRANSPORT_CHANNELS
MATCH_TRANSPORT_TEAMFRAME
MATCH_TRANSPORT_AIRPORT_TEAM
```

`MATCH_TRANSPORT_TEAMFRAME` continua experimental e nao e o D1 AirPort Team. A
ideia desse backend antigo e usar CH1-CH4
como barramento de dados de 40 bits por atualizacao e montar um frame atomico de
128 bits em 4 fragments:

```text
Robot A = 40 bits
Robot B = 40 bits
Robot C = 40 bits
CRC8    = 8 bits
```

Cada fragmento deve carregar pelo menos `part index` e `sequence`. O comando so
pode ser aplicado apos receber os 4 fragments da mesma sequencia e validar o CRC.
Fragmento faltante, duplicado de sequencia antiga, mistura de sequencias ou CRC
errado descartam o frame.

## Host PC do backend CRSF alternativo

`crsf-host/crsf_match.py` contem um modulo independente para:

- quantizar/dequantizar `vx`, `vy`, `omega`;
- montar os 16 canais dos robos A/B/C;
- gerar frame CRSF `RC_CHANNELS_PACKED`;
- parsear stream CRSF para testes;
- validar o assembler experimental de TeamFrame.

Esse modulo nao usa nem altera o protocolo Quad-MD existente.

Para o primeiro teste sem ROS2:

```bash
python -m pip install -r crsf-host/requirements.txt
python crsf-host/crsf_bridge.py --port /dev/ttyUSB0 --baud 921600 --test
```

O bridge sempre inicia transmitindo `CH16` desligado por um curto intervalo. No
modo `--test`, depois desse intervalo ele envia:

```text
Robot A: vx = +1.0, vy = 0, omega = 0
Robot B: vx = 0, vy = +1.0, omega = 0
Robot C: vx = 0, vy = 0, omega = +1.0
```

Para manter/parar todos os robos com enable global desligado:

```bash
python crsf-host/crsf_bridge.py --port /dev/ttyUSB0 --baud 921600 --disable
```

`--rate` controla a frequencia do loop de envio; o default e `333` frames/s e o
loop usa `time.perf_counter()` para reduzir deriva. Com `--stdin-json`, a leitura
de stdin roda separada do loop CRSF; o bridge continua transmitindo o ultimo
comando valido enquanto aguarda nova entrada. Se nenhum JSON valido chegar antes
de `--command-timeout-ms` (default `100`), o bridge desliga `CH16` e envia os
tres robos parados com brake. Ao encerrar, o bridge envia `--shutdown-safe-frames`
frames seguros (default `20`) antes de fechar a serial.

## Pontos que exigem teste fisico

- Confirmar AirPort transparente em 9600 baud no AION Nano TX e nos tres RX.
- Confirmar que os tres RX entregam exatamente o mesmo D1 e que cada STM32 usa
  somente o slot correspondente ao seu `robot_id`.
- Verificar com analisador logico que `PA2` permanece inativo sem E0/E2 e que
  transmite somente E1/E3 nos tempos previstos quando solicitado.
- Medir a taxa sustentavel do frame de 32 bytes; iniciar em 20 Hz e ajustar sem
  ultrapassar o limite fisico proximo de 30 Hz em 9600/8N1.
- Medir jitter/perda real e ajustar `MATCH_AIRPORT_COMMAND_TIMEOUT_MS` se 100 ms
  nao der margem adequada ao enlace.
- Validar o sentido fisico de `vx`, `vy` e `omega` em cada robo.
- Para o backend alternativo CRSF, confirmar 420000 baud e 333 Hz Full
  Resolution / 16ch Rate/2.
- Integrar o evento de kick ao hardware do kicker quando esse atuador estiver
  disponivel no firmware.
