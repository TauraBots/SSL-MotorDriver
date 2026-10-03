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
ROS2 / PC
  |
  v
CRSF TX bridge
  |
  v
ELRS TX normal
  |
  +-- RX A -> STM32 A
  +-- RX B -> STM32 B
  +-- RX C -> STM32 C
```

Match mode usa ExpressLRS normal com CRSF `RC_CHANNELS_PACKED`. Todos os RX
compartilham a mesma binding phrase e recebem os mesmos 16 canais. Cada STM32
usa seu `robot_id` salvo para escolher apenas o bloco de canais correspondente.
A telemetria ELRS/Quad-MD fica desabilitada nesse modo.

## Configuracao

O padrao de build permanece em bench:

```c
#define TAURA_COMM_MODE COMM_MODE_BENCH
```

Para firmware de partida, compilar com:

```c
#define TAURA_COMM_MODE COMM_MODE_MATCH
#define TAURA_MATCH_TRANSPORT MATCH_TRANSPORT_CHANNELS
```

Os bauds da `USART2` sao escolhidos automaticamente por `TAURA_COMM_MODE`:

```text
quadmd_bench: USART2 = 9600 baud, 8N1
quadmd_match: USART2 = 420000 baud, 8N1
```

O arquivo `YahBoom-4ch.ioc` permanece com `USART2.BaudRate=9600`. Esse e o
valor base do CubeMX e faz com que `MX_USART2_UART_Init()` continue sendo codigo
gerado normal e compativel com o perfil bench. Depois que todos os perifericos
sao inicializados, `main()` chama `ApplyCommModeUartBaud()` dentro de um bloco
`USER CODE`. A funcao tambem esta em bloco protegido e:

- mantem `9600` sem reinicializacao no build bench;
- troca `huart2.Init.BaudRate` para `420000` e chama `HAL_UART_Init()` novamente
  no build match, antes de `SerialService_Init()` iniciar DMA/recepcao.

Assim, uma nova geracao pode reescrever a linha de baud dentro de
`MX_USART2_UART_Init()` para `9600`, conforme o `.ioc`, sem perder a selecao de
match. Nao mova a selecao dinamica de volta para essa linha gerada.

Para verificar em firmware, coloque um breakpoint depois de
`ApplyCommModeUartBaud()` e inspecione `huart2.Init.BaudRate`. O valor deve ser
`9600` com `TAURA_COMM_MODE=COMM_MODE_BENCH` e `420000` com
`TAURA_COMM_MODE=COMM_MODE_MATCH`. Uma verificacao eletrica equivalente pode
ser feita medindo `PA2` (USART2 TX) com analisador logico configurado para 8N1.

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

quadmd_match
  symbols:
    USE_HAL_DRIVER
    STM32F103xE
    TAURA_COMM_MODE=COMM_MODE_MATCH
    TAURA_MATCH_TRANSPORT=MATCH_TRANSPORT_CHANNELS
```

Em STM32CubeIDE: Project Properties -> C/C++ Build -> Settings -> MCU GCC
Compiler -> Preprocessor, duplique a configuracao existente e adicione os
simbolos acima ao perfil `quadmd_match`. Repita tambem no MCU G++ Compiler para
os arquivos C++.

## 333 Full / 16ch Rate/2

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
seguro e nao mantem o ultimo comando indefinidamente.

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
```

TeamFrame e experimental e nao substitui o modo 333 Full. A ideia e usar CH1-CH4
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

## Host PC

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

- Confirmar baud/configuracao UART exata esperada pelo RX ELRS em modo CRSF.
- Validar o sentido fisico de `vx`, `vy` e `omega` em cada robo.
- Confirmar que o TX esta em 333 Hz Full Resolution / 16ch Rate/2.
- Medir jitter/perda real e ajustar `MATCH_COMMAND_TIMEOUT_MS` se 50 ms ficar
  agressivo para o enlace configurado.
- Integrar o evento de kick ao hardware do kicker quando esse atuador estiver
  disponivel no firmware.
