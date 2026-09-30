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

Nao houve alteracao de pinagem. A placa documentada possui `USART2` em
`PA2/PA3` com DMA; nenhum segundo UART foi configurado sem conflito no `.ioc`.
Assim, bench e match sao modos alternativos sobre a UART existente.

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

## Pontos que exigem teste fisico

- Confirmar baud/configuracao UART exata esperada pelo RX ELRS em modo CRSF.
- Validar o sentido fisico de `vx`, `vy` e `omega` em cada robo.
- Confirmar que o TX esta em 333 Hz Full Resolution / 16ch Rate/2.
- Medir jitter/perda real e ajustar `MATCH_COMMAND_TIMEOUT_MS` se 50 ms ficar
  agressivo para o enlace configurado.
- Integrar o evento de kick ao hardware do kicker quando esse atuador estiver
  disponivel no firmware.
