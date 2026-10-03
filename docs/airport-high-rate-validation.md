# Validação de alta taxa com ELRS/AirPort

## Objetivo e arquitetura

Este modo existe para **medir** a taxa sustentável do transporte; não afirma
que o AirPort suporte 120 Hz. A arquitetura operacional ensaiada é:

```text
TAURA Framework / Configurator (PC)
             ↕ USB serial
       ELRS TX / AirPort
             ↕ RF
       ELRS RX / AirPort
             ↕ UART
          STM32 Quad-MD
```

O AirPort é somente um transporte serial transparente. O protocolo binário da
Quad-MD não muda: D0, E0, E1, E2, E3 e F0–F5 mantêm layout, SOF, CRC, ID e
sequências atuais. Não há MAVLink nem CRSF customizado.

## Três taxas diferentes

- **Serial baud** é a velocidade UART entre PC/TX e entre RX/STM32.
- **ELRS packet rate** é a frequência de pacotes no enlace RF.
- **Protocol target rate** é a frequência lógica desejada de D0, E0 e E1.

Esses valores não são equivalentes. Serial baud de 9600 não significa
telemetria a 9600 Hz. Um packet rate ELRS de 333 Hz também não significa E1 a
333 Hz. O perfil define somente targets lógicos e não configura baud nem packet
rate RF.

## Baud e capacidade do AirPort

O AirPort não transporta dados OTA na mesma velocidade de qualquer baud UART
arbitrariamente selecionado. Seu buffer serial é limitado a 64 bytes. Se a
UART fornecer dados mais rapidamente do que o enlace OTA consegue escoar, o
buffer pode encher e dados podem ser descartados.

Referência de capacidade AirPort a ser confrontada com o packet rate realmente
configurado no TX/ELRS:

| Packet rate | Máximo aproximado OTA | Baud sugerido |
|---|---:|---:|
| 25 Hz | ~62 B/s | 600 |
| 50 Hz | ~125 B/s | 1200 |
| 100 Hz | ~250 B/s | 2400 |
| 100 Hz Full Res | ~500 B/s | 4800 |
| 150 Hz | ~375 B/s | 2400 |
| 200 Hz | ~500 B/s | 4800 |
| 200 Hz Full Res | ~1000 B/s | 9600 |
| 250 Hz | ~625 B/s | 4800 |
| 333 Hz Full Res | ~1665 B/s | 9600 ou 14400 |
| 500 Hz | ~1250 B/s | 9600 |
| 1000 Hz | ~2500 B/s | 19200 |
| 1000 Hz Full Res | ~5000 B/s | 38400 |

O RX deste ensaio é 2.4 GHz. O ponto inicial é 9600 baud, já confirmado
fisicamente no BETAFPV 2.4GHz Nano RX com ExpressLRS 4.0.0 e protocolo AirPort.
Isso não identifica nem presume o packet rate RF atual; ele deve ser conferido
no TX/ELRS.

As opções 115200, 921600 e 1000000 continuam disponíveis para conexão direta,
debug e outros transportes. Em particular, o caminho independente STM32 ↔
ESP32 usado no artigo pode continuar em 921600 baud. Esse valor não é uma
recomendação para AirPort.

## Perfis lógicos

| Perfil | D0 target | E0 FAST target global | FULL target por robô | Default inicial AirPort | Reply guard |
|---|---:|---:|---:|---:|---|
| NORMAL | 20 Hz | 5 Hz | 5 Hz | 9600 | sim |
| TEST 50/20 | 50 Hz | 20 Hz | 10 Hz | 9600 | não |
| TEST 100/50 | 100 Hz | 50 Hz | 10 Hz | 9600 | não |
| TEST 120/60 | 120 Hz | 60 Hz | 10 Hz | 9600 | não |
| VALIDATION 120 | 120 Hz | 120 Hz | 10 Hz | 9600 | não |

9600 é o baud inicial atualmente configurado no AirPort; ele **não garante**
que o tráfego target de qualquer perfil caiba no enlace. Trocar o perfil não
altera o ComboBox de baud. A taxa medida é que determina o resultado.

NORMAL é o perfil seguro e padrão e preserva a guarda legada de resposta. Os
demais perfis são experimentais. O scheduler usa `PreciseTimer` de 1 ms apenas
para despertar a aplicação e decide envios com `time.monotonic_ns()` e deadlines
acumulados. Atrasos não geram rajadas; D0 tem prioridade e deadlines omitidos
são contabilizados. VALIDATION 120 desloca E0 em meia fase.

## Telemetria FAST e FULL

- FAST solicita `BASIC | MOTORS`.
- FULL solicita também `BATTERY | DIAGNOSTICS`.
- FULL substitui FAST naquele ciclo, mantendo E0 limitado ao target FAST.
- FULL é acompanhado por ID de robô.
- No AirPort multi-RX, E0 fica direcionado ao `uplink_robot_id` que respondeu
  E1/E3 mais recentemente. Sem uplink conhecido, o Configurator faz probe lento
  entre A/B/C ate receber uma resposta valida; o alvo de controle permanece
  independente do alvo de telemetria.

A taxa FAST é global. Em 120 E0/s, um robô pode receber aproximadamente 120
pedidos/s, dois recebem cerca de 60/s cada e quatro cerca de 30/s cada.

## Carga útil do protocolo

Tamanhos reais do protocolo atual:

- D0: 19 bytes;
- E0: 10 bytes;
- E1 FAST: 34 bytes (`base 11 + BASIC 7 + MOTORS 16`);
- E1 FULL: 51 bytes (`FAST 34 + BATTERY 4 + DIAGNOSTICS 13`).

Estimativa para um robô, com FULL substituindo FAST:

| Perfil | PC → robô | Robô → PC |
|---|---:|---:|
| NORMAL | 20×19 + 5×10 = **430 B/s** | 5×51 = **255 B/s** |
| TEST 50/20 | 50×19 + 20×10 = **1150 B/s** | 10×34 + 10×51 = **850 B/s** |
| TEST 100/50 | 100×19 + 50×10 = **2400 B/s** | 40×34 + 10×51 = **1870 B/s** |
| TEST 120/60 | 120×19 + 60×10 = **2880 B/s** | 50×34 + 10×51 = **2210 B/s** |
| VALIDATION 120 | 120×19 + 120×10 = **3480 B/s** | 110×34 + 10×51 = **4250 B/s** |

Isso é carga serial útil da Quad-MD. Não inclui framing UART nem overhead,
encapsulamento, correção ou retransmissão internos do ExpressLRS. A comparação
com capacidade OTA é apenas informativa e não bloqueia perfis. Enquanto o modo
RF não for informado, o Configurator mostra: **AirPort OTA capacity unknown —
validate experimentally.** Taxas medidas abaixo do target podem representar o
limite do transporte, e não necessariamente um defeito do scheduler.

## Medições e diagnóstico

O painel Diagnóstico mostra separadamente transporte, baud serial, perfil,
targets e taxas medidas de comandos, requests e responses. As taxas medidas
usam eventos reais em uma janela monotônica; o número configurado no perfil
nunca é apresentado como resultado medido.

Latência é calculada por
`(response_rx_time_ns - request_tx_time_ns) / 1e6`. Uma request sem E1
correspondente antes de `response_timeout_ms` entra em `telemetry response
loss`; isso não é chamado automaticamente de perda RF. Respostas tardias ou
sem request correspondente são contabilizadas separadamente como unmatched.

O botão **GRAVAR MÉTRICAS CSV** gera uma linha por segundo com targets, taxas
medidas, response loss, latências, deadlines perdidos e throughput observado.
Use esse arquivo para comparar cada etapa do teste progressivo.

## Prerequisite before physical AirPort test

PC/TX, RX e UART da STM32 devem usar exatamente o mesmo baud. Para o teste
AirPort atual, os três lados devem estar em 9600 baud. O firmware STM32 usado no
caminho direto com ESP32 pode estar configurado de outra forma; não misture as
duas configurações e não conecte antes de confirmar a UART da STM32.

Esta alteração não cria nem aplica um novo perfil no firmware STM32.

## Primeiro teste físico confirmado

Hardware:

```text
PC / COM4
  ↕ ELRS TX / AirPort
  ↕ RF
BETAFPV 2.4GHz Nano RX / ExpressLRS 4.0.0
  Serial Protocol = AirPort
  AirPort baud = 9600
  ↕ UART
STM32 Quad-MD
```

Configuração inicial na UI:

- Serial port: `COM4`
- Serial baud: `9600`
- Radio Profile: `NORMAL`
- Command target: 20 Hz
- Telemetry target: 5 Hz

Verifique discovery, D0, E0, E1, bateria, RPM, latência, response loss, CRC e
watchdog de comunicação. Somente após NORMAL estável, suba manualmente:

1. TEST 50/20;
2. TEST 100/50;
3. TEST 120/60;
4. VALIDATION 120.

Em cada etapa, grave métricas por alguns segundos. Não há avanço automático e
nenhum perfil afirma que o AirPort sustentará seu target.

## Execução

```powershell
python -m pip install -r configurator/requirements.txt
python configurator/ssl-configurator.py app --port COM4 --baud 9600
```

O ComboBox continua listando todas as portas detectadas; COM4 é apenas o exemplo
do hardware atual, não uma dependência do código.
