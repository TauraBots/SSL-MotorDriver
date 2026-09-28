# Validação de alta taxa com ELRS/AirPort

## Objetivo e arquitetura

Este modo existe para **medir** a taxa sustentável do transporte; ele não afirma
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

O AirPort continua sendo somente um transporte serial transparente. O protocolo
binário da Quad-MD não muda: D0, E0, E1, E2, E3 e F0–F5 mantêm layout, SOF, CRC,
ID e sequências atuais. Não há MAVLink nem CRSF customizado. STM32, receptor
AirPort e lado PC devem estar configurados com baud rates compatíveis; o
Configurator não configura o firmware do AirPort.

## Perfis

| Perfil | D0 target | E0 FAST target global | FULL target por robô | Baud recomendado | Reply guard |
|---|---:|---:|---:|---:|---|
| NORMAL | 20 Hz | 5 Hz | 5 Hz | 9600 | sim |
| TEST 50/20 | 50 Hz | 20 Hz | 10 Hz | 921600 | não |
| TEST 100/50 | 100 Hz | 50 Hz | 10 Hz | 921600 | não |
| TEST 120/60 | 120 Hz | 60 Hz | 10 Hz | 921600 | não |
| VALIDATION 120 | 120 Hz | 120 Hz | 10 Hz | 921600 | não |

NORMAL é o perfil seguro e padrão e preserva a guarda legada de resposta. Os
demais perfis são experimentais e expressam apenas frequências alvo. Ao escolher
um perfil experimental desconectado, a GUI seleciona 921600. Se já houver uma
conexão em outro baud, a GUI avisa e não a altera silenciosamente.

O scheduler usa um `PreciseTimer` de 1 ms apenas para despertar a aplicação. A
decisão de envio usa `time.monotonic_ns()` e deadlines acumulados, portanto o
período de 120 Hz é 8,333333 ms e não é arredondado para um timer de 8 ms. Se o
event loop atrasar, no máximo um D0 e um E0 são enviados naquela passagem; os
deadlines omitidos são contados, sem rajada de recuperação. Quando ambos vencem
juntos, D0 é enviado primeiro. VALIDATION 120 desloca E0 em meia fase para
distribuir os bytes no tempo.

## Telemetria FAST e FULL

- FAST solicita `BASIC | MOTORS`.
- FULL solicita `BASIC | MOTORS | BATTERY | DIAGNOSTICS`.
- Em perfis divididos, um FULL **substitui** o FAST daquele ciclo. Nunca são
  enviados os dois pedidos no mesmo ciclo, então a taxa global de E0 permanece
  limitada pelo target FAST.
- O agendamento de FULL é mantido por ID de robô. O polling E0 continua em
  round-robin e independente do robô selecionado para comando.

A taxa FAST é global para o link, não por robô. Com target global de 120 E0/s,
um robô pode receber até aproximadamente 120 pedidos/s, dois recebem cerca de
60/s cada e quatro recebem cerca de 30/s cada. D0 continua destinado somente ao
robô ativo.

## Medições

O painel Diagnóstico separa sempre `target` de `measured`. As taxas medidas usam
eventos transmitidos/recebidos em uma janela monotônica limitada. Latência é
`(response_rx_time_ns - request_tx_time_ns) / 1e6`; o painel mostra última,
média, máxima e p95 da janela.

Uma solicitação sem E1 correlacionável antes de `response_timeout_ms` é uma
`telemetry response loss`, não uma alegação de perda de pacote RF. A porcentagem
usa respostas válidas e timeouts considerados na janela. Respostas tardias ou
sem request correspondente são contabilizadas como `unmatched`.

O botão **GRAVAR MÉTRICAS CSV** no painel Diagnóstico grava uma linha por segundo
em `radio_link_metrics.csv` (ou no caminho escolhido), incluindo perfil,
quantidade de robôs, targets, taxas medidas, perda de resposta, latências,
deadlines perdidos e bytes/s. Não é gravada uma linha por pacote.

## Estimativa de banda do protocolo

Para um robô em VALIDATION 120, pelos tamanhos reais atuais:

- D0: 19 bytes × 120 Hz = 2280 B/s no sentido PC → STM32;
- E0: 10 bytes × 120 Hz = 1200 B/s no sentido PC → STM32;
- E1 FAST: 34 bytes (`base 11 + BASIC 7 + MOTORS 16`);
- E1 FULL: 51 bytes (`FAST 34 + BATTERY 4 + DIAGNOSTICS 13`);
- E1 combinado: 110 × 34 + 10 × 51 ≈ 4250 B/s no sentido STM32 → PC.

Logo, a carga útil aproximada é 3480 B/s de ida e 4250 B/s de volta, antes de
considerar discovery/configuração. Esses valores são carga serial útil do
protocolo. Eles não representam framing UART, correções, encapsulamento,
retransmissões ou qualquer outro overhead RF interno do ELRS/AirPort.

Os limites atuais de RX são proporcionais a essa carga: 512 bytes por wake-up,
buffer limitado a 4096 bytes e até 32 frames processados por passagem. Um E1
FAST a 120 Hz representa cerca de 4080 B/s, muito abaixo da capacidade de
drenagem nominal desses limites, sem criar buffer ilimitado.

## Procedimento progressivo no hardware

1. Confirme o mesmo baud no PC, nos dois lados AirPort e na STM32. Comece em
   NORMAL (20/5) e observe por pelo menos alguns segundos.
2. Selecione manualmente TEST 50/20 e registre taxa real de D0, pedidos E0,
   respostas E1, perda de resposta, latência e deadlines perdidos.
3. Repita com TEST 100/50.
4. Repita com TEST 120/60.
5. Somente então selecione VALIDATION 120 e repita a medição.

Não há avanço automático entre perfis. Para cada etapa, verifique também jitter
por meio dos deadlines perdidos e exporte o CSV para comparação posterior. Se a
taxa medida ficar abaixo do target ou deadlines crescerem, registre o resultado
como limite observado do conjunto PC/Qt/serial/AirPort naquele ensaio.

## Execução

Instale as dependências com `python -m pip install -r configurator/requirements.txt`
e execute `python configurator/ssl-configurator.py app`. Selecione porta, baud e
**RADIO PROFILE** no cabeçalho; NORMAL permanece selecionado por padrão.
