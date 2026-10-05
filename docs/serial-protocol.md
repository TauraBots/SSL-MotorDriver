# Protocolo serial

## Interface

- Porta: `USART2`
- Baud rate: `9600`
- Formato: `8N1`
- RX com `DMA` circular

## Comando de velocidade do robô

O comando principal é um frame binário empacotado de `19 bytes`. O host envia
o movimento cartesiano desejado e o firmware calcula os quatro setpoints com
`OmniKinematics`. Todos os campos multibyte usam little-endian.

Layout:

| Offset | Tamanho | Campo |
| --- | --- | --- |
| 0 | 2 | header `0xAA55` (bytes `55 AA`) |
| 2 | 1 | tipo `0xD0` |
| 3 | 1 | versão `1` |
| 4 | 1 | `uint8_t robot_id`, ASCII `A..Z` ou `*` para broadcast |
| 5 | 4 | `uint32_t sequence` |
| 9 | 2 | `int16_t vx_mm_s`, velocidade para a direita em mm/s |
| 11 | 2 | `int16_t vy_mm_s`, velocidade para a frente em mm/s |
| 13 | 2 | `int16_t omega_mrad_s`, rotação anti-horária em mrad/s |
| 15 | 1 | `uint8_t kick_power`, limitado a `0..100` |
| 16 | 1 | `uint8_t brake`, zero para coast e nao-zero para brake |
| 17 | 2 | `CRC16-CCITT-FALSE` dos bytes `0..16` |

Pacotes destinados a outro robô são descartados antes de atualizar comandos ou
watchdog. O ID `*` transmite em broadcast. O firmware converte os valores para
SI, executa a cinemática, limita conjuntamente as rodas e então converte os
resultados para RPM. A geometria não faz parte da aplicação do host.

O frame legado de 19 bytes com `robot_id` no offset 2 e quatro RPMs nos offsets
7–14 continua aceito apenas para compatibilidade e ensaios diretos de bancada.

## Identidade fisica e configuracao persistente

Cada placa usa o UID imutavel de 96 bits gravado pela ST como identidade fisica.
O ID logico `A..Z` fica em uma estrutura versionada na ultima pagina de 2 KiB
da Flash (`0x0803F800`), protegida por CRC32 e contador de geracao.

Se a estrutura estiver vazia ou invalida, a placa fica `UNCONFIGURED` e rejeita
todos os comandos normais, inclusive broadcast. O UID continua acessivel pelo
protocolo de descoberta.

Descobrir placas:

```text
python scripts/ssl-configurator.py discover --port COM7 --baud 9600
```

Configurar o ID logico de um UID especifico, com a placa reiniciada e antes de
enviar comandos de motor:

```text
python scripts/ssl-configurator.py set-id --port COM7 --baud 9600 --uid 0123456789ABCDEF01234567 --robot-id B
```

Tipos de frame:

| Tipo | Direcao | Conteudo |
| --- | --- | --- |
| `0xF0` | host -> placas | descoberta, com nonce de 32 bits |
| `0xF1` | host -> placa | UID de 96 bits, novo ID, chave e CRC16 |
| `0xF2` | placa -> host | UID, estado configurado, ID e geracao |
| `0xF3` | placa -> host | UID, resultado, ID e geracao |
| `0xF4` | host -> placa | UID, aceleracao linear/angular, chave e CRC16 |
| `0xF5` | placa -> host | UID, resultado da gravacao, ID e geracao |

O `SET_ID` e aceito apenas pelo UID exato e pela chave de configuracao. Antes de
gravar a Flash, o firmware forca `COMM_LOST`, zera os quatro motores, ativa o
freio e limpa os estados integrais dos PIDs. As respostas de descoberta usam
atraso curto derivado do UID e do nonce para reduzir colisoes quando varias
placas compartilham o meio.

Os limites de aceleracao podem ser gravados pela aba `CONFIGURACAO DA PLACA` ou
pela linha de comando:

```text
python scripts/ssl-configurator.py set-motion --port COM7 --uid 0123456789ABCDEF01234567 --linear-accel 4.0 --angular-accel 10.0
```

O frame `0xF4` possui 29 bytes: header e tipo, UID de 12 bytes, dois `float32`
little-endian (`m/s2` e `rad/s2`), chave de configuracao e CRC16. O firmware
aceita aceleracao linear entre `0.1` e `20.0 m/s2` e angular entre `0.1` e
`50.0 rad/s2`. Antes da gravacao os motores entram em estado seguro. Os valores
sao aplicados imediatamente e novamente carregados a cada inicializacao.

## Aplicativo do host

O `scripts/ssl-configurator.py` concentra configuracao, controle e monitoramento.
Sem argumentos ele abre a interface grafica. Para
dirigir o robo `B`, mostrar a telemetria ao vivo e grava-la em CSV:

```text
python scripts/ssl-configurator.py drive --port COM7 --baud 9600 --robot-id B --log telemetry_B.csv
```

Por compatibilidade, o nome da acao `drive` pode ser omitido:

```text
python scripts/ssl-configurator.py drive --port COM7 --baud 9600 --robot-id B
```

Use `python scripts/ssl-configurator.py --help` para listar as acoes disponiveis.
A aba `ANÁLISE CSV` gera os gráficos dos arquivos gravados pelo aplicativo.

Somente uma mudanca de `sequence` torna o comando novo. Frames repetidos nao
renovam o watchdog. O D1 AirPort possui um watchdog MATCH de 120 ms; depois que
o comando chega ao AppC, permanece tambem a protecao interna de 150 ms. Em
timeout, os quatro setpoints sao zerados, o freio e ativado, os estados dos PIDs
sao limpos e `kick_power` volta a zero. O kicker ainda nao possui acionamento
fisico neste firmware.

## Telemetria enviada

A telemetria nunca e espontanea. O host envia uma requisicao direcionada de 10
bytes e somente a placa configurada com o ID exato responde. Broadcast nao e
permitido. A requisicao nao atualiza o watchdog de comandos.

Requisicao:

| Campo | Tipo | Observacao |
| --- | --- | --- |
| `header` | `uint16_t` | `0xAA55` |
| `type` | `uint8_t` | `0xE0` |
| `version` | `uint8_t` | `1` |
| `robot_id` | `uint8_t` | ID exato `A..Z` |
| `request_sequence` | `uint16_t` | correlacao com a resposta |
| `flags` | `uint8_t` | mascara solicitada |
| `crc` | `uint16_t` | CRC16 dos 8 bytes anteriores |

A resposta `0xE1` tem tamanho variável, é enviada exclusivamente por DMA após
turnaround de 3 ms e nunca atualiza o watchdog de comandos. O firmware limita
respostas a no máximo 10 Hz. Flags desconhecidas são removidas da máscara
ecoada.

Cabeçalho comum, presente em todas as respostas:

| Campo | Tipo | Observacao |
| --- | --- | --- |
| `sof` | `uint16_t` | valor `0xAA55` |
| `type` | `uint8_t` | `0xE1` |
| `version` | `uint8_t` | `1` |
| `robot_id` | `uint8_t` | origem da resposta |
| `request_sequence` | `uint16_t` | eco da requisicao |
| `status` | `uint8_t` | bit 0 válido; bits 1–4 falha dos encoders M1–M4; bit 5 subtensão |
| `flags` | `uint8_t` | máscara efetivamente incluída |

Depois do cabeçalho, os grupos aparecem na ordem abaixo quando solicitados:

| Flag | Valor | Conteúdo | Bytes |
| --- | --- | --- | --- |
| `BASIC` | `0x01` | `time_ms:u32`, `communication_ok:u8`, `brake:u8`, `kick_power:u8` | 7 |
| `MOTORS` | `0x02` | `rpm1_x10..rpm4_x10:i16`, `cmd1..cmd4:i16` | 16 |
| `BATTERY` | `0x04` | `battery_mv:u16`, `battery_adc:u16` | 4 |
| `DIAGNOSTICS` | `0x08` | `crc_errors:u32`, `received_packets:u32`, `watchdog_ok:u8`, `last_command_sequence:u32` | 13 |

O CRC16 ocupa os dois últimos bytes e cobre todo o frame anterior. Uma resposta
sem grupos possui 11 bytes; `TELEMETRY_FLAG_FULL = 0x0F` produz 51 bytes.
Solicitar somente `BATTERY`, por exemplo, produz 15 bytes e não transmite RPM
nem diagnóstico.

Exemplo de fluxo com `robot_id='C'` e `request_sequence=100`:

```text
Host  -> barramento: E0, C, sequence=100
Robô C -> host:      E1, C, sequence=100
Robôs A/B/D/E/F:     silêncio
```

ID diferente, ID inválido, broadcast `*`, versão desconhecida ou CRC inválido
são ignorados sem resposta. Requisições de telemetria não alteram setpoints,
PID, `sequence` de comando ou temporizador do watchdog.

## Mensagem de boot

Ao iniciar, a placa transmite:

```text
USART2 READY\r\n
```

## Observacoes

- A recepcao descarta qualquer frame com CRC invalido.
- Respostas de telemetria usam a fila de TX por DMA.
- Em 9600 baud, recomenda-se polling de 5 Hz e comandos de 20 Hz.
- A subtensão é confirmada após três leituras abaixo de 9,6 V e liberada acima
  de 10,2 V. Enquanto ativa, os setpoints são zerados e o freio é aplicado.
- A medição de encoder acumula contagens por 10 ms; o controle permanece em 1 kHz.
