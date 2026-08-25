# Protocolo serial

## Interface

- Porta: `USART2`
- Baud rate: `9600`
- Formato: `8N1`
- RX com `DMA` circular

## Comando recebido

O comando e um frame binario empacotado de `19 bytes`. Todos os campos
multibyte usam little-endian.

Layout:

| Offset | Tamanho | Campo |
| --- | --- | --- |
| 0 | 2 | header `0xAA55` (bytes `55 AA`) |
| 2 | 1 | `uint8_t robot_id`, ASCII `A..Z` ou `*` para broadcast |
| 3 | 4 | `uint32_t sequence` |
| 7 | 2 | `int16_t motor1` em RPM |
| 9 | 2 | `int16_t motor2` em RPM |
| 11 | 2 | `int16_t motor3` em RPM |
| 13 | 2 | `int16_t motor4` em RPM |
| 15 | 1 | `uint8_t kick_power`, limitado a `0..100` |
| 16 | 1 | `uint8_t brake`, zero para coast e nao-zero para brake |
| 17 | 2 | `CRC16-CCITT-FALSE` dos bytes `0..16` |

Pacotes destinados a outro robo sao descartados antes de atualizar comandos ou
watchdog. O ID `*` transmite em broadcast.

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

O `SET_ID` e aceito apenas pelo UID exato e pela chave de configuracao. Antes de
gravar a Flash, o firmware forca `COMM_LOST`, zera os quatro motores, ativa o
freio e limpa os estados integrais dos PIDs. As respostas de descoberta usam
atraso curto derivado do UID e do nonce para reduzir colisoes quando varias
placas compartilham o meio.

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
renovam o watchdog. Sem uma nova sequencia por 150 ms, os quatro setpoints sao
zerados, o freio e ativado, os estados dos PIDs sao limpos e `kick_power` volta
a zero. O kicker ainda nao possui acionamento fisico neste firmware.

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

A resposta completa possui 42 bytes e e enviada por DMA apos turnaround de 3
ms. O firmware limita respostas a no maximo 10 Hz.

| Campo | Tipo | Observacao |
| --- | --- | --- |
| `sof` | `uint16_t` | valor `0xAA55` |
| `type` | `uint8_t` | `0xE1` |
| `version` | `uint8_t` | `1` |
| `robot_id` | `uint8_t` | origem da resposta |
| `flags` | `uint8_t` | eco da requisicao |
| `status` | `uint8_t` | `1` para resposta valida |
| `request_sequence` | `uint16_t` | eco da requisicao |
| `time_ms` | `uint32_t` | tempo de `HAL_GetTick()` |
| `command_sequence` | `uint32_t` | ultimo comando aceito |
| `rpm1_x10` | `int16_t` | RPM de `M1` multiplicado por 10 |
| `rpm2_x10` | `int16_t` | RPM de `M2` multiplicado por 10 |
| `rpm3_x10` | `int16_t` | RPM de `M3` multiplicado por 10 |
| `rpm4_x10` | `int16_t` | RPM de `M4` multiplicado por 10 |
| `cmd1` | `int16_t` | comando interno de `M1` |
| `cmd2` | `int16_t` | comando interno de `M2` |
| `cmd3` | `int16_t` | comando interno de `M3` |
| `cmd4` | `int16_t` | comando interno de `M4` |
| `battery_mv` | `uint16_t` | tensao em milivolts |
| `battery_adc` | `uint16_t` | leitura crua do ADC |
| `brake` | `uint8_t` | `0` coast, `1` brake |
| `communication_ok` | `uint8_t` | `1` OK, `0` LOST/sem comando |
| `kick_power` | `uint8_t` | potencia do ultimo comando novo, `0..100` |
| `crc` | `uint16_t` | CRC16-CCITT-FALSE |

## Mensagem de boot

Ao iniciar, a placa transmite:

```text
USART2 READY\r\n
```

## Observacoes

- A recepcao descarta qualquer frame com CRC invalido.
- Respostas de telemetria usam a fila de TX por DMA.
- Em 9600 baud, recomenda-se polling de 5 Hz e comandos de 20 Hz.
