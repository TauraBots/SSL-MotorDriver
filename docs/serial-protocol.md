# Protocolo serial

## Interface

- Porta: `USART2`
- Baud rate: `1000000`
- Formato: `8N1`
- RX com `DMA` circular

## Comando recebido

O firmware espera frames de `20 bytes`.

Layout:

| Offset | Tamanho | Campo |
| --- | --- | --- |
| 0 | 1 | `0x55` |
| 1 | 1 | `0xAA` |
| 2 | 1 | tipo |
| 3 | 3 | reservado |
| 6 | 4 | `float vx` |
| 10 | 4 | `float vy` |
| 14 | 4 | `float omega` |
| 18 | 2 | `CRC16-CCITT-FALSE` little-endian dos bytes `0..17` |

Tipo suportado:

- `0x01`: comando de velocidade do robo

Convencoes usadas na cinematica:

- `vx`: eixo +x para a direita
- `vy`: eixo +y para frente
- `omega`: rotacao anti-horaria
- raio da roda usado na conversao: `0.03 m`
- raio geometrico do robo usado na conversao: `0.09 m`

Depois da conversao, as velocidades das rodas em `rad/s` sao convertidas para `rpm` e aplicadas como setpoint dos 4 controladores.

## Telemetria enviada

A telemetria e enviada a cada `1 ms`.

O frame binario empacotado possui `30 bytes`:

| Campo | Tipo | Observacao |
| --- | --- | --- |
| `sof` | `uint16_t` | valor `0xAA55` |
| `seq` | `uint16_t` | contador incremental |
| `time_ms` | `uint32_t` | tempo de `HAL_GetTick()` |
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
| `crc8` | `uint8_t` | XOR de todos os bytes anteriores do frame |

## Mensagem de boot

Ao iniciar, a placa transmite:

```text
USART2 READY\r\n
```

## Observacoes

- A recepcao descarta qualquer frame com CRC invalido.
- O firmware hoje envia a telemetria com `HAL_UART_Transmit()` bloqueante, nao por DMA.
- Existe fila de TX por DMA implementada em `main.c`, mas a telemetria periodica ainda nao usa essa fila.
