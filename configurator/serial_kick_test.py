import serial
import struct
import time
import sys

PORT = "COM4"       # TROQUE pela COM do STM32
BAUD = 9600
KICK_POWER = 20     # 20% primeiro
DURATION = 3.0      # mantém D1 vivo por 3 segundos
RATE_HZ = 20

FLAG_KICK = 1 << 0
FLAG_BRAKE = 1 << 2
FLAG_ENABLED = 1 << 4


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def slot(vx=0, vy=0, omega=0, kick_power=0, flags=0):
    return struct.pack(
        "<hhhBB",
        vx,
        vy,
        omega,
        kick_power,
        flags
    )


def make_d1(seq, kick=False, enabled=True):
    frame = bytearray()

    frame += bytes([
        0x55,
        0xAA,
        0xD1,
        0x01
    ])

    frame += struct.pack("<H", seq & 0xFFFF)

    # ROBÔ A
    flags_a = 0

    if enabled:
        flags_a |= FLAG_ENABLED

    if kick:
        flags_a |= FLAG_KICK

    frame += slot(
        kick_power=KICK_POWER if kick else 0,
        flags=flags_a
    )

    # ROBÔ B
    frame += slot()

    # ROBÔ C
    frame += slot()

    crc = crc16_ccitt_false(frame)

    frame += struct.pack("<H", crc)

    assert len(frame) == 32

    return frame


with serial.Serial(PORT, BAUD, timeout=0.1) as ser:

    print(f"Conectado em {PORT} @ {BAUD}")
    time.sleep(0.5)

    seq = 1
    period = 1.0 / RATE_HZ

    # Primeiro inicializa o estado do STM32 com KICK desligado
    print("Inicializando D1...")
    for _ in range(5):
        ser.write(make_d1(seq, kick=False, enabled=True))
        seq += 1
        time.sleep(period)

    # Agora cria a borda 0 -> 1
    print(f"KICK {KICK_POWER}%")
    ser.write(make_d1(seq, kick=True, enabled=True))
    seq += 1
    time.sleep(period)

    # Volta imediatamente para KICK=0
    print("Mantendo comunicação...")
    end = time.time() + DURATION

    while time.time() < end:
        ser.write(make_d1(seq, kick=False, enabled=True))
        seq += 1
        time.sleep(period)

    print("Safe stop...")

    for _ in range(10):
        ser.write(make_d1(seq, kick=False, enabled=False))
        seq += 1
        time.sleep(period)

    print("Teste concluído.")