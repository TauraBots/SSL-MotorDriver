import serial
import time

def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# Discovery request:
# SOF 55 AA
# TYPE E2
# VERSION 01
# SEQUENCE 0001
payload = bytes([
    0x55, 0xAA,
    0xE2,
    0x01,
    0x01, 0x00
])

crc = crc16(payload)

frame = payload + crc.to_bytes(2, "little")

print("Enviando:", frame.hex(" "))

ser = serial.Serial("COM4", 9600, timeout=0.1)

for i in range(5):
    ser.write(frame)
    ser.flush()
    print("E2 enviado", i + 1)
    time.sleep(0.2)

time.sleep(1)

rx = ser.read(ser.in_waiting)

print("RX:", rx.hex(" "))

ser.close()