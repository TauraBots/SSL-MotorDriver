#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import time

import pygame
import serial


RX_SOF0 = 0x55
RX_SOF1 = 0xAA
RX_TYPE_VEL = 0x01


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_vel_frame(vx: float, vy: float, omega: float) -> bytes:
    payload = struct.pack("<BBBBBBfff", RX_SOF0, RX_SOF1, RX_TYPE_VEL, 0, 0, 0, vx, vy, omega)
    return payload + struct.pack("<H", crc16_ccitt_false(payload))


def parse_args():
    parser = argparse.ArgumentParser(description="Pygame keyboard teleop for the YahBoom 4ch motor driver")
    parser.add_argument("--port", required=True, help="Serial port, for example COM3")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate")
    parser.add_argument("--rate-hz", type=float, default=40.0, help="Command send rate")
    parser.add_argument("--linear", type=float, default=2.0, help="Linear velocity in m/s")
    parser.add_argument("--angular", type=float, default=5.0, help="Angular velocity in rad/s")
    parser.add_argument("--accel-tau", type=float, default=0.18, help="Exponential acceleration time constant in seconds")
    parser.add_argument("--stop-tau", type=float, default=0.08, help="Exponential stop time constant in seconds")
    return parser.parse_args()


def draw(screen, font, vx: float, vy: float, omega: float, port: str, baud: int):
    screen.fill((20, 24, 28))
    lines = [
        "YahBoom teleop",
        f"{port} @ {baud}",
        "",
        "W/S: forward/back",
        "A/D: left/right",
        "Q/E: rotate",
        "Space: stop",
        "Esc or X: quit",
        "",
        f"vx    {vx:+.2f} m/s",
        f"vy    {vy:+.2f} m/s",
        f"omega {omega:+.2f} rad/s",
    ]

    y = 18
    for i, line in enumerate(lines):
        color = (235, 238, 240) if i in (0, 1, 9, 10, 11) else (165, 172, 178)
        text = font.render(line, True, color)
        screen.blit(text, (20, y))
        y += 28
    pygame.display.flip()


def main():
    args = parse_args()
    period_s = 1.0 / max(1.0, args.rate_hz)
    vx = 0.0
    vy = 0.0
    omega = 0.0
    last_update = time.monotonic()

    pygame.init()
    pygame.display.set_caption("YahBoom Teleop")
    screen = pygame.display.set_mode((360, 380))
    font = pygame.font.SysFont("consolas", 18)
    clock = pygame.time.Clock()

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        ser.write(encode_vel_frame(0.0, 0.0, 0.0))
        next_send = time.monotonic()
        running = True

        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_x):
                        running = False

                keys = pygame.key.get_pressed()

                target_vx = 0.0
                target_vy = 0.0
                target_omega = 0.0

                if keys[pygame.K_SPACE]:
                    pass
                else:
                    if keys[pygame.K_w]:
                        target_vy += args.linear
                    if keys[pygame.K_s]:
                        target_vy -= args.linear
                    if keys[pygame.K_d]:
                        target_vx += args.linear
                    if keys[pygame.K_a]:
                        target_vx -= args.linear
                    if keys[pygame.K_q]:
                        target_omega += args.angular
                    if keys[pygame.K_e]:
                        target_omega -= args.angular

                now = time.monotonic()
                dt = max(0.0, now - last_update)
                last_update = now

                target_is_zero = (target_vx == 0.0) and (target_vy == 0.0) and (target_omega == 0.0)
                tau = max(0.001, args.stop_tau if target_is_zero else args.accel_tau)
                alpha = 1.0 - math.exp(-dt / tau)

                vx += (target_vx - vx) * alpha
                vy += (target_vy - vy) * alpha
                omega += (target_omega - omega) * alpha

                if abs(vx) < 1.0e-4:
                    vx = 0.0
                if abs(vy) < 1.0e-4:
                    vy = 0.0
                if abs(omega) < 1.0e-4:
                    omega = 0.0

                if now >= next_send:
                    ser.write(encode_vel_frame(vx, vy, omega))
                    next_send = now + period_s

                draw(screen, font, vx, vy, omega, args.port, args.baud)
                clock.tick(60)
        finally:
            ser.write(encode_vel_frame(0.0, 0.0, 0.0))
            pygame.quit()
            print("Stopped")

    return 0


if __name__ == "__main__":
    sys.exit(main())
