"""Compile the actual firmware C with mocked IDF/network calls (no ESP32 required).
Run: python tests/run_web_server_host_tests.py --cc /path/to/gcc
Or:  python tests/run_web_server_host_tests.py --cc .pio/host-test-tools/ziglang/zig.exe --zig
"""
import argparse
import pathlib
import re
import subprocess

root = pathlib.Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='cc')
parser.add_argument('--zig', action='store_true')
args = parser.parse_args()
build = root / '.pio' / 'web_server_host_tests'
build.mkdir(parents=True, exist_ok=True)
parts = [(root / 'tests/web_server_test_stubs.h').read_text(encoding='utf-8-sig')]
for name in ('include/app_config.h', 'include/quadmd_protocol.h', 'include/web_server.h', 'src/web_server.c'):
    source = (root / name).read_text(encoding='utf-8-sig')
    parts.append(re.sub(r'^\s*#(?:include|pragma once)[^\n]*', '', source, flags=re.M))
parts.append((root / 'tests/web_server_host_test.c').read_text(encoding='utf-8-sig'))
source = build / 'test.c'
source.write_text('\n'.join(parts), encoding='utf-8')
exe = build / 'test.exe'
cmd = [args.cc] + (['cc'] if args.zig else [])
subprocess.run(cmd + ['-std=c11', '-Wall', '-Wextra', '-Werror', str(source), '-o', str(exe)], cwd=root, check=True)
subprocess.run([str(exe)], cwd=root, check=True)
