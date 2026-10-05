"""Compile and run real application/transport code against a deterministic fake HAL."""
import argparse
import subprocess
import sys
from pathlib import Path
root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='C:/mingw64/bin/gcc.exe')
args = parser.parse_args()
out = root / 'build' / 'tests'
out.mkdir(parents=True, exist_ok=True)
sources = [p for p in (root / 'Core' / 'Src').glob('*.c')
           if p.name not in ('main.c', 'stm32f1xx_it.c',
                             'stm32f1xx_hal_msp.c', 'system_stm32f1xx.c')]
subprocess.run([args.cc, '-std=c99', '-O0', '-g', '-Wall', '-Wextra', '-Werror', '-DCAR_TEST',
                '-I' + str(root / 'tests' / 'stubs'), '-I' + str(root / 'Core' / 'Inc'),
                str(root / 'tests' / 'test_firmware.c'), *map(str, sources),
                '-o', str(out / 'test_firmware.exe')], check=True)
subprocess.run([str(out / 'test_firmware.exe')], check=True)
subprocess.run([sys.executable, '-m', 'unittest', 'discover',
                '-s', str(root / 'tests'), '-p', 'test_host_link.py', '-v'], check=True)
