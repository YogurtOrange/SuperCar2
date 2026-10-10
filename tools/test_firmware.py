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
for mode in (0, 1, 2):
    executable = out / ('test_firmware.exe' if mode == 0 else f'test_firmware_single_{mode}.exe')
    print(f'Testing CAR_SINGLE_MOTOR_TEST={mode}', flush=True)
    subprocess.run([args.cc, '-std=c99', '-O0', '-g', '-Wall', '-Wextra', '-Werror',
                    '-DCAR_TEST', f'-DCAR_SINGLE_MOTOR_TEST={mode}',
                    '-I' + str(root / 'tests' / 'stubs'), '-I' + str(root / 'Core' / 'Inc'),
                    str(root / 'tests' / 'test_firmware.c'), *map(str, sources),
                    '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
subprocess.run([sys.executable, '-m', 'unittest', 'discover',
                '-s', str(root / 'tests'), '-p', 'test_*.py', '-v'], check=True)
