"""Build the actual Keil target with ARM Compiler 5, without launching uVision.

Usage: python tools/build_firmware.py --toolchain E:/Keil5/Core/ARM/ARMCC/bin
Outputs: build/firmware/SuperCar2.{axf,hex,map} and build.log
"""
import argparse
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--toolchain', default='E:/Keil5/Core/ARM/ARMCC/bin')
    args = parser.parse_args()
    tc = Path(args.toolchain)
    for exe in ('armcc.exe', 'armasm.exe', 'armlink.exe', 'fromelf.exe'):
        if not (tc / exe).is_file():
            parser.error(f'Compiler missing: {tc / exe}')
    out = ROOT / 'build' / 'firmware'
    out.mkdir(parents=True, exist_ok=True)
    target = ET.parse(ROOT / 'MDK-ARM' / 'SuperCar2.uvprojx').find('.//Target')
    controls = target.find('.//Cads/VariousControls')
    includes = controls.findtext('IncludePath').split(';')
    defines = controls.findtext('Define').split(',')
    objects = []
    with (out / 'build.log').open('w', encoding='utf-8') as log:
        def run(command):
            log.write('$ ' + subprocess.list2cmdline([str(x) for x in command]) + '\n')
            proc = subprocess.run([str(x) for x in command], cwd=ROOT / 'MDK-ARM',
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            text = proc.stdout.decode('utf-8', errors='replace')
            log.write(text)
            log.write(f'Exit code: {proc.returncode}\n')
            log.flush()
            print(text, end='')
            if proc.returncode:
                raise SystemExit(proc.returncode)
        for f in target.findall('./Groups/Group/Files/File'):
            source = (ROOT / 'MDK-ARM' / f.findtext('FilePath')).resolve()
            typ = f.findtext('FileType')
            if typ not in ('1', '2'):
                continue
            obj = out / (source.stem + '.o')
            if typ == '1':
                run([tc / 'armcc.exe', '--cpu', 'Cortex-M3', '--c99', '-O2',
                     '--split_sections', '--apcs=interwork', '-g',
                     *['-I' + str((ROOT / 'MDK-ARM' / inc).resolve()) for inc in includes],
                     *['-D' + d for d in defines], '-c', source, '-o', obj])
            else:
                run([tc / 'armasm.exe', '--cpu', 'Cortex-M3', '--apcs=interwork',
                     '-g', source, '-o', obj])
            objects.append(obj)
        scatter = out / 'SuperCar2.sct'
        scatter.write_text('LR_IROM1 0x08000000 0x10000 {\n'
                           '  ER_IROM1 0x08000000 0x10000 {\n'
                           '    *.o (RESET, +First)\n    *(InRoot$$Sections)\n'
                           '    .ANY (+RO)\n  }\n'
                           '  RW_IRAM1 0x20000000 0x5000 { .ANY (+RW +ZI) }\n}\n',
                           encoding='ascii')
        run([tc / 'armlink.exe', '--cpu', 'Cortex-M3', '--strict', '--scatter', scatter,
             '--map', '--info=sizes', '--list', out / 'SuperCar2.map',
             '-o', out / 'SuperCar2.axf', *objects])
        run([tc / 'fromelf.exe', '--i32combined', '--output', out / 'SuperCar2.hex',
             out / 'SuperCar2.axf'])
    print('Built:', out / 'SuperCar2.hex')

if __name__ == '__main__':
    main()
