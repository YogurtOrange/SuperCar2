"""Idempotently add application modules after a CubeMX regeneration."""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
project = root / 'MDK-ARM' / 'SuperCar2.uvprojx'
tree = ET.parse(project)
groups = tree.find('.//Target/Groups')
name = 'Application/Car'
for group in list(groups):
    if group.findtext('GroupName') == name:
        groups.remove(group)
group = ET.SubElement(groups, 'Group')
ET.SubElement(group, 'GroupName').text = name
files = ET.SubElement(group, 'Files')
for source in sorted((root / 'Core' / 'Src').glob('*.c')):
    if source.name in ('main.c', 'stm32f1xx_it.c', 'stm32f1xx_hal_msp.c', 'system_stm32f1xx.c'):
        continue
    f = ET.SubElement(files, 'File')
    ET.SubElement(f, 'FileName').text = source.name
    ET.SubElement(f, 'FileType').text = '1'
    ET.SubElement(f, 'FilePath').text = '../Core/Src/' + source.name
# The generated project had an enabled empty post-build command.
for node in tree.findall('.//AfterMake/RunUserProg2'):
    node.text = '0'
ET.indent(tree, space='  ')
tree.write(project, encoding='UTF-8', xml_declaration=True)
print('Registered', len(files), 'application source files.')
