"""把本机Keil设备包打包供pyOCD使用；排除不可执行CCM作为算法默认RAM。"""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('不覆盖已有设备包')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.output, 'w', ZIP_DEFLATED) as archive:
        for file in args.source.rglob('*'):
            if not file.is_file() or file.suffix.lower() not in ('.pdsc', '.flm', '.svd'):
                continue
            data = file.read_bytes()
            if file.suffix.lower() == '.pdsc':
                root = ET.fromstring(data)
                for memory in root.iter('memory'):
                    if int(memory.get('start', '0'), 0) == 0x10000000:
                        memory.set('default', '0')
                data = ET.tostring(root, encoding='utf-8', xml_declaration=True)
            archive.writestr(file.relative_to(args.source).as_posix(), data)
    print(args.output)


if __name__ == '__main__':
    main()
