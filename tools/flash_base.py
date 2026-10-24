"""明确探针/芯片/HEX摘要后烧录；先备份，禁止自动解锁或全片擦除。"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import time

from intelhex import IntelHex
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
from pyocd.flash.file_programmer import FileProgrammer

BASE, END = 0x08000000, 0x08100000


def digest(data):
    return hashlib.sha256(data).hexdigest()


def image_check(path, expected):
    data = path.read_bytes()
    if digest(data) != expected.lower():
        raise ValueError('HEX摘要与本次批准版本不一致')
    image = IntelHex(str(path))
    segments = image.segments()
    if not segments or any(start < BASE or end > END for start, end in segments):
        raise ValueError('HEX含片外/选项字节地址，拒绝烧录')
    stack, entry = struct.unpack('<II', bytes(image.tobinarray(start=BASE, size=8)))
    if not 0x20000000 < stack <= 0x20020000 or not entry & 1 or not BASE <= (entry & ~1) < END:
        raise ValueError('初始栈或复位向量不符合STM32F407ZG')
    return image, data


def read_flash(target, start, size):
    result = bytearray()
    for offset in range(0, size, 4096):
        result.extend(target.read_memory_block8(start + offset, min(4096, size - offset)))
    return bytes(result)


def main():
    parser = argparse.ArgumentParser(description='默认只读识别；--program才执行备份/写入/逐字节读回/复位')
    parser.add_argument('--probe', required=True)
    parser.add_argument('--pack', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--program', action='store_true')
    parser.add_argument('--hex', type=Path)
    parser.add_argument('--expect-sha')
    parser.add_argument('--expect-uid')
    parser.add_argument('--expect-backup-sha', help='重试前核对Flash仍与原始备份一致')
    parser.add_argument('--frequency', type=int, default=1000000, help='SWD时钟Hz，不改变连接或擦写策略')
    args = parser.parse_args()
    if args.frequency not in (100000, 500000, 1000000):
        parser.error('SWD时钟只允许100k/500k/1MHz')
    if args.program and not all((args.hex, args.expect_sha, args.expect_uid)):
        parser.error('烧录必须提供HEX、明确SHA256和已只读确认的芯片UID')
    args.output.mkdir(parents=True, exist_ok=False)
    record = dict(probe=args.probe, target='stm32f407zg', pack=str(args.pack.resolve()),
                  auto_unlock=False, chip_erase=False, started_wall_s=time.time(), programmed=False)
    manifest = args.output / 'manifest.json'

    def save():
        manifest.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')

    options = dict(pack=str(args.pack.resolve()), target_override='stm32f407zg',
                   frequency=args.frequency, connect_mode='attach', resume_on_disconnect=False,
                   auto_unlock=False, config_file=None)
    save()
    try:
        session = ConnectHelper.session_with_chosen_probe(
            unique_id=args.probe, blocking=False, options=options)
        if session is None:
            raise RuntimeError('指定探针不存在')
        with session:
            target = session.target
            algorithms = []
            for region in target.memory_map:
                if region.is_flash and region.flash:
                    algorithm = region.flash.flash_algo
                    algorithms.append(dict(load_address=algorithm['load_address'],
                                           stack=algorithm['begin_stack']))
            record['flash_algorithms'] = algorithms
            if not algorithms or any(not 0x20000000 <= a['load_address'] < 0x20020000
                                     or not 0x20000000 < a['stack'] <= 0x20020000 for a in algorithms):
                raise RuntimeError('下载算法未放在可执行主SRAM，拒绝擦写')
            identity = target.read32(0xE0042000)
            flash_kb = target.read16(0x1FFF7A22)
            uid = bytes(target.read_memory_block8(0x1FFF7A10, 12)).hex()
            record.update(device_id=f'{identity & 0xfff:03x}', revision=f'{identity >> 16:04x}',
                          flash_kb=flash_kb, uid=uid)
            save()
            print(json.dumps({k: record[k] for k in ('device_id', 'revision', 'flash_kb', 'uid')}), flush=True)
            if identity & 0xfff != 0x413 or flash_kb != 1024:
                raise RuntimeError('实物芯片ID/容量不符合STM32F407ZG，拒绝写入')
            if not args.program:
                return
            if uid != args.expect_uid.lower():
                raise RuntimeError('芯片UID发生变化')
            image, data = image_check(args.hex, args.expect_sha)
            frozen = args.output / 'approved.hex'
            frozen.write_bytes(data)
            record['image_sha256'] = digest(data)
            target.halt()
            print('读取旧固件完整1MiB备份', flush=True)
            old = read_flash(target, BASE, END - BASE)
            (args.output / 'previous.bin').write_bytes(old)
            record['backup_sha256'] = digest(old)
            save()
            if args.expect_backup_sha and digest(old) != args.expect_backup_sha.lower():
                raise RuntimeError('当前Flash与原始备份不一致，停止重试')
            target.reset_and_halt(Target.ResetType.SYSRESETREQ)
            print('备份完成，按需擦除扇区并编程', flush=True)
            try:
                FileProgrammer(session, chip_erase='sector', smart_flash=False,
                               trust_crc=False).program(str(frozen))
                target.halt()
            except BaseException:
                target.halt()
                raise
            record['programmed'] = True
            save()
            print('逐字节读回HEX全部有效地址', flush=True)
            verified = 0
            for start, end in image.segments():
                expected = bytes(image.tobinarray(start=start, end=end - 1))
                actual = read_flash(target, start, end - start)
                if actual != expected:
                    raise RuntimeError(f'读回不一致：0x{start:08x}')
                verified += len(actual)
            record['verified_bytes'] = verified
            record['verified'] = True
            save()
            target.reset_and_halt(Target.ResetType.SYSRESETREQ)
            record['initial_sp'] = f'{target.read_core_register("sp"):08x}'
            record['initial_pc'] = f'{target.read_core_register("pc"):08x}'
            target.resume()
            record['resumed_after_verify'] = True
            save()
            print('写入/读回一致，已复位运行；仅新固件启动停车，未发送轨迹目标', flush=True)
    except BaseException as exc:
        record['error'] = f'{type(exc).__name__}: {exc}'
        save()
        raise


if __name__ == '__main__':
    main()
