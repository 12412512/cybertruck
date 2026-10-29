"""恢复Keil构建/调试保护项，保留用户设备包、驱动和其余工程设置。"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'firmware/MDK-ARM/Car.uvprojx'
OPTIONS = PROJECT.with_suffix('.uvoptx')
OPTION_FLAGS = ('sLdApp', 'sGomain', 'tLdApp', 'tGomain', 'sRunDeb')


def put(parent, path, value, *, missing_only=False):
    node = parent
    for name in path.split('/'):
        child = node.find(name)
        node = child if child is not None else ET.SubElement(node, name)
    if not missing_only or node.text is None:
        node.text = str(value)


def enforce(project, options=None):
    targets = project.findall('./Targets/Target')
    if len(targets) != 1 or targets[0].findtext('TargetName') != 'Car':
        raise ValueError('保护策略仅用于单Car目标')
    target = targets[0]
    local = None
    if options is not None:
        matches = [node for node in options.findall('./Target') if node.findtext('TargetName') == 'Car']
        if len(matches) != 1:
            raise ValueError('本机选项必须包含唯一Car目标')
        local = matches[0].find('TargetOption')
    put(target, 'TargetOption/TargetCommonOption/ListingPath', '.\\Listings\\')
    put(target, 'TargetOption/Utilities/Flash1/UpdateFlashBeforeDebugging', 0)
    debug = target.find('TargetOption/DebugOption')
    if debug is None:
        raise ValueError('缺少DebugOption，不能推测完整工程格式')
    # IDE可能仅留下OPTHX；补齐厂家XSD要求的子块，而不是塞入孤立标志。
    for name, use in (('Simulator', 'UseSimulator'), ('Target', 'UseTarget')):
        default = '0' if name == 'Simulator' else '1'
        if local is not None:
            default = local.findtext('DebugOpt/uSim' if name == 'Simulator' else 'DebugOpt/uTrg', default)
        put(debug, f'{name}/{use}', default, missing_only=True)
        for field in ('LoadApplicationAtStartup', 'RunToMain'):
            put(debug, f'{name}/{field}', 0)
        for field in ('RestoreBreakpoints', 'RestoreWatchpoints', 'RestoreMemoryDisplay',
                      'RestoreFunctions', 'RestoreToolbox'):
            put(debug, f'{name}/{field}', 1, missing_only=True)
        if name == 'Simulator':
            put(debug, f'{name}/LimitSpeedToRealTime', 0, missing_only=True)
    put(debug, 'RunDebugAfterBuild', 0)
    selection = '13' if local is None else local.findtext('DebugOpt/nTsel', '13')
    put(debug, 'TargetSelection', selection, missing_only=True)
    dlls = target.find('TargetOption/DllOption')
    for name, prefix in (('SimDlls', 'Sim'), ('TargetDlls', 'Target')):
        for field, source in (('CpuDll', 'DllName'), ('CpuDllArguments', 'DllArguments'),
                              ('PeripheralDll', 'DlgDll'), ('PeripheralDllArguments', 'DlgDllArguments')):
            put(debug, f'{name}/{field}', dlls.findtext(prefix + source, ''), missing_only=True)
        put(debug, f'{name}/InitializationFile', '', missing_only=True)
    driver = 'STLink\\ST-LINKIII-KEIL_SWO.dll' if local is None else local.findtext('DebugOpt/pMon', '')
    put(debug, 'TargetDlls/Driver', driver, missing_only=True)
    # 按厂家定义顺序保存，避免缺失块补在错误位置。
    order = ('OPTHX', 'Simulator', 'Target', 'RunDebugAfterBuild', 'TargetSelection', 'SimDlls', 'TargetDlls')
    debug[:] = sorted(debug, key=lambda node: order.index(node.tag) if node.tag in order else len(order))
    for node in target.iter():
        if node.tag.startswith('RunUserProg'):
            node.text = '0'
    if local is not None:
        put(local, 'OPTLEX/ListingPath', '.\\Listings\\')
        for flag in OPTION_FLAGS:
            put(local, 'DebugOpt/' + flag, 0)


def save(tree, path):
    ET.register_namespace('xsi', 'http://www.w3.org/2001/XMLSchema-instance')
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)


def repair():
    project = ET.parse(PROJECT)
    options = ET.parse(OPTIONS) if OPTIONS.is_file() else None
    enforce(project.getroot(), None if options is None else options.getroot())
    save(project, PROJECT)
    if options is not None:
        save(options, OPTIONS)
    print('PASS: Keil列表目录、自动调试/下载和本机加载/运行选项已恢复')


if __name__ == '__main__':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    repair()
