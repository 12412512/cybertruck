"""恢复 CubeMX 生成后的 Keil 工具链和手写文件清单，不改外设源码。"""
from pathlib import Path
import json
import sys
import xml.etree.ElementTree as ET
from keil_policy import enforce, save, OPTIONS

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "firmware/MDK-ARM/Car.uvprojx"


def put(parent, path, value):
    node = parent
    for name in path.split("/"):
        child = node.find(name)
        node = child if child is not None else ET.SubElement(node, name)
    node.text = str(value)


def sync():
    profile = json.loads((ROOT / 'config/base_trials.json').read_text(encoding='utf-8'))
    addresses = profile['addresses']
    if (len(addresses) != 4 or len(set(addresses)) != 4
            or any(type(value) is not int or not 1 <= value <= 255 for value in addresses)):
        raise ValueError('启动停止地址必须与唯一试验配置一致')
    header = ('/* 由sync_project.py从config/base_trials.json生成；勿手改。 */\n'
              '#ifndef ADDRESSES_H\n#define ADDRESSES_H\n'
              '#define BASE_SAFE_ADDRESSES {' + ', '.join(str(v) for v in addresses) + '}\n#endif\n')
    (ROOT / 'firmware/User/Config/addresses.h').write_text(header, encoding='utf-8')
    tree = ET.parse(PROJECT)
    targets = tree.findall("./Targets/Target")
    if len(targets) != 1 or targets[0].findtext("TargetName") != "Car":
        raise ValueError("必须只有一个 Car Target")
    target = targets[0]
    # 工具链元数据紧跟 ToolsetName，兼容本机 Keil MDK 5.24。
    for tag, value in (("pCCUsed", r"6070000::V6.7::.\ARMCLANG"), ("uAC6", "1")):
        existing = target.find(tag)
        if existing is not None:
            target.remove(existing)
        node = ET.Element(tag)
        node.text = value
        target.insert(list(target).index(target.find("TargetOption")), node)
    common = target.find("./TargetOption/TargetCommonOption")
    put(common, "Device", "STM32F407ZG")
    put(common, "Cpu", 'IRAM(0x20000000,0x20000) IROM(0x08000000,0x100000) CPUTYPE("Cortex-M4") FPU2 CLOCK(120000000) ELITTLE')
    put(common, "OutputDirectory", ".\\Objects\\")
    put(common, "ListingPath", ".\\Listings\\")
    put(common, "OutputName", "Car")
    put(common, "CreateHexFile", "1")
    # 构建不执行用户命令、不下载、不自动进入调试。
    for node in target.iter():
        if node.tag.startswith("RunUserProg") or node.tag in (
            "RunDebugAfterBuild", "UpdateFlashBeforeDebugging", "LoadApplicationAtStartup", "RunToMain"
        ):
            node.text = "0"
    ads = target.find("./TargetOption/TargetArmAds")
    for key, value in {"v6Lang": 3, "v6LangP": 3, "vShortEn": 1,
                       "vShortWch": 1, "v6Lto": 0, "v6WtE": 0, "v6Rtti": 0,
                       "wLevel": 3}.items():
        put(ads, "Cads/" + key, value)
    put(ads, "Aads/uClangAs", "0")
    for region, kind, address, size in (
        ("IRAM", 0, "0x20000000", "0x20000"),
        ("IROM", 1, "0x08000000", "0x100000"),
        ("OCR_RVCT4", 1, "0x08000000", "0x100000"),
        ("OCR_RVCT9", 0, "0x20000000", "0x20000"),
    ):
        for key, value in (("Type", kind), ("StartAddress", address), ("Size", size)):
            put(ads, "ArmAdsMisc/OnChipMemories/" + region + "/" + key, value)
    include = ads.find("Cads/VariousControls/IncludePath")
    paths = [p for p in (include.text or "").split(";") if p and not p.startswith("../User/")]
    paths.extend("../User/" + name for name in ("App", "Board", "Comm", "Device", "Control", "Config"))
    include.text = ";".join(paths)
    groups = target.find("Groups")
    for group in list(groups):
        if group.findtext("GroupName", "").startswith("User/"):
            groups.remove(group)
    for folder in sorted((ROOT / 'firmware/User').iterdir()):
        sources = sorted(p for p in folder.glob('*') if p.suffix in ('.c', '.h'))
        if not sources:
            continue
        group = ET.SubElement(groups, "Group")
        put(group, "GroupName", "User/" + folder.name)
        files = ET.SubElement(group, "Files")
        for source in sources:
            entry = ET.SubElement(files, "File")
            put(entry, "FileName", source.name)
            put(entry, "FileType", 1 if source.suffix == '.c' else 5)
            put(entry, "FilePath", "../User/" + folder.name + '/' + source.name)
    options = ET.parse(OPTIONS) if OPTIONS.is_file() else None
    enforce(tree.getroot(), None if options is None else options.getroot())
    save(tree, PROJECT)
    if options is not None:
        save(options, OPTIONS)
    print("PASS: Keil ARM Compiler 6.7、单 Target 和按职责分组的手写文件已同步")


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sync()
