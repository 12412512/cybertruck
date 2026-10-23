"""只读核对基础工程配置；不编译、不启动设备、不写入工程。"""

import json
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "firmware"
ERRORS = []
CHECKS = 0


def check(ok, message):
    """累计明确的检查结果，尽量一次列出全部问题。"""
    global CHECKS
    CHECKS += 1
    if not ok:
        ERRORS.append(message)


def read(path):
    try:
        return path.read_text(encoding="utf-8-sig")
    except (OSError, UnicodeError) as exc:
        check(False, f"无法读取 {path.relative_to(ROOT)}：{exc}")
        return ""


def compact(value):
    return re.sub(r"\s+", "", value)


def clean_c(source):
    """删除注释，保留字符串，避免把注释里的示例当成执行代码。"""
    pattern = r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|/\*.*?\*/|//[^\n]*'
    return re.sub(pattern, lambda m: " " if m[0].startswith(("/*", "//")) else m[0],
                  source, flags=re.S)


def body(source, name):
    """按配对花括号提取 C 函数体，不依赖生成器的缩进。"""
    match = re.search(r"\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", source)
    if not match:
        check(False, f"缺少函数定义：{name}")
        return ""
    start = match.end()
    depth = 1
    for pos in range(start, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[start:pos]
    check(False, f"函数花括号不完整：{name}")
    return ""


def assignment(source, name, expected, label):
    values = re.findall(r"\b" + re.escape(name) + r"\s*=\s*([^;]+);", source)
    check(bool(values) and all(compact(v) == compact(str(expected)) for v in values),
          f"{label}：{name} 应为 {expected}，实际 {values or '缺失'}")


def call(source, name, arguments, label):
    pattern = r"\b" + re.escape(name) + r"\s*\(([^;()]*)\)"
    found = [compact(m) for m in re.findall(pattern, source)]
    check(compact(arguments) in found, f"{label}：缺少 {name}({arguments})")


def load_ioc():
    data = {}
    for line in read(FW / "Car.ioc").splitlines():
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.replace(r"\ ", " ").replace(r"\:", ":")
        check(key not in data, f"IOC 重复配置项：{key}")
        data[key] = value.replace(r"\:", ":")
    return data


def verify_ioc(data):
    expected = {
        "Mcu.CPN": "STM32F407ZGT6", "Mcu.Package": "LQFP144",
        "RCC.HSI_VALUE": "16000000", "RCC.PLLSourceVirtual": "RCC_PLLSOURCE_HSI",
        "RCC.PLLM": "16", "RCC.PLLN": "240", "RCC.PLLQ": "5",
        "RCC.SYSCLKSource": "RCC_SYSCLKSOURCE_PLLCLK",
        "RCC.SYSCLKFreq_VALUE": "120000000", "RCC.HCLKFreq_Value": "120000000",
        "RCC.APB1CLKDivider": "RCC_HCLK_DIV4", "RCC.APB1Freq_Value": "30000000",
        "RCC.APB2CLKDivider": "RCC_HCLK_DIV2", "RCC.APB2Freq_Value": "60000000",
        "CAN1.Prescaler": "4", "CAN1.BS1": "CAN_BS1_12TQ",
        "CAN1.BS2": "CAN_BS2_2TQ", "CAN1.NART": "DISABLE",
        "TIM1.Prescaler": "119", "TIM1.Period": "19999",
        "PC0.Signal": "GPIO_Input", "PC0.GPIO_PuPd": "GPIO_PULLUP",
        "PB0.Signal": "GPIO_Output", "PB0.PinState": "GPIO_PIN_SET",
        "PB0.GPIO_ModeDefaultOutputPP": "GPIO_MODE_OUTPUT_PP",
        "PA13.Signal": "SYS_JTMS-SWDIO", "PA14.Signal": "SYS_JTCK-SWCLK",
        "ProjectManager.ProjectName": "Car", "ProjectManager.ProjectFileName": "Car.ioc",
        "ProjectManager.StackSize": "0x1000", "ProjectManager.KeepUserCode": "true",
    }
    pins = {
        "PA9": "USART1_TX", "PA10": "USART1_RX", "PA2": "USART2_TX",
        "PA3": "USART2_RX", "PC6": "USART6_TX", "PC7": "USART6_RX",
        "PA11": "CAN1_RX", "PA12": "CAN1_TX", "PE9": "S_TIM1_CH1",
        "PE11": "S_TIM1_CH2", "PE13": "S_TIM1_CH3",
    }
    expected.update({f"{pin}.Signal": signal for pin, signal in pins.items()})
    for number in (1, 2, 6):
        for field, value in {"BaudRate": "115200", "WordLength": "WORDLENGTH_8B",
                             "StopBits": "STOPBITS_1", "Parity": "PARITY_NONE"}.items():
            expected[f"USART{number}.{field}"] = value
    for channel in (1, 2, 3):
        expected[f"TIM1.Channel-PWM Generation{channel} CH{channel}"] = f"TIM_CHANNEL_{channel}"
        expected[f"TIM1.Pulse-PWM Generation{channel} CH{channel}"] = "0"
    for key, value in expected.items():
        check(data.get(key) == value, f"IOC {key} 应为 {value}，实际 {data.get(key, '缺失')}")
    check(data.get("RCC.PLLP", "RCC_PLLP_DIV2") in ("2", "RCC_PLLP_DIV2"),
          "IOC PLLP 应为 2；未显式保存时采用芯片默认值并核对生成代码")
    active_ips = {v for k, v in data.items() if re.fullmatch(r"Mcu\.IP\d+", k)}
    check(active_ips == {"CAN1", "NVIC", "RCC", "SYS", "TIM1", "USART1", "USART2", "USART6"},
          f"IOC 启用外设不符：{sorted(active_ips)}")
    check(not any(k.startswith("PC1.") for k in data), "IOC 不得分配 PC1")
    check(not any("CH4" in k or (k.endswith(".Signal") and "CH4" in v)
                  for k, v in data.items()), "IOC 不得配置 TIM1 CH4")
    try:
        rate = int(data["RCC.APB1Freq_Value"]) / int(data["CAN1.Prescaler"]) / 15
        check(rate == 500000, f"IOC CAN 计算波特率错误：{rate}")
        check(abs(float(data.get("CAN1.CalculateBaudRate", rate)) - rate) <= 1,
              "IOC CAN 显示波特率与时钟计算不一致")
    except (KeyError, ValueError, ZeroDivisionError):
        check(False, "IOC 无法计算 CAN 波特率")


def gpio_configs(source, defines):
    """顺序读取生成器设置的 GPIO 字段及每次 HAL_GPIO_Init 调用。"""
    state = {}
    result = []
    pattern = r"GPIO_InitStruct\.(\w+)\s*=\s*([^;]+);|HAL_GPIO_Init\s*\(\s*(\w+)\s*,\s*&GPIO_InitStruct\s*\)"
    for match in re.finditer(pattern, source):
        if match[1]:
            state[match[1]] = compact(match[2])
        else:
            port = defines.get(match[3], match[3])
            pins = {defines.get(pin, pin) for pin in state.get("Pin", "").split("|")}
            result.append((port, pins, state.copy(), match.start()))
    return result


def verify_sources(data):
    names = ("main", "can", "tim", "usart", "gpio", "stm32f4xx_it")
    original = {name: read(FW / "Core/Src" / f"{name}.c") for name in names}
    sources = {name: clean_c(source) for name, source in original.items()}
    header = clean_c(read(FW / "Core/Inc/main.h"))
    defines = dict(re.findall(r"^\s*#define\s+(\w+)\s+(\w+)\s*$", header, re.M))
    clock = body(sources["main"], "SystemClock_Config")
    for name, value in {
        "RCC_OscInitStruct.OscillatorType": "RCC_OSCILLATORTYPE_HSI",
        "RCC_OscInitStruct.HSIState": "RCC_HSI_ON",
        "RCC_OscInitStruct.PLL.PLLState": "RCC_PLL_ON",
        "RCC_OscInitStruct.PLL.PLLSource": "RCC_PLLSOURCE_HSI",
        "RCC_OscInitStruct.PLL.PLLM": data.get("RCC.PLLM", "16"),
        "RCC_OscInitStruct.PLL.PLLN": data.get("RCC.PLLN", "240"),
        "RCC_OscInitStruct.PLL.PLLP": "RCC_PLLP_DIV2",
        "RCC_OscInitStruct.PLL.PLLQ": data.get("RCC.PLLQ", "5"),
        "RCC_ClkInitStruct.SYSCLKSource": "RCC_SYSCLKSOURCE_PLLCLK",
        "RCC_ClkInitStruct.AHBCLKDivider": "RCC_SYSCLK_DIV1",
        "RCC_ClkInitStruct.APB1CLKDivider": "RCC_HCLK_DIV4",
        "RCC_ClkInitStruct.APB2CLKDivider": "RCC_HCLK_DIV2",
    }.items():
        assignment(clock, name, value, "生成时钟")
    call(clock, "HAL_RCC_ClockConfig", "&RCC_ClkInitStruct,FLASH_LATENCY_3", "生成时钟")
    for name, value in {
        "hcan1.Init.Prescaler": data.get("CAN1.Prescaler", "4"),
        "hcan1.Init.TimeSeg1": data.get("CAN1.BS1", "CAN_BS1_12TQ"),
        "hcan1.Init.TimeSeg2": data.get("CAN1.BS2", "CAN_BS2_2TQ"),
        "hcan1.Init.AutoRetransmission": "DISABLE",
    }.items():
        assignment(body(sources["can"], "MX_CAN1_Init"), name, value, "生成 CAN")
    tim = body(sources["tim"], "MX_TIM1_Init")
    assignment(tim, "htim1.Init.Prescaler", data.get("TIM1.Prescaler", "119"), "生成 PWM")
    assignment(tim, "htim1.Init.Period", data.get("TIM1.Period", "19999"), "生成 PWM")
    assignment(tim, "sConfigOC.Pulse", "0", "生成 PWM")
    for channel in (1, 2, 3):
        call(tim, "HAL_TIM_PWM_ConfigChannel", f"&htim1,&sConfigOC,TIM_CHANNEL_{channel}", "生成 PWM")
    check(not re.search(r"\bTIM_CHANNEL_4\b", sources["tim"]), "生成 PWM 不得配置 CH4")
    for number in (1, 2, 6):
        uart = body(sources["usart"], f"MX_USART{number}_UART_Init")
        for field, value in {"BaudRate": "115200", "WordLength": "UART_WORDLENGTH_8B",
                             "StopBits": "UART_STOPBITS_1", "Parity": "UART_PARITY_NONE",
                             "Mode": "UART_MODE_TX_RX", "HwFlowCtl": "UART_HWCONTROL_NONE"}.items():
            assignment(uart, f"huart{number}.Init.{field}", value, f"生成 USART{number}")
    configs = []
    for name, function in (("gpio", "MX_GPIO_Init"), ("can", "HAL_CAN_MspInit"),
                           ("tim", "HAL_TIM_MspPostInit"), ("usart", "HAL_UART_MspInit")):
        configs.extend(gpio_configs(body(sources[name], function), defines))
    expected_gpio = [
        ("GPIOA", {"GPIO_PIN_9", "GPIO_PIN_10"}, "GPIO_MODE_AF_PP", "GPIO_AF7_USART1"),
        ("GPIOA", {"GPIO_PIN_2", "GPIO_PIN_3"}, "GPIO_MODE_AF_PP", "GPIO_AF7_USART2"),
        ("GPIOC", {"GPIO_PIN_6", "GPIO_PIN_7"}, "GPIO_MODE_AF_PP", "GPIO_AF8_USART6"),
        ("GPIOA", {"GPIO_PIN_11", "GPIO_PIN_12"}, "GPIO_MODE_AF_PP", "GPIO_AF9_CAN1"),
        ("GPIOE", {"GPIO_PIN_9", "GPIO_PIN_11", "GPIO_PIN_13"}, "GPIO_MODE_AF_PP", "GPIO_AF1_TIM1"),
        ("GPIOC", {"GPIO_PIN_0"}, "GPIO_MODE_INPUT", None),
        ("GPIOB", {"GPIO_PIN_0"}, "GPIO_MODE_OUTPUT_PP", None),
    ]
    check(len(configs) == len(expected_gpio), "生成 GPIO 初始化组数量不符，检查额外引脚")
    for port, pins, mode, alternate in expected_gpio:
        matches = [config for config in configs if config[0] == port and config[1] == pins]
        check(len(matches) == 1, f"生成 GPIO {port} {sorted(pins)} 应恰好配置一次")
        if matches:
            state = matches[0][2]
            check(state.get("Mode") == mode, f"生成 GPIO {port} {sorted(pins)} 模式不符")
            if alternate:
                check(state.get("Alternate") == alternate, f"生成 GPIO {port} {sorted(pins)} 复用不符")
            if port == "GPIOC" and pins == {"GPIO_PIN_0"}:
                check(state.get("Pull") == "GPIO_PULLUP", "生成 PC0 必须上拉")
    gpio = body(sources["gpio"], "MX_GPIO_Init")
    call(gpio, "HAL_GPIO_WritePin", "BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET", "蜂鸣器默认电平")
    check(defines.get("BUZZER_Pin") == "GPIO_PIN_0" and defines.get("BUZZER_GPIO_Port") == "GPIOB",
          "蜂鸣器标签必须对应 PB0")
    write_at = gpio.find("HAL_GPIO_WritePin")
    buzzer = [item for item in gpio_configs(gpio, defines) if item[0] == "GPIOB"]
    check(bool(buzzer) and 0 <= write_at < buzzer[0][3], "PB0 必须先置高，再配置输出")
    irq_defs = [("CAN1_TX", "can", "HAL_CAN_IRQHandler", "&hcan1", 2),
                ("CAN1_RX0", "can", "HAL_CAN_IRQHandler", "&hcan1", 2),
                ("CAN1_SCE", "can", "HAL_CAN_IRQHandler", "&hcan1", 2),
                ("USART1", "usart", "HAL_UART_IRQHandler", "&huart1", 3),
                ("USART2", "usart", "HAL_UART_IRQHandler", "&huart2", 3),
                ("USART6", "usart", "HAL_UART_IRQHandler", "&huart6", 4)]
    for name, module, handler, handle, priority in irq_defs:
        check(data.get(f"NVIC.{name}_IRQn", "").split(":")[0] == "true", f"IOC 未启用 {name} IRQ")
        call(sources[module], "HAL_NVIC_EnableIRQ", f"{name}_IRQn", "NVIC")
        call(sources[module], "HAL_NVIC_SetPriority", f"{name}_IRQn,{priority},0", "NVIC")
        call(body(sources["stm32f4xx_it"], f"{name}_IRQHandler"), handler, handle, "IRQ 入口")
    main = original["main"]
    for section, pattern in (("Includes", r'#include\s+"app\.h"'),
                             ("2", r"\bApp_Init\s*\(\s*\)\s*;"),
                             ("3", r"\bApp_Run\s*\(\s*\)\s*;")):
        match = re.search(r"/\*\s*USER CODE BEGIN " + section + r"\s*\*/(.*?)/\*\s*USER CODE END "
                          + section + r"\s*\*/", main, re.S)
        check(bool(match) and bool(re.search(pattern, clean_c(match[1]))),
              f"main USER CODE {section} 缺少最小 App 钩子")
    for name in ("App_Init", "App_Run"):
        check(len(re.findall(r"\b" + name + r"\s*\(", sources["main"])) == 1,
              f"main 中 {name} 必须且仅调用一次")


def verify_minimal_app():
    expected = {FW / "User/App/app.c", FW / "User/App/app.h"}
    actual = {path for path in (FW / "User").rglob("*") if path.suffix.lower() in (".c", ".h")}
    check(actual == expected, "User 仅允许最小 App/app.c 和 app.h，发现缺失或额外业务源码")
    app = clean_c(read(FW / "User/App/app.c"))
    for name in ("App_Init", "App_Run"):
        check(not body(app, name).strip(), f"{name} 在基础工程必须为空，不执行业务")
    definitions = re.findall(r"\b(\w+)\s*\([^;{}]*\)\s*\{", app)
    check(sorted(definitions) == ["App_Init", "App_Run"], "最小 App 不得定义额外函数")
    forbidden = (r"\bHAL_CAN_(?:Start|AddTxMessage|ActivateNotification)\s*\(",
                 r"\bHAL_TIM(?:Ex)?_\w*Start\w*\s*\(",
                 r"\bHAL_UART(?:Ex)?_(?:Receive\w*|Transmit\w*)\s*\(",
                 r"\bLL_(?:CAN|TIM|USART)_\w+\s*\(")
    for folder in (FW / "Core", FW / "User"):
        for path in folder.rglob("*"):
            if path.suffix.lower() not in (".c", ".h"):
                continue
            source = clean_c(read(path))
            for pattern in forbidden:
                match = re.search(pattern, source)
                check(match is None, f"基础工程禁止启动通信或输出：{path.relative_to(ROOT)} {match[0] if match else ''}")


def verify_keil():
    project = FW / "MDK-ARM/Car.uvprojx"
    try:
        xml = ET.fromstring(read(project))
    except ET.ParseError as exc:
        check(False, f"Keil XML 无法解析：{exc}")
        return
    targets = xml.findall("./Targets/Target")
    check(len(targets) == 1, "Keil 必须只有一个 Target")
    if not targets:
        return
    target = targets[0]
    check(target.findtext("TargetName") == "Car", "Keil Target 必须为 Car")
    check(target.findtext(".//Device") in ("STM32F407ZG", "STM32F407ZGTx"), "Keil 器件必须为 STM32F407ZG")
    check(target.findtext(".//uAC6") == "1", "Keil 必须选择 ARM Compiler 6")
    compiler = target.findtext(".//pCCUsed", "")
    check(bool(re.search(r"V?6\.7\b", compiler, re.I)), f"Keil 编译器版本应为 6.7，实际 {compiler or '未设置'}")
    for tag, directory in (("OutputDirectory", "Objects"), ("ListingPath", "Listings")):
        value = target.findtext(f".//{tag}", "").replace("\\", "/")
        check(bool(value) and (project.parent / value).resolve() == (project.parent / directory).resolve(),
              f"Keil {tag} 应指向 ./{directory}/")
    for tag in ("BeforeCompile", "BeforeMake", "AfterMake"):
        node = target.find(f".//{tag}")
        check(node is not None, f"Keil 缺少 {tag} 配置")
        if node is not None:
            for index in (1, 2):
                check(node.findtext(f"RunUserProg{index}") == "0", f"Keil {tag}.RunUserProg{index} 必须关闭")
    for tag in ("RunDebugAfterBuild", "UpdateFlashBeforeDebugging"):
        check(target.findtext(f".//{tag}") == "0", f"Keil {tag} 必须关闭")
    refs = []
    allowed_core = {"main.c", "gpio.c", "can.c", "tim.c", "usart.c", "stm32f4xx_it.c",
                    "stm32f4xx_hal_msp.c", "system_stm32f4xx.c"}
    for node in target.findall(".//FilePath"):
        value = (node.text or "").strip().replace("\\", "/")
        path = (project.parent / value).resolve()
        check(bool(value) and path.is_file(), f"Keil 文件引用不存在：{value}")
        refs.append(path)
        check(path.is_relative_to(FW.resolve()), f"Keil 不得引用新工程之外的文件：{value}")
        if path.is_relative_to((FW / "Core").resolve()):
            check(path.name in allowed_core, f"Keil 出现额外 Core 业务文件：{value}")
        if path.is_relative_to((FW / "User").resolve()):
            check(path in {(FW / "User/App/app.c").resolve(), (FW / "User/App/app.h").resolve()},
                  f"Keil 出现额外 User 业务文件：{value}")
    check(len(refs) == len(set(refs)), "Keil 存在重复文件引用")
    check(refs.count((FW / "User/App/app.c").resolve()) == 1, "Keil 必须且仅引用一次 app.c")
    include_dirs = []
    for node in target.findall(".//IncludePath"):
        for value in (node.text or "").split(";"):
            if value.strip():
                path = (project.parent / value.strip().replace("\\", "/")).resolve()
                check(path.is_dir(), f"Keil 头文件搜索路径不存在：{value}")
                include_dirs.append(path)
    check((FW / "User/App").resolve() in include_dirs, "Keil 缺少 User/App 头文件路径")
    startup = read(FW / "MDK-ARM/startup_stm32f407xx.s")
    stack = re.search(r"^\s*Stack_Size\s+EQU\s+(0x[0-9a-fA-F]+|\d+)\b", startup, re.M)
    check(bool(stack) and int(stack[1], 0) == 0x1000, "Keil 启动文件栈必须为 4 KiB")


def main():
    data = load_ioc()
    verify_ioc(data)
    verify_sources(data)
    verify_minimal_app()
    verify_keil()
    print(json.dumps({"result": "FAIL" if ERRORS else "PASS", "checks": CHECKS,
                      "errors": ERRORS, "mode": "只读静态检查；未编译、未连接硬件"},
                     ensure_ascii=False, indent=2))
    return 1 if ERRORS else 0


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
