# 基础工程验收记录

日期：2026-10-23。范围为目录与规范、CubeMX 配置与生成、最小 Keil 工程和本地 Git 基线。未迁移业务，未烧录，未连接硬件发出动作指令。

## 工具与配置

| 项目 | 本次版本或设置 |
| --- | --- |
| CubeMX | 6.18.1-RC2，CLI 实际生成 |
| STM32 固件包 | STM32Cube FW_F4 1.28.3 |
| Keil | μVision 5.24a |
| 编译器 | ARM Compiler 6.7 |
| 芯片与 Target | STM32F407ZGT6，单 Target `Car` |
| 时钟 | HSI16 → PLL120 MHz；APB1 30 MHz、APB2 60 MHz |
| 基线标签 | `v0.1.0-base`，由本阶段提交后创建；提交 ID 以本地 Git 为准 |

外设明细见[硬件配置基线](wiring.md)。`Car.ioc` 是唯一 CubeMX 配置源；`sync_project.py` 负责恢复生成器不能保持的本机 Keil 编译器选择、空 App 引用和构建行为。

## 已完成验证

| 验证 | 结果与证据 |
| --- | --- |
| CubeMX 实际代码生成 | CLI 加载 `.ioc` 后执行 `project generate`，生成结果为 `OK` |
| 重复生成保留性 | `main.c`、`User/App/app.c`、`app.h` 的 SHA256 均不变；记录在 `output/cubemx/regenerate-check.json` |
| 配置与最小工程静态检查 | `tools/check_project.py`：574 项，`PASS`，`errors=[]` |
| Keil 完整重编译 | 重编译 Target `Car`，0 Error(s)、0 Warning(s) |
| 最小应用 | 仅空 `App_Init()`、`App_Run()`；main 在 USER CODE 区调用；无旧业务源文件 |
| 构建行为 | 构建前后用户程序、自动调试及调试前自动下载均关闭 |

编译尺寸：`Code=9172`、`RO-data=448`、`RW-data=16`、`ZI-data=5032`，单位为字节。启动栈为 4 KiB。

本次 `Car.hex` SHA256：

```text
38631548A2801C7BEDDAA51FA5C9FE2773EFF5F21FBAE6A214551AEB5D254A94
```

此摘要标识本次构建产物；换工具版本或后续修改后应重新构建、记录，不能沿用旧摘要。

## 复现与证据位置

从仓库根目录运行：

```powershell
.\tools\generate.ps1
.\tools\build.ps1
```

工具参数与只读检查用法见[工具说明](../tools/README.md)。

- 生成日志：`output/cubemx/generate.log`、`generate-errors.log`。
- 重复生成记录：`output/cubemx/regenerate-check.json`。
- 编译日志：`output/keil/build.log`。
- 产物：`firmware/MDK-ARM/Objects/Car.axf`、`Car.hex`。

原始日志和编译产物不提交 Git，仓库保存本摘要与可复现脚本。Git 节点在本地 `main` 分支提交并使用带说明标签 `v0.1.0-base` 定位，不配置远程推送。

## 工具提示与未验证项

CubeMX 加载时出现提示：`Pin17 (VP_RIF_VS_RIF1) cannot be retrieved for this MCU`。已通过只读检查本机 6.18.1-RC2 安装程序内 `com.st.microxplorer.mcu.Mcu.loadConfProperties` 的字节码定位：导入器恢复 17 个配置引脚后尝试追加 RIF 虚拟引脚，而 F407/DIE413 数据库没有该功能。当前 `.ioc`、生成代码与 Keil 工程均无 RIF。未修改工具安装或为消除提示添加无关配置；实际生成与编译分别通过。

CubeMX 启动日志另含本机第三方包及数据库提示。它们与 Keil 的零警告编译结果分开记录；本记录不声称生成器日志完全无提示。

实物接线、时钟实测、UART/CAN 通信、PWM 波形、机构动作及完整任务均未验证。CAN、PWM 与 UART 收发未启动，PB0 按配置先置高；这些为代码与配置检查结果，不是板上测量结果。
