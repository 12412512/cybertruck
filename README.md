# 工创搬运小车

树莓派负责感知、规划、任务调度及网页控制台，STM32F407ZGT6 负责底盘、机构、反馈和保护。本仓库从空目录建立，旧工程仅作为后续迁移参考。

## 当前阶段

本阶段已建立目录、开发规范、CubeMX 硬件配置与最小 Keil 工程。CubeMX 实际生成及重复生成通过，574 项静态检查通过，Keil 完整重编译为 0 错误、0 警告。

当前应用只有空的 `App_Init()`、`App_Run()`。未迁入业务代码，未烧录，未触发硬件动作。版本、命令、结果及工具提示见[基础工程验收记录](docs/baseline.md)；编译通过不代表实物验收。

整机升级已进入阶段0测量准备：按[测量方法](docs/stage0_measurement.md)分批完成断电静态尺寸、只读观测和后续受控动作，使用[回填记录](docs/stage0_record.md)保存结果。旧模块按[复用依据](docs/reuse.md)核对后迁入；当前尚未完成阶段0实测出口。

## 工程入口

| 入口 | 用途 |
| --- | --- |
| [firmware/Car.ioc](firmware/Car.ioc) | 唯一 CubeMX 配置源 |
| [firmware/MDK-ARM/Car.uvprojx](firmware/MDK-ARM/Car.uvprojx) | Keil 工程，单 Target：`Car`，ARM Compiler 6.7 |
| [firmware/User/](firmware/User/) | 手写 STM32 代码，按职责划分 |
| [pi/](pi/) | 后续树莓派主程序及部署入口 |
| [config/](config/) | 后续地图、设备绑定、姿态与视觉标定 |
| [docs/](docs/) | 架构、接线、规范和验收记录 |
| [tools/](tools/) | 生成、检查与编译脚本；后续部署和诊断工具 |
| [tests/](tests/) | 后续软件测试和协议样本 |
| `output/` | 编译副产物、日志、录像和临时结果，不纳入 Git |

目录入口暂未实现时，仅保留职责说明；不使用占位逻辑模拟业务已完成。

## 生成与编译

在工程根目录使用 PowerShell 执行：

```powershell
.\tools\generate.ps1
.\tools\build.ps1
```

生成脚本先由 CubeMX 加载 `.ioc` 并生成代码，再同步 Keil 工具链与空应用文件引用，最后执行只读检查。编译脚本执行静态检查和完整重编译，不下载、不进入调试。

产物位于 `firmware/MDK-ARM/Objects/`，日志位于 `output/`，均不纳入 Git。工具路径覆盖参数与单独检查命令见[工具说明](tools/README.md)。

## 开发约定

- 开始工作前阅读根目录 [AGENTS.md](AGENTS.md)。
- 硬件连接与参数见[接线基线](docs/wiring.md)；分层与运行职责见[整体架构](docs/architecture.md)；命名和排版见[代码风格](docs/style.md)。
- CubeMX 生成文件与手写代码分离。生成区之外不手改生成代码，重新生成后必须检查差异并重新编译。
- 验证区分配置检查、编译、通信、实物动作及完整任务；每项只声明本次实际完成的范围。
- 本地 Git 保存关键节点，提交时写明验证范围并打带说明的标签；不自动推送远程。

## 后续实施顺序

基础工程 → 底盘恢复 → 机构恢复 → 树莓派服务整合 → 空载自主导航 → 单件实物闭环 → 两批完整任务 → 重复性和用时验收。

任何硬件动作都必须有对应阶段的明确授权；本次建立工程不包含这些动作。
