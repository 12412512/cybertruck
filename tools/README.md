# 开发工具

本阶段提供 CubeMX 生成、Keil 同步、只读检查及完整重编译工具。没有下载、调试或业务控制命令。

## 工具职责

| 工具 | 输入与行为 | 输出与副作用 |
| --- | --- | --- |
| `generate.ps1` | CubeMX CLI 加载 `firmware/Car.ioc`，执行 `project generate`；随后同步工程并检查 | 更新生成代码与 Keil 配置，日志写入 `output/cubemx/` |
| `sync_project.py` | 恢复ARM Compiler 6.7、单Target、头文件目录及按职责分组的User源码 | 更新`Car.uvprojx`；不改外设源码；静态检查另有当前阶段允许文件清单 |
| `check_project.py` | 核对`.ioc`、生成代码、GPIO/NVIC、只读App和串口所有权、Keil引用与构建设置 | 只读；打印JSON，成功退出0，失败退出1；禁止CAN/PWM启动 |
| `build.ps1` | 先检查，再以 Keil `-r`、`-t Car`、`-o` 完整重编译 | 更新 `Objects/` 与编译日志，成功时打印 HEX 的 SHA256 |
| `test.ps1` | 使用`.venv`执行主机回归，再做固件静态检查 | 原生C测试库写入`output/tests/`；无物理设备访问 |
| `stop_preview.ps1` | 核对本项目预览目录和进程身份后停止后台预览 | 仅停止记录中的本机核心/Web进程，不访问物理设备 |
| `keil_policy.py` | 恢复列表目录、完整DebugOption保护及现有本机选项 | 保留设备包、编译/驱动选择；不下载、不启动调试，`.uvoptx`仍不进Git |

同步工具还会关闭构建前后用户程序、自动进入调试与调试前自动下载。它不调用硬件接口。

同步工具共享`keil_policy.py`，会补齐IDE保存后缺失的调试子块，并处理当前机器已有的`Car.uvoptx`加载/运行选项。IDE另存后如检查失败，可显式执行`.venv/Scripts/python.exe -B tools/keil_policy.py`恢复保护，再编译；构建脚本在编译前后都检查，不把保护项缺失当作关闭。

## 使用方法

在工程根目录运行：

```powershell
.\tools\generate.ps1
.\tools\build.ps1
```

单独执行只读检查：

```powershell
$taskPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
& $taskPython -B .\tools\check_project.py
```

默认 CubeMX 为 `D:\STM\STM32CubeMX.exe`，Keil 为 `D:\Kiles\UV4\UV4.exe`，Python 为上例所示的本机随附运行时。CubeMX 脚本使用同安装目录下的 `jre/bin/java.exe`。本次验证使用 CubeMX 6.18.1-RC2、F4 包 1.28.3、Keil 5.24a 与 ARM Compiler 6.7。

换机器后通过参数指定实际路径：

```powershell
.\tools\generate.ps1 -CubeMX 'D:\STM\STM32CubeMX.exe' -Python $taskPython
.\tools\build.ps1 -Keil 'D:\Kiles\UV4\UV4.exe' -Python $taskPython
```

若本机脚本执行策略阻止直接运行，可从根目录用 `powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\generate.ps1` 运行本次命令；编译脚本同理。

## 产物与结果

- CubeMX 日志：`output/cubemx/generate.log`、`generate-errors.log`。
- Keil 日志：`output/keil/build.log`。
- 固件产物：`firmware/MDK-ARM/Objects/Car.axf`、`Car.hex`。
- 基线重复生成记录：`output/cubemx/regenerate-check.json`。当前只读阶段生成前清单为`output/cubemx/user-before.json`，重复生成后核对8个手写文件SHA256不变，摘要见`docs/stage1_readonly.md`。

以上输出由 Git 忽略；可追踪的结果摘要保存到 [docs/baseline.md](../docs/baseline.md)。生成脚本同时检查退出结果与本次生成文件，编译脚本检查零错误零警告及本次产物时间，不把旧日志当作本次结果。

后续工具继续放本目录，正式运行实现不从工具目录导入。

## CAN只读取证

```powershell
.\.venv\Scripts\python.exe -B tools/inspect_can.py --owner-url http://172.20.10.4:8080 --seconds 10 --output output/can-time/新文件名.json
```

只用既有所有者固定GET，不打开UART、不请求控制权、不发STOP或清故障。窗口0.5～60秒，拒绝覆盖既有证据；原始样本与前后诊断写output。解析实现位于正式`pi/robot/device/can_diag.py`，工具仅组织采样与文件输出。
