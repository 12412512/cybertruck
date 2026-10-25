# 开发工具

`replay_lidar.py --case detour --start start_1 --execute-model --output <新目录>`生成厂家格式合成报文、检查扫描、自动规划并运行内存协议模型；没有真实设备选项。依赖已有`output/workbench/native.dll`，可由`build_odom.py`构建；案例及验收见`docs/lidar_offline.md`。

自主规划离线接入见`docs/planned_link.md`：重建主机库后，原8000的“执行链联调（合成PC0）”复用正式Comm/Start/Exec与独立Plant，不打开设备。动态包仅在主机`EXEC_PREVIEW`编译分支有效，不能加载到真实串口。

换板集中验收准备工具`accept_board.py`默认只打印计划；`--analyze`只读旧记录，只有现场另获授权后的显式`--collect`才可能打开串口。方法及未覆盖项见`docs/board_acceptance.md`，本轮没有执行采集。

本目录提供 CubeMX 生成、Keil同步、检查、重编译和诊断工具；没有下载或调试启动命令。`bench_wheel.py`仅用于明确确认现场条件后的单次架空实测，必须显式`--execute`；不会随服务、编译或测试自动运行。当前用户已要求跳过逐轮重复测试。

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
| `bench_wheel.py` | 经旧HTTP所有者请求一次6RPM/600ms架空单轮诊断 | 原始证据写独占新文件；无自动重试、无串口接管；异常一次STOP后只读留证，不自动清错续跑 |

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

## 正式C主机模型库

`python -B tools/build_odom.py`将正式Odom/Kin/Yaw/Stamp/反馈编码器构建到`output/native/odometry.dll`（Linux为so）；Zig依赖沿用tests/requirements.txt，不链接HAL、驱动、App或测试实现。`--output`可指定另一个产物文件；Windows旧模型正在运行时先停本机模拟或换新文件。

模型由`robot.main --mode sim --odom-library <库路径>`显式加载，或用`robot.odom_sim`独立回放；不开设备、不自动构建、不在真实模式加载。详细运行与验收见`docs/odometry.md`。

跟踪节点的主机库增加正式Ref/Track及其Curve/Line/Path依赖，默认旧库入口保持兼容；可用`--output output/native/tracking.dll`单独构建。`robot.main --mode sim --track-library <路径>`显式选择闭环，真实模式拒绝；Windows库正被核心持有时先停对应本机模拟。详见`docs/tracking.md`。

导航主机模型沿用`build_odom.py`，可显式输出`output/native/navigation.dll`；`robot.main --mode sim --route-library <路径>`加载统一坐标/规划与多段闭环，不能与独立odom/track库参数同用。`robot.route_sim --library <路径> --blocked <道路编号>`可独立运行封路场景，默认报告output/navigation/report.json；所有执行权限仍关闭。

连续导航节点改用`output/native/flow.dll`，C与ctypes结构需一起重建。`python -B tools/compare_nav.py --library output/native/flow.dll --output output/flow/release`直接调用正式模型，对照起点1停车/连续、起点2连续和五站巡回，保存源码/配置/库哈希及完整失败/成功结果；不接设备。`robot.route_sim`默认two_batch，支持`--scope tour`、`--start start_2`、`--stopped`。范围、未建模项和预算见`docs/flow.md`。

提速节点须重建到`output/native/speed.dll`以取得正式C转姿区间上界。`tools/tune_nav.py --library output/native/speed.dll --output output/speed/region`对照六场景；`--case`选择单个或多个，`--collect`核对版本并汇总，支持不同case并行留证。当前区域返航不要求原中心或朝向，历史对照可传`--fixed-return-position --fixed-return-heading`，详见`docs/speed.md`。

弯道版改建`output/native/corners.dll`，新增正式几何导数限速查询。`tools/tune_corners.py --library output/native/corners.dll --output output/corners/release`固定0.30m/s对照旧策略、窗口优化、转姿角速及两起点；case可并行，collect核对哈希/数据域/同速口径。独立回放可用`--legacy-corners`与`--turn-rate`，仅模拟，不改实车配置。上电只读留证继续复用`inspect_can.py`，见`docs/field_test.md`。

## 板端识别、烧录与只读验收

可选烧录依赖在`requirements-flash.txt`，不属于树莓派运行依赖。`flash_base.py`默认只读识别，显式`--program`还必须绑定探针、芯片UID及本次HEX SHA256；写前完整备份，写后全部有效字节读回，再复位运行。不得把旧DFP的CCM当作可执行下载RAM。`make_flash_pack.py`生成独立修正包，不修改本机源设备包。

`accept_base.py`须在旧所有者停用、当前串口明确后运行；仅发送白名单查询与保活，无配置、使能、运动或清故障。输出文件拒绝覆盖，任何判据失败均非零退出。当前串口打开可能使板复位，正式open_transport静默500ms后才允许握手，仍重新绑定boot/session。

本次已执行命令、包来源、恢复边界、Pi部署及证据见[板端记录](../docs/board_readonly.md)。输出路径仅为本次证据，重做要用新的目录及新编译摘要，不能直接重复复制烧录命令。

`freeze_start1.py`复用当前工作台参数，生成区1三工位外缘间隙50mm的两批路线，完成离线闭环后只写`output/start-switch/start1_candidate.json`，不覆盖用户草稿、不连接设备。受限执行候选在`config/start1_route.json`，同步工具按摘要生成`Config/start1_route.h`。PC1统一入口及离线/实物边界见[选区记录](../docs/zone_switch.md)。
