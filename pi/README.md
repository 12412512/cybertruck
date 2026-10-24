# 树莓派程序

当前实现GC只读协议核心及独立FastAPI网页，默认不打开设备；增加对旧CARCAN13串口所有者的只读观察模式。本机Python3.12虚拟环境已安装；新程序尚未部署树莓派，新GC板端未验收。

核心入口`robot/main.py`，网页入口`robot/web/app.py`。`robot/task.py`为尺寸无关的任务状态机，当前仅接模拟域，不调用执行器。

| 目录 | 职责 |
| --- | --- |
| `robot/device/` | 唯一 STM32 串口管理、相机及雷达设备管理 |
| `robot/nav/` | 地图、A*、定位估计与路径执行 |
| `robot/vision/` | 扫码、物料、圆环和结果确认 |
| `robot/web/` | 网页接口与页面资源 |
| `robot/web/static/` | 只读中文总览HTML、CSS和JavaScript |
| `deploy/` | 服务启动和部署配置 |

树莓派维护唯一任务与计数账本；网页不直接打开串口。包导入不依赖旧工程路径。

当前`device/protocol.py`负责纯协议，`device/link.py`负责会话与时效，`device/sim.py`为不导入串口的演示设备。`runtime.py`管理独占锁、原子快照和过期判定。正式任务、计数和物理状态尚未接入。

`vision/task_code.py`迁入旧任务码解析并增强新帧锁存；`task_check.py`管理分阶段观测证据；`ledger.py`负责SQLite事务和事件幂等；`task_sim.py`提供独立物料模型与离线入口。模拟计数单独标识，真实计数仍为空。

## 本机运行

使用Python3.11及以上；本机环境为仓库根目录`.venv/`。以下命令从工程根目录执行：

```powershell
.\.venv\Scripts\python.exe -m pip install -r pi\requirements.txt
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -m robot.main
```

另开PowerShell启动网页：

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -m robot.web.app
```

访问`http://127.0.0.1:8000`。核心默认offline，两进程默认共用`output/runtime/`；用`--runtime`修改时两端必须一致。Ctrl+C停止对应进程；核心停止后网页撤销在线。

看演示时先停止离线核心，再改用`python -m robot.main --mode sim`（使用上述虚拟环境Python）。页面显著显示模拟数据。本次预览共用独立目录`output/preview/`。Web默认仅本机，需要局域网查看时显式设置`--host`。

本次后台预览的进程号记录在`output/preview/processes.json`，可从根目录执行`tools/stop_preview.ps1`停止；脚本先核对进程命令行和本项目预览目录，防止PID复用误停其他程序。

`--mode serial --port <明确设备路径>`只兼容GC-RO新固件，当前不能对CARCAN13运行。实车串口当前由旧`car_web`独占，本工程未烧录、未部署Pi服务，不争抢设备；`deploy/`未安装服务或开机自启。

协议、时效和保护边界见[协议说明](../docs/protocol.md)。

## 观察已有实车服务

本轮核对Pi8080的真实所有者与CARCAN13固件后，通过其状态GET合并到新Web，不触发控制权、运动或旧控制保活。独立运行目录避免混入模拟账本：

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -B -m robot.main --mode observer --owner-url http://172.20.10.4:8080 --runtime output/observer
```

另开终端：

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -B -m robot.web.app --port 8001 --runtime output/observer
```

访问`http://127.0.0.1:8001/`。本轮已启动本机预览，PID记录于`output/observer/processes.json`；使用`tools/stop_preview.ps1 -Name observer`可核对身份后只停止本机两个进程。手动终端运行用Ctrl+C，不依赖PID文件。旧Pi8080未替换、未停止。

新页展示原始四轮、IMU、CAN快照、串口错误与原服务控制权状态；设备过期立即撤销有效值。真实状态与任务模拟计数隔离；本模式不推进模拟账本或实际任务。合并安排见[Web合并](../docs/web_merge.md)，实测及故障记录见[本轮验收](../docs/motion_readonly.md)。

`device/can_diag.py`将当前寄存器、冻结帧和软件锁存分开解释，`tools/inspect_can.py`可在明确的GET窗口留证。`device/feedback.py`为未来新时间负载的严格解码/去重缓存，未接入Link、observer或真实串口；当前真实观察仅补HTTP请求/应答/缓存时间，MCU接收与物理采样时刻仍未知。规范、故障结论和本轮验收见[CAN及反馈时间](../docs/feedback_time.md)。

## 任务离线联调

只运行任务模型并生成报告（不需要通信进程）：

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -m robot.task_sim --code '136+321+631+312' --start-zone 1
```

与模拟通信和只读Web一起运行：先停止原核心，在核心命令中增加：

```powershell
.\.venv\Scripts\python.exe -m robot.main --mode sim --task-demo '136+321+631+312' --start-zone 1
```

Web启动方法不变。完成后保留完整任务码、模拟12/12计数及三槽/码垛结果；核心重启遇到未结束任务时先进入等待对账，不能靠重复启动命令续跑。`--task-demo`不允许offline或serial模式。

当前账本为运行目录中的`task.sqlite3`及SQLite附属文件，不进Git。没有`--task-demo`时，核心仅显示已有账本并将未结束轮次置为待对账；独立`task_sim`不填`--code`时只查看，不推进。

显式终止待对账的模拟任务：先停止核心，再执行`python -m robot.task_sim --runtime <同一运行目录> --abort-run <run_id>`，随后可以用`--code`创建新轮次；这是终止旧轮次，不是确认未知动作已成功。详细顺序与限制见[任务离线联调](../docs/task_sim.md)。

## 连续里程计联合模拟

```powershell
.\.venv\Scripts\python.exe -B tools/build_odom.py
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -B -m robot.main --mode sim --task-demo '136+321+631+312' --odom-library output/native/odometry.dll --runtime output/preview
```

网页沿用`--runtime output/preview --port 8000`。本轮已重启本机预览为该组合，SQLite保留历史并新增模拟轮次；停止仍用`tools/stop_preview.ps1`。Windows库被运行中的核心持有时，先停本机模拟再重建，或在构建工具用`--output`指定新文件。

模型通过正式C负载编码、GC拆包/CRC、请求/时间缓存和正式C连续观察器，显示模拟odom、解析对照和近期轨迹。物料任务与里程模型各自标记simulation，NAV仍未接真实场地；顶层真实pose/grabs/places保持未知。`--odom-library`在offline/observer/serial模式均拒绝，主程序不自动编译或加载模型。

只生成离线报告：`python -B -m robot.odom_sim --library output/native/odometry.dll`，默认输出`output/odometry/report.json`。来源、共同时间对齐及未验证项见[连续里程计](../docs/odometry.md)。

## 轨迹跟踪联合闭环

当前本机8000已改为`--mode sim --track-library output/native/tracking.dll --runtime output/preview`，保留既有任务账本，未新建模拟任务。模型包含正式里程计，不能同时传`--odom-library`；真实模式拒绝加载。构建命令为`python -B tools/build_odom.py --output output/native/tracking.dll`；单独报告为`python -B -m robot.track_sim --library output/native/tracking.dll`。两者均在根目录使用虚拟环境，运行时设置`PYTHONPATH=pi`。

页面显示参考/实走双轨迹、误差、命令与模型轮速、参考结束和到位的不同时间、零速批次确认及新反馈稳定窗口。8001及Pi旧服务未修改。运行步骤、复用与未验证项见[跟踪与到位](../docs/tracking.md)。

## 统一坐标与导航巡回

当前本机8000使用`--mode sim --route-library output/native/corners.dll --route-speed .30 --runtime output/preview`，默认新转姿窗口、1.2rad/s弯道/窗口角速及区域返航。启停区1车头+X、启停区2车头−X、扫码+X。上面模型接入段落为历史记录，以本节为现行入口。构建：`python -B tools/build_odom.py --output output/native/corners.dll`；独立回放：`python -B -m robot.route_sim --library output/native/corners.dll --cruise-m-s .30 --start start_1`。均用虚拟环境并设置PYTHONPATH=pi，详情见[弯道与转姿](../docs/corners.md)。

新页显示名义地图、轨迹、轮速与角速上限、区域停车及未验证项。起点1模型84.29s、起点2模型89.41s，尚未满足完整60秒，工位内移位和视觉修正仍缺。`--legacy-corners`保留旧窗口/曲率策略，`--turn-rate`仅模拟角速；本轮同速工具为`tools/tune_corners.py`。模型与独立odom/track互斥，真实模式拒绝加载或使用`--route-speed`；不推进扫码/搬运账本，Pi8080和本机8001未改动。
