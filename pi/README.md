# 树莓派程序

当前实现GC只读协议的主控核心及独立FastAPI网页，默认不打开设备。本机Python3.12虚拟环境已安装；树莓派安装、实际串口和新固件尚未现场验收。

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

`--mode serial --port <明确设备路径>`为后续实机入口，当前不运行。它只兼容GC-RO新固件，不是旧CARCAN固件的网页替代品。本次未烧录、未连接物理UART、未部署树莓派；`deploy/`未安装服务或配置开机自启。

协议、时效和保护边界见[协议说明](../docs/protocol.md)。

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
