# B端第一天：视觉与任务离线适配交付

2026-10-30；共同基线 `6f8694f7ff95de99ee3460530d16ed8657092a3d`，分支 `codex/vision-mission`。
本节点是离线合同演练，接口待A审查；不表示真实任务调度已部署。最后烧录0.3.50、+Y实物未通过和自动2号位对中未验收的结论保持。

## 本节点内容

| 文件 | 用途 |
| --- | --- |
| `pi/robot/task_replay.py` | `OfflineMission`：请求冻结、执行收据校验、观察姿态与证据绑定，复用原Task与Ledger |
| `pi/robot/vision/task_evidence.py` | `CandidateGate`：适配旧物料/环号/占用观测；合成分阶段断言的独立边界 |
| `pi/robot/task.py` | 新增UNKNOWN事件，结果未知与已知FAILED分开，二者均进入待对账并保留物料未知状态 |
| `tools/replay_mission.py` | 显式生成合成执行收据和视觉断言，两个起点的独立账本与报告 |
| `tests/test_task_replay.py`、`tests/test_task_evidence.py` | 新增25项测试，包含实际旧RingObserver的合成候选稳定链 |

未修改A负责的firmware、device、arrival、centering、centering_fit、ring_feed、Web主入口、相机配置、协议及AGENTS。未访问Pi、串口或相机，未执行连接/部署/烧录脚本。测试中的本地FastAPI客户端、模拟核心不代表设备连接。

## 接口审查与缺口

| 现有接口 | 已有能力 | 缺口及本次处理 |
| --- | --- | --- |
| `Task.build_plan/reduce` | 四组任务码、两批12抓12放、分阶段确认、失败与恢复待对账 | 原合同只接受simulation；继续保留，新增UNKNOWN，不开启physical域 |
| `task_check.phases/check_frames` | QR与取放阶段、新帧/100ms窗口/250ms时效、颜色、身份、空爪和放稳验证 | 原合同依赖sim-v1与语义断言；本次在外层绑定请求SHA、姿态、相机、平面和独立环号，原始夹具帧一起入事件 |
| `Ledger` | SQLite原子事务、幂等、投影与事件重放审计 | 原样复用；请求/收据/原始断言保存在事件payload；所有计数仍属simulation，未建立实物成功账本 |
| `Detector.process` | 图像到物料/环/QR像素候选，标注metric_valid=false | 无物体不等于空槽，像素中心不等于夹持中心；空白图测试验证不会产生成功事件 |
| `FrameEngine`、`MaterialObserver`、`RingObserver` | 任务码和batch/order绑定、目标识别、时序稳定、旧帧拒绝 | `CandidateGate`外层核对上下文、任务、目标、帧序及时间，候选ready始终不能确认HELD/STORED/RELEASED |
| `OccupancyObserver` | 同任务会话的空环参考、实时环几何、颜色/占用稳定 | 不能证明夹爪分离、抓起或两层码垛成功；缺参考保持未知，不由同色占用推断放置成功 |
| 唯一真实设备核心 | 由A统一持有串口与执行权 | 本次没有传输/执行实现；A确认接口后才能接现有核心，不另建串口所有者 |

## 提议给A的合同（尚未接入线上）

请求字段：`schema_version=1`、`domain=offline_fixture`、`transport=none`、`run_id`、`action_id`、任务`session_id`、`core={owner,boot_id,session_id}`、`parameters={revision,sha256}`、完整语义`goal`、`request_sha256`。摘要使用现有Ledger的canonical JSON计算，不依赖字段顺序；比较区分bool/int和int/float。

`parameters.sha256`应由A绑定冻结执行参数/保存版本的实际内容，而非只使用显示名称。当前工具中的摘要全部为fixture，不引用任何实车保存表或零点。正式接入还应由A在核心内部验证固件能力、配置删除/变更、执行权代次及实际时效，B的离线合同不授予这些许可。

收据：`domain=offline_fixture`、`source_kind=synthetic_execution`、原样请求、`status`、`observed_s`，COMPLETE另带`result`。状态分别为ACCEPTED/RUNNING/COMPLETE/FAULT/UNKNOWN；COMPLETE只进入EXECUTED，不能直接增加成功数。FAULT映射FAILED，UNKNOWN明确保留UNKNOWN，两者停止推进并等待对账。重复同内容收据不产生新事件；同事件不同内容冲突，跨任务/会话/参数/目标拒绝。应答丢失时查询`pending_request()`，不重新创建动作；本模块不发查询网络请求。

观察合同：`camera_id`、`camera_config_sha256`、`image_size`、`pose_id`、`pose_revision`、`pose_sha256`、`target_plane`、`view`。帧另带请求摘要、观察代次、阶段、采集时间、单调帧序、证据编号和目标环号。图像配置、观察姿态和目标平面必须同时匹配。A还需将上下文与实际采集服务及保存姿态绑定，不能只信调用者自行填写的标签。

本节点的证据边界：

- 旧视觉结果进入`CandidateGate`时只产生候选，`material_event=UNKNOWN`，`confirmation_allowed=false`。ready不等于抓取/存车/释放成功。
- 任务演练只接受显式`source_kind=synthetic_assertion`的夹具帧。它们经原`task_check`验证后转换为simulation载荷，原始来源同时保留，不能把historical_replay/camera_live改标签后当新实物证据。
- `snapshot.domain=offline_rehearsal`，内部任务仍为simulation，`physical_counts=null`、`physical_acceptance=false`、`motion_authority=false`。
- 最终确认同时要求新鲜的完成结果和新鲜的最终视觉证据。未释放、未放稳、旧完成结果配新帧均不能计数。重新打开未完任务仍需调用原`Task.recover()`进入对账；工具遇到已有账本直接拒绝，不自动续跑。

## 环境与样本核对

Python 3.11.9，Windows，`.venv`使用system-site-packages以复用本机OpenCV/NumPy；OpenCV 5.0.0、NumPy 2.4.6。已按`pi/requirements.txt`安装固定依赖：FastAPI 0.162.0、Starlette 1.9.0、Pydantic 2.14.1、pyserial 3.5；HTTP测试客户端httpx 0.28.1。FastAPI测试有弃用提示但用例可运行。

初始环境缺FastAPI导致旧视觉测试导入失败，补齐后恢复。Windows默认GBK导致pip直接读取含中文注释的requirements失败，使用`python -X utf8 -m pip install -r pi/requirements.txt`后成功；未改共享依赖文件。

仓库跟踪的图像仅有三个数字模板及两张CAD/地图说明图。数字模板参与了现有分类器等价回归，不当作本车现场帧。`output/vision/samples.json`、`output/vision/samples/`和交接ZIP中的data/manifest未随仓库交付；旧迁移清单中原始目录为A端Windows路径，本机没有原目录，无法验证原现场图像或逐帧结果。未伪造manifest或以合成帧填补现场回放测试。

## 集中采样清单（供A安排一次采集）

每组保存原帧/连续帧、SHA256、相机身份和分辨率、配置摘要、实际观察姿态ID/修订/SHA、目标平面、目标编号/颜色、采集时间/序列及人工实物标签。失败帧同样保存。历史数据标历史，不重放到运行时保存表。先补原交接包现有样本和manifest，再安排以下缺口；本清单不要求用户立即操作。

| 观察姿态/目标 | 成功样本 | 失败或未知样本 | 用途 |
| --- | --- | --- | --- |
| 扫码观察位/四组任务码 | 同码连续新帧，覆盖两起点与两组任务码 | 模糊/多码/旧帧/遮挡 | 码锁存与上下文验证 |
| 原料观察位/当前目标物料 | 目标六色、完整可见并停稳 | 错色/黑色印刷干扰/ROI边缘/遮挡 | SOURCE候选；换正式台需重验 |
| 夹持观察位/当前物料 | 抓起、离开来源且确实持住的连续帧和人工标签 | 抓空/滑落/与背景重叠/夹爪遮挡 | HELD真实证据方案，现无可直接接受的观测器 |
| 车载三槽各自观察位/槽1～3 | 同视图先空槽、再存入正确颜色、夹爪离开 | 错槽/遮挡/假空/存车后掉落 | EMPTY与STORED对应；第三槽265度仅作待验参数 |
| 粗加工观察位/环1～3 | 空环编号及实时几何、放入正确颜色后占用 | 错环/错色/无当前空参考/多物料跨环 | 环号和TARGET候选；2号对中前置仍归A验收 |
| 粗加工取出观察位/指定环物料 | 来源正确、持料、取出后存回原槽完整连续过程 | 抓空/取错/未脱离/存回错槽 | 单件取回闭环的真实证据 |
| 暂存观察位/环1～3第一层 | 释放后夹爪分离、位置正确且持续稳定 | 未松开/倾倒/压环/错色错环 | RELEASED/PLACED，命令完成不能替代 |
| 暂存第二层观察位/同色底层与顶面 | 已确认底层、同色顶件放稳的多视图连续帧 | 底层遮挡/异色/偏叠/顶件滑落 | 现有同色占用不能证明叠放，待单件链路验收后采 |

## 本次验证

| 范围 | 结果 |
| --- | --- |
| `test_task*.py` | 52项通过：原任务27项、新任务合同18项、新视觉证据7项 |
| `test_ring_digit_performance.py` | 3项通过，三个现有模板的旋转/噪声/边缘/拒识与旧评分逐项一致 |
| `test_vision_workspace.py` | 33项通过，合成图像与模拟HTTP设备，未访问实物 |
| 以上本节点相关回归合计 | 88项通过 |
| `test_vision_migration.py` | 9项中8通过、1项ERROR：缺现场samples.json；原始失败保留，不报全绿 |
| `tools/check_project.py` | 1,146项静态检查通过；未运行Keil或下载 |
| 独立命令回放 | 两起点、两码覆盖六色，均34步/模拟12抓12放；事件重放投影一致 |

第一件物料的独立预期轨迹由用例断言：原料→tray:1→rough:3→tray:1→stage:3；另一任务码为rough:1/stage:1。负例覆盖重复/冲突结果、旧会话、参数漂移、目标合同篡改、bool冒充编号、旧/未来/重复帧、姿态/相机/平面错配、错误环号/颜色、夹爪未分离、未放稳、结果UNKNOWN/FAULT、重启不自动重发。新增适配器导入测试禁止加载设备核心、串口、相机、socket和cv2，确保其本身只有纯合同逻辑。

原始日志与回放账本归忽略目录`output/b-day1/`；`zone1/report.json`、`zone2/report.json`均显式否认实物验收。未运行全仓所有原生C/Keil用例；上述范围之外不作通过声明。

## 复现

从仓库根目录执行，先准备Python环境和requirements；OpenCV、NumPy及httpx为视觉/HTTP测试依赖。本机使用上述版本，跨OpenCV版本的识别效果仍需另外核对。

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p 'test_task*.py' -v
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_ring_digit_performance.py -v
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_vision_workspace.py -v
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_vision_migration.py -v
.\.venv\Scripts\python.exe -B tools/check_project.py
.\.venv\Scripts\python.exe -B tools/replay_mission.py --runtime output/b-review-zone1 --start-zone 1
.\.venv\Scripts\python.exe -B tools/replay_mission.py --runtime output/b-review-zone2 --start-zone 2 --code '245+123+542+321'
```

历史视觉用例在现场样本未补齐前仍有一项ERROR。回放输出目录必须选新的；不删除既有账本来自动重试。

## 交付与下一步

本节点集中做一次本地提交和说明标签；提交号以`git log -1`及本次聊天交付为准。尚未推送、创建Fork或PR；后续按用户安排通过Fork提交，A先审接口与差异，再决定集成。不得直接推上游main。

A需确认共享核心请求/结果合同、提供旧样本包与manifest，并集中补上述证据。实物横移、四向映射、2号环自动对中、抓放视觉证据、正式任务所有权、完整跑图与180秒实测均不在本次通过范围内。
