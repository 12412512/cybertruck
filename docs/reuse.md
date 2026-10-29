# 旧模块复用依据与边界

2026-10-24只读核对两份参考工程。优先迁入成熟的协议解析、反馈保护、观测门槛与纯算法，再按新接口调整；不重复发明已验证能力，也不整树复制旧入口和路径。此表不表示代码已经迁入新工程。

以下源路径均相对各参考工程根目录，证据代表记录所述旧安装/旧固件范围。当前硬件状态仍需现场只读核对。

| 内容 | 参考来源 | 已有依据 | 新工程必须补齐 |
| --- | --- | --- | --- |
| 轴方向、人工零点、行程换算和保护 | `智能搬运小车/User/Mech/axis.*`、`User/Config/mech_cfg.h` | `Docs/mechanism_axes_20261022/验证记录.md`有5/10毫米往返；`Docs/mechanism_pose_fit_20261022/验证记录.md`有49/50毫米尺量、换算修正及静止反馈 | 当前基准、完整行程、负载、三槽姿态及过渡避碰；历史145/107毫米未完成全程验证 |
| 车载抓放姿态 | `智能搬运小车/User/Mech/pose.*`、`Docs/grab_once_20261022/继续联调.md` | 49/50毫米原料候选和43毫米车载候选等记录；存车视野存在遮挡 | 本次零点和位置、三槽真实观察、每个成功事件；不能把旧暂停点当当前许可 |
| 连续里程计与IMU历史 | `智能搬运小车/User/Chassis/odom.*`、`yaw_history.*` | 已有连续组合不逐段清零、异步反馈/时效检查接口 | 转成新X前/Y左/逆时针正约定，核对时间对齐、轮型及实测尺度；编码器不能证明无打滑 |
| CAN传输和错误诊断 | `ros小车/CAR/Chassis/`及`CAR/CAN通讯优化说明.md`；旧车Board/电机协议 | 已有仲裁丢失与真实发送错误处理、只读反馈和现场日志 | 择优整合成唯一调度器，保留错误快照；历史错误原因未全部闭环，不能直接标可靠性通过 |
| 四组任务码解析 | `智能搬运小车/Pi/qr_camera/task_codec.py` | 纯解析已区分第二批粗加工位置与按第一层颜色映射的暂存位置 | 新任务账本、帧号/时间/会话去重及完整规则测试；不直接沿用仅按调用次数锁存的时间逻辑 |
| 相机所有权、最新帧缓冲 | `智能搬运小车/Pi/vision_runtime/live_camera.py` | 有设备锁、采集与处理分离、源代次处理 | 移除硬编码旧部署路径；实现两只相机各自唯一采集者；现有SourceManager只流式采选中源，不能冒充双相机同时运行 |
| 六色、圆环、占位及局部观测 | `智能搬运小车/Pi/`各视觉模块及对应`Docs/`记录 | 有真实样本、遮挡/蓝浅蓝/黑料等问题记录，局部新帧稳定门槛和人工小步实验 | 新安装多平面标定、三槽视野、码垛顶面与结果确认；不把局部稳定等同转盘停止或全局位姿 |
| 抓取结果证据门槛 | `智能搬运小车/User/Mech/grab_check.*` | EMPTY/RAW/HELD/STORED顺序、姿态代次、帧龄、多帧确认及单次回调 | 成功账本移至树莓派唯一维护；补完整放置证据及对账，避免两端各自累计 |
| TJC显示 | `智能搬运小车/User/Display/lcd.*` | 已有完整任务码和状态/计数显示接口 | 显示树莓派已确认账本；不再由多个底层分支直接调用累加；实屏文字和最终保留结果另验收 |
| 地图和A* | `ros小车/competition_robot_ws/src/competition_navigation/competition_navigation/planner.py`及`competition_map/` | 有名义地图、封路与无路的纯算法基础 | 实测道路、带朝向状态、偏心载物轮廓、完整扫掠、时间代价、到站朝向；名义地图不是部署地图 |
| 雷达协议与有/无障碍对照 | `ros小车/lidar/cyqm_protocol.py`、`cyqm_state.py`、`静态障碍流程.md` | 有静止扫描完整性、STOP和跨帧候选；右侧近距离场景变化进入检测链的记录 | 调整后安装高度/遮挡、两起点独立配准与每条必要道路覆盖；旧前方遮挡和某路障碍未检出不能省略 |
| Web状态与控制权思路 | `ros小车/CAR/Web/` | 有反馈、令牌、地图预览和断连处理的参考实现 | 按已定FastAPI/独立控制核心重组，网页不占串口；比赛只读、草稿/生效参数分开；不直接复制旧Flask单体服务 |

迁移时在对应模块说明中记录来源、原始版本/哈希、修改点、测试和未验证项。同一能力只保留一个正式所有者；旧测试入口、模拟工位等待和按键固定序列只作测试参考，不成为正式任务实现。

手写代码继续使用简洁函数名、中文注释和有序排版；C模块`.c/.h`同目录，按现有`App/Board/Comm/Device/Control/Config`职责归类，上位机按`device/nav/vision/web`归类。测试可引用正式实现，正式实现不依赖`tools/`或`tests/`。

## 本轮实际复用

- `pi/robot/device/protocol.py`的COBS编码/解码和超长帧丢弃至分隔符逻辑改编自`ros小车/lidar/cyqm_protocol.py`；原文件SHA256为`124b316f7d6d9a4048971b36f2be2cf8003d3df93504ec81989ae22a0e14b839`。替换GC帧头、长度、启动/会话和请求语义；C版本与Python版本通过同一批随机互通样本验证。
- `pi/robot/device/link.py`借鉴`ros小车/CAR/Web/link.py`的唯一串口所有权、接收时刻参与时效判断、重新连接先核对和未知状态不冒充有效反馈。原文件SHA256为`e48545d8480fd030ee48920481a6ebadf7a3ca0b18829df7b9e27a524b915012`。未复制其旧运动接口、Flask耦合和从Tools导入正式协议的结构。
- 两份参考工程均未写入；旧抓放/闭环/视觉尚未移植。当前只读软件通过与物理验收分开记录。

## 任务离线节点复用

- `pi/robot/vision/task_code.py`迁入`智能搬运小车/Pi/qr_camera/task_codec.py`的四组解析及颜色到暂存环映射，原文件SHA256：`3bf5e91414ade5c5e1413b816a3cd7c34d740fbaf2e2d5019efb6454eb3fb766`。保留规则与输出结构，注释/异常改为中文；锁存新增帧号、采集时间、流身份、帧龄和稳定跨度约束，避免按调用次数重复计帧。
- `task_check.py`迁用旧`User/Mech/grab_check.c`的EMPTY/SOURCE/HELD/STORED、同槽视图、动作/会话/姿态代次、250毫秒帧龄及三帧/100毫秒门槛；原文件SHA256：`1d7534a83ada7f1baab5d8ad4e6b821db7e0ac20eebe0d8e1a7c0fcbeed3eca8`。在Python中实现任务级校验，并扩展放置的目标/持料/释放/稳定确认。没有移植或运行其机构动作入口。
- SQLite账本和可重放状态投影为新实现，替代两端各自计数的风险；当前只接模拟证据。原STM32/TJC计数逻辑未接入，实车帧、实时反馈和成功事件适配仍待后续验证。

## 底盘计算基础复用

- `firmware/User/Control/kin.c`改编旧`User/Chassis/kinematics.c`的正逆运动学、有限值检查和四轮同比限幅。原SHA256：`c3ca8e46cf249239909bf43acdc2c1519da06bc19ce6cd7abf6dfb2bf2b8dbaa`。接口改为米/秒/弧度、X前/Y左/逆时针正，移除旧车辆候选默认值，轮序仍为FL/FR/RL/RR；实际驱动极性不混入理论轮速。
- `Control/yaw.c`沿用旧`User/Chassis/yaw_history.c`的64槽环形时间历史、uint32回绕和包围区间插值方法。原SHA256：`bb43b83675091d331ec991de61277a280dc66c67bd9fc5f278eba8812b7dea37`。输入改为CCW正弧度，新增±pi展开、时效/角速度检查与异常锁存，不外推、不向IMU写零。
- 两模块只做数值运算与离线验证，没有移植旧路测标志、速度、轮几何或自动执行入口；后续里程计及设备适配另行验证。

## 真实观察与高级几何复用

- `Control/curve.c/.h`改编旧`User/Chassis/motion/scurve.c`的`SPlan/SSample`静止七段曲线部分。原SHA256：`7bae3d24d1c7a7ac139969e4e5f6c80a4b76e65668d6527af43a36a84cb116d8`。换成Module_动作、SI数值域、中文注释和展开排版；没有迁入`PathTimePlan`分段变速部分。
- `Control/path.c/.h`改编旧`User/Chassis/motion/path.c`及其类型布局。原SHA256：`1a2f8c4acdc2f84a1e9237299042b470d8bdb78b6c87512e6de61ff4a935fc3d`。保留余弦升降曲率、Gauss构表、七次Hermite、边界投影与冻结校验；转为米、Y左/CCW正，turn正值代表左。仅纯几何，不复用旧测试默认半径/速度和执行入口。
- `Control/line.c/.h`迁用旧`motion/motion.c`的XY距离归一化和共用标量时间轴投影；原SHA256：`98cc7782c0843fb377c0fa08e156d93f49ac0196b9789726591ba3c7ef4e15cc`。移除全局运动状态和驱动耦合，调用者持有独立冻结Plan，不含真实完成状态。
- `device/observer.py`直接读取已核对旧服务`/api/state`，复用其串口反馈结果，未复制或打开另一个旧串口管理器。旧控制权、租约、失焦释放、地图和雷达接口逐项核对后按`web_merge.md`安排合并；当前只接只读诊断。
- 原来源文件均未写入，移植以新测试和真实观察分别验收，详细证据与未验证项见`motion_readonly.md`。

## 接收时间链复用

- `Device/motor.*`仅迁入旧CAR `Chassis/motor_protocol.c`的35/36/3A接收分支；原SHA256为`d1c1a48baeeb3f87b9bcda85663d7a38cc5af1e93e02133965415b9fb92b1bd5`。保留5000RPM回复保护和完整有符号编码器幅值；新增主循环处理时间、接收序号/源代校验，不迁入任何构帧或控制函数。
- `Device/imu.*`改编旧智能车`User/Device/imu/imu.c`的11字节格式、校验、重叠移窗同步搬运tick、原始样本、16槽及连续性代号；原SHA256为`1fa9c777a8073684acff1acb8977a3de09757bb044341f9fc44c3d148fd6dfa7`。抽离UART/HAL/全局应用耦合，加入handled时刻与溢出失效，保留原始三轴，不复制未标定量程假设。
- `can_diag.py`依据旧CAR错误冻结上下文及本地ST寄存器位定义做只读解释。旧`Chassis/can_transport.c`本次SHA256为`a91d49de77b7b5c3bb29ac688bf0cff2da7c58867d4b4ab8c75dff7cd2e67431`，没有迁入其发包/重试/清HAL错误入口。
- `stamp.*`、`feedback.*`及Python反馈缓存为新接口适配，复用既有GC帧/CRC而不另起传输协议；时间负载先离线验证，不改变当前Comm能力。详细新旧时间语义与物理出口见`feedback_time.md`。

## 连续里程计复用

- `Control/odom.*`迁用旧`User/Chassis/odom.c`的连续轮增量、IMU历史、时效/计数跳变门槛、保留原点和整批提交；来源SHA256为`48607c8c51ac53c313ed225b3e915a23460c151f166575e5fd4c344f9c69d670`。
- 为避免各轮积分区间不一致，改为四轮历史在共同时间边界插值，再按全部轮/IMU节点分段积分；继续复用正式Kin和Yaw，不复制另一套公式/角度展开。移除HAL/全局轮总线/路测档位，只接显式配置和带代次样本。
- `device/odometry.py`只做正式C的模型绑定；`odom_sim.py`复用现有C反馈编码、Python GC/CRC和FeedbackCache，独立解析曲线作真值。主控仅sim显式加载，正式代码不导入tests/tools实现；详见`odometry.md`。

## 轨迹跟踪与稳定到位复用

- 旧`User/Chassis/drive.c`的参考在位置/航向各自时间回查、连续odom、零速批次TXOK屏障、发送后新反馈和静止窗口迁入新`Control/ref.*`、`track.*`。原SHA256为`a954c9c7269650013f41a76976e2cd778ab679589c5a37a431c2921f9004b720`。
- 旧`heading.c`（SHA256 `f6939aca741a6fbdc88934f9fa35ec0217db0415de952035f933dd4a8f4fff13`）及`position.c`（SHA256 `fe0f436e6808eacc3a5d87591b64f2a1877b7fc5d8a898bbc1338642bfc2c50c`）的有界P、滞回和误差方向用于新接口。新增角速度误差阻尼与整体轮速平滑，未复制其旧全局状态和整数配对算法；新实现仍须实车验收。
- 沿用正式Kin/Curve/Line/Path/Odom/反馈编码，不复制测试公式作为生产控制。独立Plant是模拟对象，明确分离参考、模型真值和观察输出；本轮零速确认仅为模型回调。参考源只读，App/CAN/PWM保持未接入；详见`tracking.md`。

## 坐标与路径规划复用

连续导航节点进一步迁入旧 `User/Chassis/motion/scurve.c` 的 `PathTimeMake/PathTimeSample` 到现有 `curve.*`，保留有界分段变速和边界连续，增加SI接口、末端接近档及新参考约束。源SHA256 `7bae3d24d1c7a7ac139969e4e5f6c80a4b76e65668d6527af43a36a84cb116d8`，具体复用范围和A/B验收见[连续导航](flow.md)。旧参考树未修改。

- 新`nav/geometry.py`沿用旧`competition_map/geometry.py`的凸包与凸多边形SAT，原SHA256为`c9c38293019fc83b31fb70f615a1303d590fa028ef0492237972b1f23efbe1ab`。不复用“只支持固定朝向平移”的限制，扩展偏心四边和来自正式C变化率界的区间包络。
- 新`nav/planner.py`复用旧`competition_navigation/planner.py`的堆A*、道路状态过滤、came重建和拒绝原因结构，原SHA256为`ffc5ac3a1e40df90ff2f6ef42840b6706a4d3ee62cbecc0a594dcc4e9f6e7d9b`。状态扩展为节点加朝向、代价改为参考时间，增加共线合并和经扫掠/轮速检查的局部弧段。用户已明确蓝区可行，旧途中蓝区禁入规则不迁入。
- 新名义地图沿用旧site_measurements的黄区、工作台和场地总框；候选停靠点为满足条件模型净空而调整，未伪装成实测。路径、弧线、参考时间和闭环继续调用已迁入的正式C；不复制另一套Python运动曲线。
- `route_sim.py`复用已有ClosedLoop/Plant/反馈编解码，逐段共享odom并核对独立真值。C Ref只新增主机查询/变化率界，未接App或硬件；详细数学边界、场景假设和验收见`navigation.md`。
