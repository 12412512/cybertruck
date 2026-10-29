# CAN只读排查与反馈时间链

2026-10-24，从`4b761ab / v0.5.0-live-observer`继续，节点`v0.6.0-feedback-time`。本轮只通过既有所有者GET缓存排查CAN，补新工程接收/处理/上报时间模块；不烧录、不清故障、不发STOP、不触发运动，不修改两份参考工程或Pi旧服务。

## CAN结论：当前寄存器、历史快照、软件锁存分开

只读入口为`http://172.20.10.4:8080/api/state`。核对旧`CAR/Web/app.py`与`link.py`确认该GET只复制已有缓存；后台既有串口/CAN查询继续运行，新程序不申请控制权、不另开串口、不改变查询调度。

| 层级 | 本轮值与含义 |
| --- | --- |
| 当前CAN诊断 | ESR=0，TEC低8位/REC/LEC为0，邮箱无待发、无隔离，transport_state=2（CAN_SENT，仅表示传输状态） |
| 冻结错误 | 序号168503、MCU板时1686069ms；HAL=`0x1000`为邮箱0发送错误；TSR=`0x1C000009`中RQCP0=1、TERR0=1、TXOK0=0、ALST0=0 |
| 冻结ESR | `0x00800033`：TEC低8位128、REC0、错误警告/被动位置位、BOFF=0、LEC=3（ACK应答错误）；不是纯仲裁丢失，也不能称为当时总线已离线 |
| 当时发送帧 | EXT ID=`0x300`，逻辑地址3、包序0，长度4，字节`FE 98 00 6B`；旧MotorBuild对应停止帧，快照transport_state=CAN_BUSY、tx_pending=1、tx_index0/count1 |
| 软件锁存 | CARCAN13为FAULT，reason2，bus_error3=STOP位1＋CAN位2；pending_stops0、stop_observed_mask15，仍stopped=false/ready=false |

LEC解释以本地ST HAL/CMSIS及[ST官方HAL错误分支](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Src/stm32f4xx_hal_can.c)为依据：LEC位1和位0同时置位映射HAL_CAN_ERROR_ACK。硬件快照只反映最后一次错误，不足以归因全部168503次历史失败。

旧源码路径：`chassis_hardware.c`先记录IRQ寄存器再进入HAL；`can_transport.c`只在非纯仲裁错误冻结寄存器和发送上下文；`motor_protocol.c`构造FE停止帧；`motor_bus.c`将失败锁存为CAN错误。`chassis_test.c`在STOPPING持续3秒未满足恢复条件时转FAULT，FAULT分支继续观察，但自动转IDLE/BusRearm只在STOPPING执行。

因此，后来轮速为0、四轮停止观察位齐全、当前ESR为0，也不会自动清掉FAULT。reason2表示旧停止流程原因，不是电气故障根因。旧Web连接/重新核对会主动STOP，这可能重建STOPPING并最终调用BusRearm，改变软件锁存；本轮未重启它或调用该入口。

已确认的错误类别是历史停止帧发送期间的ACK异常；**物理根因未确定**。CAN ACK由接收节点在总线层提供，不能单凭ID0x300推断3号驱动器损坏。驱动上电顺序、收发器、连接与终端、干扰等仍缺现场证据；不因当前恢复接收就宣布长期可靠性通过。

## 只读采样证据

`tools/inspect_can.py`复用正式`OwnerClient`与`can_diag.py`，固定GET、有界窗口、拒绝覆盖既有证据，不导入串口、不含控制请求。诊断模块可独立用于Web与离线测试，正式实现不从tools导入。

```powershell
.\.venv\Scripts\python.exe -B tools/inspect_can.py --owner-url http://172.20.10.4:8080 --seconds 10 --output output/can-time/diagnostic.json
```

本轮10秒窗口77个样本、0请求失败，板时推进9926ms，CAN RX推进1985，错误增长0、动作增长0；前后ESR0，锁存仍在。随后3秒窗口25/25个板状态时效有效样本，板时推进2957ms、RX推进591，错误/动作增长仍为0。旧协议无启动身份，本工具明确报告`reset_identity_available=false`；前后比较拒绝计数倒退，不能用跨复位快照作长期结论。

原始证据在忽略目录：`output/can-time/before.json`、`diagnostic.json`、`diagnostic-after.json`。10秒文件SHA256：`d7a4974735b1d5e2bda0c1bc8a207413fa689cf9dddb5798281a10d47094fcae`。当前Web增列当前ESR、冻结ACK/停止帧与软件锁存，防止将旧快照误读成新故障；仅展示，不提供清除按钮。

## 时间链的真实来源

| 字段 | 时钟与记录者 | 能证明什么 |
| --- | --- | --- |
| `device_sample_time` | 设备内部采样时刻；现硬件协议未给出 | 保持未知，不能用接收时间冒充 |
| `first_ms/last_ms` | 同一次MCU启动的HAL毫秒；唯一接收者在字节/完整CAN帧到达时记录 | UART组帧接收窗口，或CAN完整帧接收事件；不表示电机/IMU物理采样同步 |
| `handled_ms` | MCU主循环实际解析时刻 | 量化解析延期；不覆盖接收时间 |
| `sequence` | 本机接收/解析序号 | 去重、逆序；不是设备事务号。CAN可因其他地址/类型跳号，不能将跳号直接解释为设备丢帧 |
| `epoch/source_epoch` | 解析连续性/底层接收源代号 | UART错误、环溢出、断流、重连等隔离；不能跨代继续积分 |
| `report_ms` | MCU构造当前反馈负载时刻 | 和接收时间同钟计算龄，支持uint32回绕 |
| `sent_s/response_s` | 主控单调时钟，本次匹配读请求的发出/应答接收时刻 | 有界往返及主控缓存；绝不与MCU tick直接相减 |

旧成功代码在CAN ISR已记录`MotorFrame.tick/rx_sequence`，旧智能车IMU已保留逐字节时间与移窗后的first/end；本轮复用这些思路。旧CAR串口输出WHEEL仅有“上报now减接收tick”的龄，IMU有龄/序号但没有相同的report_ms锚点。旧Web保存`received_at`，JSON却只用它修正fresh布尔值，未导出该时间或修正age数值。

MCU接收时间是软件观察时刻，仍包含中断响应和毫秒量化误差；不是CAN线上的硬件捕获时间，也不是传感器内部采样时间。

因此，**不能从当前旧HTTP返回值恢复精确MCU接收时刻**，也不能用CSTATE板时替代不同查询中的IMU锚点。真实observer继续`feedback_timing_verified=false`、精确测量时间未知；只新增准确的主控HTTP请求/应答/缓存时间。修改或重启旧所有者不在本轮只读范围。

## 新工程已实现的离线时间链

- `Device/stamp.*`：同MCU时间序列、严格小于250ms时效、处理先后、uint32半周期/回绕及同毫秒多帧；换代需显式重建历史。
- `Device/motor.*`：只读解析35 RPM、36完整有符号32位幅值位置、3A状态，保留完整帧接收tick/序号/代次。保留旧5000RPM回复数值保护；无构帧、查询、使能或运动函数，未给地址贴未经验证轮位。
- `Device/imu.*`：复用旧55/11字节校验、已知类型和重叠移窗；保存原始三轴/原始帧/首末字节tick、处理tick、序号及UART源代。固定16槽；坏帧/超时/溢出使连续性失效，旧队列标签不改写。无HAL、中断或UART操作。
- `Comm/feedback.*`：只编码原始值与时间的56字节负载；不发送。当前`comm.c`仍未接它，0x10为未来预留读请求；主机调用正式C状态机验证仍返回READONLY，HELLO能力保持原值。
- `device/feedback.py`：核对外层启动/会话/请求/应答状态，验证负载和MCU时间序列，用完整请求往返作为传输上界；缓存去重不续新鲜度，同序号冲突、跨源换代、复位、逆序及主控倒时清空并锁存。当前不导入Link，不打开设备、不自动重建历史。

Web的`load_snapshot`也已补上核心快照停留时间：即使核心快照未达到500ms总过期，设备龄累加到250ms也立即撤销board、轮/IMU、位姿、零点及当前CAN解读。观察缓存时间同时增加；多次读取各自从原始快照计算，不累计重复加龄。缺失、负值、非有限或布尔设备龄均拒绝在线显示。

原始RPM/计数和IMU signed16是诊断数据，物理单位转换仍在设备标定边界完成；未知量程、安装方向、轮径/驱动比例不写成默认值。后续融合采用MCU接收时间历史，保留串行传输与设备内部采样的未知延迟，不声称硬同步。

## 预留反馈负载V1

外层仍用GC的COBS/CRC与boot/session/request；未来读请求type0x10、无请求负载，应答type0x90，首字节结果0后跟以下56字节。**这只是编码器/解码器约定，当前两端在线调度未实现，不能对旧CARCAN13发此请求。**

| 偏移 | 内容 |
| --- | --- |
| 0～3 | uint8：version1、kind、address、flags0；flags0表明未提供物理采样时刻 |
| 4～31 | 7个uint32小端：first、last、handled、sequence、epoch、source_epoch、report |
| 32～55 | 3个int64小端原始值；kind1速度/2位置/3状态只用第0项，kind4角度/5角速度为三轴signed16原值 |

CAN反馈first=last；IMU保留字节窗口。kind1～3地址1～255，kind4～5地址0。源/解析代号不能为0，序号0可表示计数回绕；无效/过期样本编码失败，不能发初始化0冒充观测。上位机先经正式CRC Decoder，再做请求关联和时间校验；MCU数据龄＋整个往返＋本机缓存龄到250ms即失效。物理测量未标定，时间链通过不等于闭环可用。

## 本轮验证与出口

95项主机回归通过，含新增17项时间链/诊断测试、500组C编码→GC帧CRC解码→Python反馈互通、2项Web快照时间/畸形龄测试，以及现固件对预留type0x10的拒绝验证。覆盖最大编码器幅值、原始IMU包含55、移窗时间、积压、源变更、半帧、坏校验、队列溢出、同毫秒多帧、重复/冲突、跨通道换代、复位/倒时和32位回绕。

扩展Web畸形龄测试时曾因测试文件漏导入json产生1项NameError，补齐后95项全部通过；失败记录保留`output/tests/runtime-import-failure.log`，最终记录为`output/tests/latest.log`。未改变固件功能以迁就测试。

本机776项静态检查通过；Keil ARM6.7完整编译0错误0警告，Code13664/RO448/RW16/ZI8336。MAP列出Stamp/Motor/Imu/Feed全部为未用段，App仍为Port_Init/Port_Run；CAN/PWM输出与USART2接收均未启动。HEX SHA256仍为`fcfc62f05e94bf3968212512506adbf2557c5187435efca2d6c18e8c8ab4a81e`。本轮无IOC或生成外设修改，不需要CubeMX重新生成。

本轮完成“只读故障归类＋新工程时间链离线准备”，**未完成真实时间字段导出或实车闭环**。下一步在明确授权交接设备/部署或板端验收后，让唯一接收者把真实事件时间送入这些接口；先验证启动身份、时间连续性、轮位/极性/尺度与IMU方向，再接连续里程计。锁存故障的电气起因及长期CAN稳定性仍需进一步证据。
