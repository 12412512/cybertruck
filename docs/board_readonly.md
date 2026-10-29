# 新固件烧录与只读验收

2026-10-24，基于93277e2/v0.13.1-imu-direction，节点标签v0.14.0-board-readonly。用户已确认供电、静止、机构收拢、可立即断电及ST-Link连接，并授权本轮烧录和只读检查；不执行短段运动。

## 结果与边界

GC-BASE-0.3.3已经写入本车STM32，103,568字节HEX有效内容逐字节读回一致。30秒新协议验收通过：294次状态快照、718条带时间反馈；未配置、未使能、动作ID为0，启动停车确认。CAN接收序号110→6104，新增5,994帧，CAN错误为0。两路UART的错误、溢出和重挂接失败均为0，协议CRC错误为0。

| 驱动地址 | RPM样本范围 | 编码器原始计数范围 | 速度/位置/状态样本数 | 驱动状态值 |
| --- | --- | --- | --- | --- |
| 1 | 0～0 | 15207～15207 | 15/15/15 | 3 |
| 2 | 0～0 | -8485～-8479 | 15/15/15 | 3 |
| 3 | 0～0 | 13569～13571 | 15/15/14 | 3 |
| 4 | 0～0 | -8473～-8473 | 14/14/14 | 3 |

编码器存在少量计数量化变化，不写成绝对不变。停车是本次新反馈与板端稳定窗口确认，不能代替后续距离精度验收。驱动状态3不解释为驱动器已断使能；本轮核心未发ARM。

IMU角度和角速度各271条，带ISR接收、首次处理、上报时间及源代。按现有量程换算，静止原始Yaw约-1.395～-1.384°，Z角速度0～0.122°/s。用户已确认航向顺时针增加；配置v2 yaw_sign=-1。此次没有实体转向，Z角速度方向、尺度和安装一致性仍在10°短转中验证。未配置时odom_valid=0，页面中的0位置不代表已测得地图位置。

## 修复依据

1. 未配置时原适配器结束启动停车便停止轮反馈查询，无法持续只读验收。现保持唯一CAN调度器必要查询，ACTIVE/READY仍沿用执行器调度，未增加第二个CAN管理者。
2. 复用旧CAR/Core/Src/imu_uart.c的错误交接：RxCplt发现HAL错误码时不把坏字节入队、不立即Receive_IT，交由ErrorCallback记录和主循环恢复。否则Receive_IT会清除原错误码。补充0x24只读UART诊断，每路9个uint32，不提供清计数接口。
3. 当前USB接线下，重新打开串口观测到MCU启动身份变化。立即发送HELLO时曾在第0毫秒产生一次UART1帧错误（HAL码4），随后锁存0x102。打开后增加500ms静默窗口再握手，5秒诊断及完整30秒验收均零UART错误。没有放宽运行期故障锁存、没有发送清故障指令；先前失败证据保留。USB控制线/复位的具体电气连接仍未测量，不将时序改善写成硬件根因已证实。

## 烧录与工具

本机使用pyOCD0.46.0、IntelHex2.3.0、Keil本地STM32F4xx_DFP1.0.8。探针8801490222005A504E413836；实物设备ID0x413、Flash1024KiB，UID为550040001951333136363733。无自动解锁、无全片擦除、无选项字节写入。

初次下载算法初始化超时。只读现场为CCM指令取值IBUSERR；本机旧DFP把不可执行CCM标为默认RAM。工具仅打包所需元数据/算法，将输出包中的CCM default设为0，源设备包不改。下载工具进一步拒绝算法加载/栈位于主SRAM以外。修正后0.3.1、0.3.2、最终0.3.3均成功读回；失败和中间版本不当作最终验收。

```powershell
.venv\Scripts\python.exe -B tools/make_flash_pack.py --source D:/Kiles/ARM/PACK/Keil/STM32F4xx_DFP/1.0.8 --output output/flash/Keil.STM32F4xx_DFP.1.0.8.sram.pack
.venv\Scripts\python.exe -B tools/flash_base.py --probe 8801490222005A504E413836 --pack output/flash/Keil.STM32F4xx_DFP.1.0.8.sram.pack --output output/flash/program_033_20261024 --program --hex firmware/MDK-ARM/Objects/Car.hex --expect-sha 856babddf0bcde2181fe3825eb4afec36e04b0e96f030121509c6a3c92931c44 --expect-uid 550040001951333136363733
```

以上为本次已执行命令，输出目录不可复用；重建后必须重新核对HEX摘要，不能盲抄旧摘要。工具默认不带--program时仅识别，不刷写或主动复位。写入前停核读取完整1MiB旧固件备份，擦写后校验全部有效字节，再复位运行。新固件仅启动STOP和必要反馈查询，不自动使能或运动。

最终HEX SHA256为`856babddf0bcde2181fe3825eb4afec36e04b0e96f030121509c6a3c92931c44`。原CARCAN13完整备份SHA256为`c8afef34ea1fcf507eb8a98e1acebca0dd80845c0dcf627e15139b6045c7f31a`，保存在output/flash/program_20261024/previous.bin及首个成功写入目录。备份不自动恢复；恢复旧固件前须重新确认停止及协议所有权。

## 树莓派部署与只读入口

独立部署目录为/home/pi/gongchuang_v0131，用户名pi；密码不落盘。旧/home/pi/car_web代码和service文件未修改，car-web.service已停止并disable，防止重启后旧协议抢占串口。不要在GC固件上启动旧服务。

唯一串口为/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0。只读验收实际执行：

```sh
cd /home/pi/gongchuang_v0131
PYTHONPATH=pi .venv/bin/python -B tools/accept_base.py --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --output output/acceptance_033_settle.json
```

工具只允许HELLO/OPEN/HEART/STATE/DIAG/FEEDBACK/EXEC_STATE/UART_DIAG，未发CONFIG/ARM/GOAL/STOP。固件自身启动STOP与工具命令分别记录。

新只读页：http://172.20.10.4:8081/。gongchuang-observe.service是本轮临时核心，使用robot.drive_core --observe --seconds 3600，独占串口；gongchuang-web.service运行robot.web.app --port 8081，读取同一output/runtime。两者均经systemd-run启动，不设置开机运行。核心一小时到期或设备重启后页面应显示过期，不自动续跑。旧8080已关闭，本机8000仍离线模拟。

接管下一个短段前先停止观察所有者，核对串口已释放；不要并行打开两个核心。重新打开串口需按本车可能复位处理，重新建立启动身份和配置，机构零点不得沿用。观察结束/失联可能锁存停止原因，不得自动清锁或接着发目标。服务状态查看：

```sh
systemctl is-active gongchuang-observe.service gongchuang-web.service car-web.service
systemctl is-enabled car-web.service
fuser -v /dev/ttyUSB0
```

实测为active/active/inactive、旧服务disabled，串口仅新核心一个PID。在线API核对firmware=GC-BASE-0.3.3、mode=DISABLED、fault=0、stop_confirmed=1、config_version=0、action_id=0，CAN序号持续推进。Web只有GET和WebSocket，无运动入口。

## 本次验证与证据

- Keil ARM6.7完整重编译0错误0警告；Code102528/RO912/RW124/ZI27756。941项阶段检查通过，IOC、时钟、引脚未改，本轮不重复生成外设。
- 执行22项、只读协议/Web23项、验收工具/串口启动9项，共54项定向回归通过。前节点190项全量回归保留为历史，本次未重复耗时导航全套。
- 原始日志、备份和板端样本在output/flash/，Git仅保存本摘要和[摘要索引](board_readonly_evidence.json)。acceptance_031/032/033是失败记录，只有acceptance_033_settle通过。
- Pi原始验收文件已下载到output/flash/pi/；通过文件SHA256为760aa0f9787735314805a49a5434896f34768f4f831af2f344aa5e7c294dfd03。原始数据不进Git。

下一步按前进100mm→左移80mm→逆时针10°分别验证，用户独立尺量并确认方向和停稳，每次动作前先告知。跳过重复单轮测试。完成后再接斜线、弧线及单段路线；当前没有整图执行许可、地图配准或完整外形证据，也没有机构动作或真实成功计数验收。
