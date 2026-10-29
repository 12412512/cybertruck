# 新PCB引脚配置

2026-10-25，按用户指定调整两个模块引脚并新增一路舵机PWM。本轮只进行配置、实际CubeMX生成、离线回归和Keil编译，没有连接设备、烧录、清故障、使能或实车动作。

| 功能 | 原引脚 | 新引脚 | 保留配置 |
| --- | --- | --- | --- |
| CAN1_RX | PA11 | PD0 | AF9、500kbit/s |
| CAN1_TX | PA12 | PD1 | AF9、500kbit/s |
| 低有效蜂鸣器 | PB0 | PG11 | 推挽、无上下拉、低速，初始化先置高再配置输出 |
| 新增舵机预留 | 无 | PE14 / TIM1_CH4 | AF1、50Hz，Pulse=0，标签AUX_PWM |

原料盘PE9/CH1、转臂PE11/CH2、夹爪PE13/CH3不变。新PE14只生成PWM配置，不并入原三路舵机状态机、姿态或Web动作接口，也没有新增角度/限位假定；默认不启动PWM。用途及机械参数后续确认后再接入正式互锁与调试流程。

CAN仍使用Prescaler=4、BS1=12TQ、BS2=2TQ，自动重发关闭、原总线调度及保护不变；三个CAN IRQ优先级仍为2。TIM1继续120MHz时钟、PSC119、ARR19999，四路初始Pulse均为0。系统120MHz、串口115200/8N1、PC0启动、PC1选区、SWD及其他已配置外设不变。旧PA11/PA12、PB0释放，不自动启用STM32原生USB或其他功能。

## 生成和验证

唯一硬件源为firmware/Car.ioc。使用工程既有tools/generate.ps1启动实际CubeMX命令行生成，再由sync_project.py恢复Keil工具链和手写引用；没有手改生成区绕过CubeMX。已核对main.h标签、can.c的GPIOD初始化/反初始化、GPIOG时钟/蜂鸣器初始电平、tim.c四路配置和PE14复用、NVIC及中断入口。

变更前配置及必要生成文件备份于output/hardware/pcb_pins_20261025/before。逐项对比IOC，只有指定引脚、引脚枚举及TIM1_CH4相关新增字段变化，其他功能参数无差异；结果见scope_verification.json。所有生成C文件USER CODE段保留，手写C/H只有Comm版本标记由0.3.20变为0.3.21，动作算法/参数不变。

为避免旧PCB镜像混淆，新候选标识GC-BASE-0.3.21，保留动作协议和version9选区配置。主控仅增加该版本的兼容识别，不解除原执行许可、会话、250ms反馈、500ms保活、故障停车或互锁。当前设备核心未重启/部署，板端最后确认仍是0.3.19，不用旧只读快照代表当前状态。

| 检查 | 本轮结果 |
| --- | --- |
| CubeMX实际生成 | 成功，main.c生成回执及最终OK |
| 工程静态检查 | 1108项通过 |
| 旧底盘执行回归 | 60项通过 |
| PC1选区及两区闭环/扫掠回归 | 10项通过 |
| Keil保护策略 | 4项通过 |
| Keil完整重编译 | 0错误、0警告，Code132800、RO4424、RW152、ZI31872字节 |
| 新引脚电平、CAN实物通信、PE14波形 | 未验收 |

日志均在output/hardware/pcb_pins_20261025：generate.log、keil.log、execution.log、route_select.log、keil_policy.log；CubeMX原始回执已另存cubemx.log和cubemx-errors.log。候选HEX/AXF归档output/firmware/candidate_0321，programmed=false。HEX SHA256为64744dad98f02f469c877bc675f4c96a4eb74d90877ee126b65ef77a82e216a8。

下一步待用户确认新板接线与烧录条件，再使用本轮.21镜像烧录。成功后先通知、只读核对新的CAN/蜂鸣器/选区输入，等待该次烧录后的放地确认及PC0才执行路线。PE14仍保持关闭，新增舵机实际动作另行告知并验收。
