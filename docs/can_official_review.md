# CAN官方资料核对（2026-10-26）

本轮只核对源码、既有故障记录及官方资料，未连接设备、复位、清故障、烧录或触发动作。用户已确认主控板焊有8MHz外部晶振；不是25MHz。

## 优先发现：正常CAN仍使用内部HSI

当前Core/Src/main.c的SystemClock_Config选择HSI作为PLL输入，M=16、N=240、P=2、Q=5。主频120MHz，APB1为30MHz。Car.ioc及stm32f4xx_hal_conf.h虽然声明HSE_VALUE=25000000，但并未启用HSE，该值不能代表实物晶振或当前时钟源。

ST官方技术答复明确不推荐正常CAN使用内部RC时钟，建议外部晶振或其他精确时钟。当前配置与该建议不符，应优先修正。它是需要验证的高优先级原因，不是已经证明的现场TERR唯一根因。此前排查应更早优先核对这一项。

下一候选建议使用HSE晶振模式8MHz，PLL M=8、N=240、P=2、Q=5，继续保持主频120MHz、APB1 30MHz和PLLQ 48MHz。同步更新IOC、生成代码、HSE_VALUE及工程检查；检查HSE就绪与PLL实际时钟源。不得只改头文件或把8MHz无源晶振设为外部时钟旁路。若HSE未就绪，不静默退回HSI继续运行CAN。

## 已核对项目

| 项目 | 当前实现及判断 |
|---|---|
| CAN位时序 | Prescaler=4、BS1=12、BS2=2、SJW=1；30MHz/(4×15)=500kbit/s，采样点13/15=86.67%，处于ST给出的500k推荐范围。名义计算正确，不代表HSI实际频率稳定。 |
| 厂家帧格式 | 扩展帧ID=(地址<<8)\|包号，每包首字节功能码，固定校验6B；查询及FD分包与手册42～44页一致。请求/回包同ID为厂家协议，不能单方面更换。 |
| 查询调度 | 当前已等待TXOK和匹配新回包，并有周转与RX空闲检查；不能再将旧版本“只等待发送”描述为当前行为。手册37页同步命令示例要求几毫秒间隔，没有给出所有查询统一10ms的依据。 |
| 错误分类 | 实测TERR0/LEC4对应发送失败/隐性位错误；LEC3才是ACK错误。当前ESR=0x80040时TEC=8、REC=0、BOFF=0，不能称为bus-off，也不能当正常仲裁丢失。 |
| HAL诊断 | 当前未启用LAST_ERROR_CODE通知，所以HAL错误仅TERR而冻结ESR带LEC4并不矛盾。错误警告/被动状态的过程记录可后续补齐，不将诊断缺项当作已证实的TERR起因。 |
| 芯片勘误 | ES0182说明时间触发通信不支持；当前TimeTriggeredMode关闭，符合要求。 |

## 第二优先级：诊断计数与停车策略耦合

当前AutoRetransmission关闭，base_port的Fault直接包含累计can_diag.errors；一次真实发送错误即可使后续动作持续被拒绝。这是工程选择，不是CAN协议要求所有瞬时错误永久终止任务。

先完成HSE单变量验证；若仍有短暂查询失败，再设计只读查询的有界恢复，分开历史错误统计与当前不可恢复故障。需要保证新鲜匹配回包、总事务期限、STOP优先及250ms反馈保护；持续错误、bus-off、过期和失联仍停车。不能仅屏蔽errors，也不能给FD/F3等动作命令无条件自动重放。本轮没有更改重发或保护策略。

## 现场证据边界

最近已烧录版本是0.3.28。其启动1620ms首次ID2/36出现TERR0/LEC4，前一ID2/35事务于1615ms已有TX/TXOK/RX，查询超时0。该证据不能证明同ID同时发送碰撞，也不能仅凭60Ω电阻或更换模块认定全部硬件因素已排除。当前故障现场保留，见board_0328.md。

## 资料

- ST技术答复，HSI与HSE用于CAN：https://community.st.com/stm32-mcus-products-25/hsi-vs-hse-for-can-136317
- ST bxCAN位时序配置：https://community.st.com/stm32-mcus-60/can-bxcan-bit-time-configuration-on-stm32-mcus-135466
- STM32F407数据手册DS8626：https://www.st.com/resource/en/datasheet/stm32f407vg.pdf
- ST HAL CAN源码：https://raw.githubusercontent.com/STMicroelectronics/stm32f4xx-hal-driver/master/Src/stm32f4xx_hal_can.c
- STM32F405/407勘误ES0182：https://www.st.com/resource/en/errata_sheet/es0182-stm32f405407xx-and-stm32f415417xx-device-errata-stmicroelectronics.pdf
- 本地厂家手册：C:\Users\RISI\Desktop\ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527.pdf，第37、42～44页；文本提取归output/can-official-review/motor_manual.txt。

本轮仅新增审查说明和忽略目录下的提取材料，未修改固件，不单独提交Git。
