# 雷达观测

本目录提供CYQM纯协议、通信状态机、离线扫描一致性检查及模拟规划参数转换，不自行打开串口、不写标定、不占用STM32接口。真实串口由外层唯一`robot.lidar_core`管理，与STM32核心分开。

| 模块 | 职责 |
| --- | --- |
| `protocol.py` | 旧工程v6 COBS/CRC、HELLO、存储记录和完整抽样点云组装 |
| `health.py` | 旧01/02/13只读握手、匹配ACK/数据、500ms保活、有界超时和扩展事件隔离；不扫描或改区 |
| `geometry.py` | 用户安装参数的纯平面换算；只接受明确的lidar_local点，拒绝厂家场地坐标重复加外参，不授权动作 |
| `review.py` | 从结束的实扫与尺量样本生成核对摘要；保留独立点和配准/覆盖待验收状态，不接设备 |
| `vendor_calibration.py` | 厂家15字节校准结果解码及保存内容一致性检查；只读纯逻辑，不提供09发送入口 |
| `calibrate.py` | 只读新备份→版本绑定→提交前重读→单次09→结果/保存/HELLO/STOP核对；取消与失败分开 |
| `contracts.py` | 静扫、校准准备与提交的有界请求，供网关及唯一核心共用 |
| `replay.py` | 连续扫描窗口、STOP、完整点云与GRID/RESULT配对、质量和时序检查 |
| `fixtures.py` | 厂家线格式的合成报文与明确的模拟标定/覆盖；含正常及负例 |
| `planning.py` | 厂家障碍候选转换到模拟map、未知道路排除及扫描来源绑定 |

协议来源是只读参考`C:\Users\RISI\Desktop\ros小车\lidar\cyqm_protocol.py`，本轮来源SHA256：`124b316f7d6d9a4048971b36f2be2cf8003d3df93504ec81989ae22a0e14b839`。主要解码实现原样复用，注释改为中文；GRID/RESULT字段沿用旧`cyqm_lidar.py`，没有从抽样点云重新发明厂家障碍定位。只迁入纯协议，不复制旧工程服务、设备接管或日志。

`scan.py`提供有界静态扫描，`calibration.py`核对实测点和地图的几何关系，两者均不直接访问设备。真实静扫与目标有/无对照已完成，三点地图配准因反射关系和间距偏差被拒绝，未应用变换。当前不增加07/08/14写命令，也不授予路线执行权限；详见根目录`docs/lidar_static.md`。基础通信与历史配置见`docs/lidar_live.md`，模拟回放见`docs/lidar_offline.md`。

原8000新增安装/四点核对与后续自动运行契约，使用tools/export_lidar_review.py导出只读摘要。树莓派接入边界及同一规划器/执行器的验证见`docs/lidar_autoplan_handoff.md`；当前真实坐标和道路覆盖尚未验收，不因页面新功能开放实车自主跑图。

最新受控校准事务及树莓派只读准备见`docs/lidar_calibration_link.md`。当前部署只开放prepare，未开放09写入；CAN0x800和所有底盘/机构保护保持，真实两点校准尚未发生。
