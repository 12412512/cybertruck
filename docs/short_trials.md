# 低速短段实车记录

2026-10-24，基于4ce36ab/v0.14.0-board-readonly，节点标签v0.14.1-short-trials。固件仍为GC-BASE-0.3.3，本轮没有改固件或再次烧录；配置base_trials.json版本2，yaw_sign=-1，hardware_verified继续为false。

用户逐项确认落地、净空、机构收拢和可立即断电，动作前分别告知。先停止只读核心、检查串口无占用，显式执行一个短段，完成/停止后关闭串口；未执行单轮、斜线、弧线或整图。

## 实际结果

| 项目 | 目标 | 板端里程计/IMU结果 | 目标下发至控制器完成 | 用户现场观察 |
| --- | --- | --- | --- | --- |
| 前进 | 100mm | X约102.74mm，Y约-0.09mm，航向约-0.060° | 4.278s | 正前方正常，已停，实际距离未量 |
| 左移 | 80mm | Y约82.45mm，X约-0.02mm，航向约+0.005° | 3.604s | 左移正常，车头基本不变，已停，实际距离未量 |
| 逆时针转向 | 10° | 相对航向约+10.102° | 2.688s | 逆时针，目测约10°，已停，未独立量角 |

三段控制器均报告完成并确认停止，所有记录中的CAN错误为0，运动目标各下发一次、不重放。前进列为最后周期里程计，后两列为最终停止帧；不能用这些内部数字代替尺量或独立量角。实车方向与航向闭环初测正常，几何、编码器比例和绝对精度尚未标定，不能据此将hardware_verified或地图executable置真。

角度原始顺时针增加，经yaw_sign=-1转换为逆时针正；本轮实体转向与控制器正角度一致。Z角速度的动态记录和独立尺度检验仍未完成，不把目测10°写成陀螺精度验收。

## 本次曲线与命令

三个短段均使用正式Curve/Ref的七段限加加速度S曲线，再由Track和真实反馈闭环修正；实际轮速还受6RPM上限、量化和误差修正影响，不宣称实测轮速严格重现理想曲线。

- 平移：速度0.03m/s，加速度0.08m/s²，加加速度0.3m/s³。
- 转向：角速度0.10rad/s，角加速度0.3rad/s²，角加加速度1.0rad/s³。
- 每个目标超时20秒，单次核心整体40秒；末尾独立STOP并核对反馈，不根据时间到冒充到位。

Pi工作目录/home/pi/gongchuang_v0131，依次执行下列case，每次在现场观察通过后才进行下一条。这里保留本次命令，不是自动批量脚本：

```sh
PYTHONPATH=pi .venv/bin/python -B -m robot.drive_core --case straight --execute --accept-candidate --site-ready --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --owner-url http://127.0.0.1:8080 --runtime /home/pi/gongchuang_v0131/output/runtime
PYTHONPATH=pi .venv/bin/python -B -m robot.drive_core --case side --execute --accept-candidate --site-ready --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --owner-url http://127.0.0.1:8080 --runtime /home/pi/gongchuang_v0131/output/runtime
PYTHONPATH=pi .venv/bin/python -B -m robot.drive_core --case turn --execute --accept-candidate --site-ready --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --owner-url http://127.0.0.1:8080 --runtime /home/pi/gongchuang_v0131/output/runtime
```

打开USB串口可能复位板，每段是独立启动身份、独立局部里程参考，动作ID均为1；不能将三段坐标直接相加成连续地图轨迹。前进boot=3685372962475046266，另外两段身份保存在摘要索引。后续连续路线需要保持同一会话及连续odom，并独立确定地图配准。

## 停车帧保存修复

首段核心stop_result记录了confirmed=true，但周期快照限频，最后持久帧仍是STOPPING。用户独立确认已停；原始证据保留，不补造该段最终帧。之后将停止等待阶段改为强制写入快照，同时在完成/停止事件中保存所依据的完整执行状态。没有更改目标、控制计算、速度参数或故障处理，也没有为补日志重复前进。

左移及转向的最终帧均为OBSERVING、stop_confirmed=1、fault=0、四轮命令0 RPM。正式GC/C/Plant的22项执行回归通过，新增断言最终快照与stop_result执行状态一致；测试日志output/field/straight_100_20261024/execution-tests.log。主机测试不替代本轮实物观察。

## 证据与结束状态

原始归档output/field/short_trials_20261024.tar.gz及解压目录包含三份JSONL、命令结果和最终状态；摘要、启动身份和SHA256见[证据索引](short_trials_evidence.json)。原始资料不进Git。没有增加任何真实抓取/放置成功计数。

本轮结束后串口无占用，gongchuang-observe.service和旧car-web.service均inactive，gongchuang-web.service仍active。执行核心按设计退出，8081会显示核心/设备状态过期，这不是新鲜停车或故障状态；不得以旧页面数据开始下一次动作。没有自动重新打开串口或恢复运动。

下一步短斜线，再进入有界弧线；动作前告知并核对当前空间。首次试验保留低速，运动前标记固定点，停止后独立量距/量角。整图还需完整外形、地图配准、连续会话及相应轨迹出口；不直接从三段初测跳到高速整图。
