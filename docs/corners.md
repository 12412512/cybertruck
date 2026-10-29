# 弯道与低速转姿优化

2026-10-24，从25f42e3 / v0.11.0-speed-parking-sim推进，节点v0.12.0-corners-sim。直线巡航保持0.30m/s，线加速度0.25m/s²、线jerk0.8m/s³、模型轮目标100RPM、停车和扫掠保护保持，不用提高直线速度制作节时结果。

## 实现

- 正式C新增Ref_Curve，按每格七次多项式系数求一至三阶几何导数界。常标量弯速下a=r''v²、j=r'''v³，据此求弯速，留0.5%数值余量，替代原统一半加速度/半jerk系数；加减速仍在首末直线。完整Ref_At继续校验全部笛卡尔和角V/A/J。
- FLOW车身转姿可覆盖部分入弯/出弯直线，不把全部90°旋转压在几何弯段内。若时长不足，最多12次有界搜索，只保留满足转姿时长的速度。位置、速度和加速度在连接处连续。
- 180°直线转姿在±6～80mm外增加±120～500mm窗口候选；逐个检查完整主体扫掠、轮速，碰撞窗口直接拒绝，不强制放宽。
- 分别比较弯道/窗口0.7、1.2、1.5rad/s模拟角速，角加速度2rad/s²、角jerk12rad/s³不提高。普通POSE和独立转向仍用原参数；请求仅为上限，轮速预检、降档和独立Plant验证仍生效。

必要工位停车、零速批次确认、发送后新反馈及150ms稳定窗口保留，不能跨任务工位合并或取消到位门槛。区域返航不要求原中心点及朝向，投影和稳定停车仍检查。

## 验证口径

同一名义地图与合成车体，全部固定0.30m/s，六场景均COMPLETE，源/配置/库哈希一致。证据为output/corners/release/summary.json及六份完整报告：

| 场景 | 弯道/窗口角速上限 | 模型时间 | 停稳段 | 轮目标峰值 |
| --- | ---: | ---: | ---: | ---: |
| 起点1旧策略 | 0.7rad/s | 94.77s | 9 | 71RPM |
| 起点1新窗口与几何限速 | 0.7rad/s | 90.65s | 9 | 72RPM |
| 起点1本版采用 | 1.2rad/s | 84.29s | 9 | 84RPM |
| 起点1较高角速对照 | 1.5rad/s | 83.93s | 9 | 85RPM |
| 起点2旧策略 | 0.7rad/s | 100.33s | 11 | 100RPM |
| 起点2本版采用 | 1.2rad/s | 89.41s | 10 | 84RPM |

起点1仅窗口/曲率算法即省4.12s，组合角速安排后共省10.48s；起点2省10.92s并减少1个停车。1.5档仅比1.2档快0.36s，因此本版采用1.2，不追逐较高角速。其余必要工位停稳没有删除。独立Plant轮速峰值分别约80.30/80.60RPM，目标上限仍100RPM。

准备墙钟与运动时间分开；离线搜索仍有明显准备开销，后续应在启动前准备候选并实测在线重规划时限。两批跨区小计仍缺工位内移位和视觉精对准，60秒尚未达到，不能作为完整导航或整机180秒验收。

用户本轮澄清300mm是杆件长度，完整外形仍未确认，原主体complete=false，不改成整车宽300mm。实测入口和测量更正见[field_test.md](field_test.md)。

```powershell
.\.venv\Scripts\python.exe -B tools/build_odom.py --output output/native/corners.dll
.\.venv\Scripts\python.exe -B tools/tune_corners.py --library output/native/corners.dll --output output/corners/release
.\.venv\Scripts\python.exe -B tools/tune_corners.py --library output/native/corners.dll --output output/corners/release --collect
```

case可独立并行，collect核对源码/配置/库哈希，不混用中间结果。robot.route_sim支持legacy-corners和turn-rate；仅主机模型，无串口/CAN输出。继续复用既有S曲线、路径几何、跟踪器、连续odom和独立Plant，旧参考工程只读。

162项完整主机回归通过，818项静态检查通过，覆盖宽窗口中间碰撞拒绝、扩大转姿范围后的V/A/J、导数界限速、角速档保持A/J及异常输入。日志output/corners/release-tests.log；Keil ARM6.7完整重编译0错误0警告，Code13664、RO448、RW16、ZI8336。HEX SHA256仍为96e719c8b7c80d9630f07d7fd335d330093a0bbab6f3f11ebf813f616271d922，新计算函数未接App执行。

本机8000已核对0.30m/s、1.2rad/s、新窗口和84.29s模型时间、9/9到位、原点连续及区域返航；模型近原地转姿降至0.00s，独立停车转向0次，必要工位停稳保留。截图output/corners/web-summary.png，状态摘要web-final.json；真实位姿/成绩仍未知，旧模拟账本保留但不随本轮推进。Pi8080及本机8001未改动。

IOC和App入口无变更，不重新生成、不烧录、不清实车故障。离线版完成后按用户要求通知上电，实测从真实状态和只读基线开始；本页不声明已完成实车运动。
