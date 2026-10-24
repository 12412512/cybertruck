# 导航与坐标

`frames`负责SE(2)及显示边界；`reference`绑定正式C参考；`geometry/field`负责名义场地、车体及扫掠；`planner`负责带朝向道路搜索、共线合并和可行弧段。正式实现不从tools/tests导入，不读取或控制串口。

唯一地图候选源为根目录`config/field_preview.json`，仅simulation；默认UNKNOWN道路不通过。蓝区可以行驶，真实可执行始终关闭，未测完整包络不能按主体尺寸认证。模型入口为`robot.route_sim`，详见`docs/navigation.md`。

当前起点1为0°、起点2为180°，扫码0°；X优先与短侧移限制已取消。`FLOW/POSE/WINDOW`均绑定正式C，拓扑搜索先按时间选择连续候选，再合并冗余点；重叠道路共享保守通行状态。默认8段两批跨区小计，工位内移位与视觉修正尚缺，不作完整60秒验收。现行说明见`docs/flow.md`。

提速节点的现行说明见`docs/speed.md`。巡航上限按Planner实例冻结；参考轮速以正式C全程/转姿区间界分别计算，必要时最多4次降档重建。返航在所选蓝区选择四个轴向的可停放目标，完成时核对odom及独立Plant的名义主体投影；不要求原中心点或出发朝向。
