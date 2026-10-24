# 导航与坐标

`frames`负责SE(2)及显示边界；`reference`绑定正式C参考；`geometry/field`负责名义场地、车体及扫掠；`planner`负责带朝向道路搜索、共线合并和可行弧段。正式实现不从tools/tests导入，不读取或控制串口。

唯一地图候选源为根目录`config/field_preview.json`，仅simulation；默认UNKNOWN道路不通过。蓝区可以行驶，真实可执行始终关闭，未测完整包络不能按主体尺寸认证。模型入口为`robot.route_sim`，详见`docs/navigation.md`。
