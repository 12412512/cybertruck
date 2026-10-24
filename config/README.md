# 上位机配置

后续保存地图、设备绑定、机构预设姿态与视觉标定。当前不复制旧配置，不填入未测量数据。

每类参数应明确唯一来源、单位、版本和适用设备；执行限位与保护参数归 `firmware/User/Config/`，硬件生成参数归 `firmware/Car.ioc`。

实际凭据与本机环境放入 Git 忽略的 `.env` 等文件，仓库仅提供必要模板。

`field_preview.json`是本轮唯一名义场地候选源：map米/弧度、主体四边偏置、净空、候选道路、工位朝向。domain=simulation且measured=false，完整包络complete=false；不能作为真实部署配置。当前蓝区可通行，起终点管理与颜色分开。新道路默认UNKNOWN，离线演示的OPEN场景在内存显式建立，不回写为实测证据。
