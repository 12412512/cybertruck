# 视觉分支负责范围与第一轮开发记录

用户于2026-10-30明确：B负责完整视觉链，包括六色、环号、扫码、抓放结果确认与性能分析；优先解决漏检和不稳定。沿用现有算法，保留250ms过期拒绝，固定已记录的观察姿态，抓放成功必须有独立证据。

分支`codex/vision-optimization`基于当前B已交付源码，不改A的设备执行核心。上一批样本工具与历史回放在PR #1单独审查；本分支的开发参数草稿不自动部署或应用。

## 现场所有权待交接

用户允许B接管视觉采集，但以先与A交接为条件。当前尚未取得A的交接确认、现行网站地址及已核验的Pi身份，不能沿用历史IP、运行PID或旧控制状态。

交接应保持原唯一相机采集服务和设备核心，不启动第二个VideoCapture/串口所有者。A暂停视觉代码、配置及采集操作的并发写入，交给B管理现有视觉服务的操作与版本；底盘、机构执行权继续归A。需要机构移动到观察位时仍由A按现场流程安排，不由B的识别器驱动。

A需提供的非凭据信息：当前调试网站地址、Pi已核验的设备身份/IP、实际视觉服务目录与PID/stream、两只相机by-id和分辨率、当前配置SHA及是否正在运行其它采集者。密码、密钥与活动控制令牌不进文档/仓库。接管前后分别留只读状态快照；无交接确认时B只离线工作。

## 已核实的视觉基础

原交接包115个包文件/108个数据文件SHA通过，56张图片（55去重）、46份工作台记录。六色各4份条件名称提示，全部24份来自同一stream，不能拆成独立训练/测试会话。9张旧图的18组六色/圆环结果与原归档完全一致。

原识别入口使用LAB/HSV与轮廓、数字模板、OpenCV二维码。历史Pi中的NCNN/ONNX参考目录不是当前默认链路，本分支不替换算法。

## 五张未检出图的定位

| 样本 | 原始诊断 | 下一步 |
| --- | --- | --- |
| 黑色_稍转角_19 | 外形距离约0.096通过，但颜色质量0.527低于0.55，颜色证据拒绝 | 补代表性表面样本；该物料位置也未完全落入原抓取ROI，不能只恢复检测就准许抓取 |
| 红色_反光_12 | 原目标颜色模型支持不足，背景出现不确定区域 | 扩充表面颜色参数并核对新增背景候选 |
| 黄色_反光_10 | 目标颜色强支持不足 | 同上；需要新会话的明亮/反光负例 |
| 蓝色_反光_08 | 同一物体内蓝/浅蓝歧义，原dominance约0.689低于0.85 | 以代表性原色样本修正分类，保留歧义拒绝门槛 |
| 绿色_反光_07 | 目标区域强颜色支持约0.109不足 | 扩充表面样本，独立验证阴影/反光下的背景误检 |

以上为原检测器输出及图像复核，不把所有不确定区域都当物料。相机或目标姿态不符、抓取ROI未覆盖时，识别正确也不能证明可抓。

## 原算法参数开发消融

`docs/vision_color_development_rois.json`记录5张原图SHA和经B图像检查选择的物料表面ROI；颜色沿用原操作者选择，ROI仍待A确认。`tools/material_color_ablation.py`只调用原sample_color与merge_color_sample，不改阈值、形状、ROI、稳定窗口、分辨率或算法。不进行灰参考/白平衡补偿。

回读全部56张原图，得到：

- 5张拟合图：所选颜色有效候选由0/5到5/5。
- 其余19张六色条件图：所选颜色有效候选维持19/19。
- 两者是开发回读，所有24张属于同一采集流；既不是独立验证，也不是完整物料框精确率。参数草稿`applied=false`，不能自动用于任务。
- 全部56张图仅这5张的有效候选颜色列表有变化；这不排除候选框或背景误检，仍需逐框人工标注。

需要在接管后的独立新会话补齐六色、正常/明亮/阴影/反光、黑线/夹爪/空背景、固定姿态及不同实测角度/高度；新帧不得回流开发拟合组。新增候选、中心误差和处理时延需单独统计。不能因24张回读达到提示一致就宣布现场优化完成。

报告保留每张图的新旧候选框、中心、分数与形状一致性，供A逐框复核；没有中心真值时不计算中心误差。本机最终参数消融输出位于忽略目录`output/vision-optimization/color-ablation-v3/`。复现（输出选择尚不存在的目录）：

```powershell
.\.venv\Scripts\python.exe -B tools/material_color_ablation.py --package C:\Users\pjb11\Desktop\协同\Computer_B_Handoff --output output/vision-review/color-ablation
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_vision_color_ablation.py -v
```

本轮仅为完整视觉任务的首批开发记录；还没有接管设备、部署参数或完成全链实物验收。环号、二维码时序和抓放结果确认的联合验收继续按固定观察位、独立证据与唯一采集者推进。

## 等价的内存优化

`legacy/material_vision/core.py`的颜色得分按原顺序逐样本取最小值，减少保留全部样本距离图及其reduce堆叠；最优分数复用原次优分数分区的第一层。原LAB距离、HSV门控、颜色歧义、质量、形状、ROI及稳定性阈值不变；默认配置文件逐字不变。

独立旧得分公式覆盖单/8样本、HSV边界、逐元素float32比对；9个合成场景比较候选/拒识/歧义。另在全部56张真实历史图比较这三类完整输出，56/56一致；原18组六色/环号归档比较继续完全一致。这验证实现兼容，不说明原算法已有足够精度。

本机Python 3.11.9、OpenCV 5.0.0、NumPy 2.4.6、OpenCV单线程；9张旧图预热后交替AB顺序7轮，每组63次，排除读盘、解码、序列化和内存跟踪开销。对照只替换旧得分聚合块，两组共用等价的最优/次优排序：

| 项目 | 旧得分聚合 | 流式最小值聚合 |
| --- | ---: | ---: |
| 检测器中位数 | 26.19ms | 24.12ms |
| 检测器P95 | 31.15ms | 29.27ms |
| 得分内核tracemalloc峰值 | 8,945,960字节 | 7,390,208字节 |

本次中位数降低约7.9%，内核跟踪分配峰值降低约17.4%。tracemalloc不是进程RSS，时间有本机调度波动；这不是Pi性能、双相机吞吐或250ms端到端验收。完整报告在`output/vision-optimization/score-performance-v1.json`。复现：

```powershell
.\.venv\Scripts\python.exe -B tools/benchmark_material_scores.py --package C:\Users\pjb11\Desktop\协同\Computer_B_Handoff --output output/vision-review/score-performance.json
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_material_score_equivalence.py -v
```

## 完整视觉链的离线覆盖与现场缺口

| 链路 | 当前离线证据 | 交接后必须补齐 |
| --- | --- | --- |
| 六色与中心 | 原包回放、上述参数消融、开发/验证会话隔离与混淆统计工具 | 独立会话的六色框/中心真值、光照/角度/高度实测值；空背景、黑线、夹爪等负例 |
| 环号1/2/3 | 原环检测历史兼容、数字模板等价、错误环号与旧帧拒绝 | 每个实际环号的近/远、偏角、遮挡正负例；固定观察位与真实中心标注 |
| 扫码 | 原OpenCV QRCodeDetector、四组任务码解析测试；任务与重复帧边界回归 | 实际纸码及现场不同曝光/距离/倾角连续序列；无效码、多码、重复/过期帧与处理时延 |
| 抓起/存车/取出/释放确认 | 单件/两批模拟账本、重复/乱序/未知结果与错误颜色环号拒绝；候选不计成功 | 同一动作前后独立证据，含真成功、未抓起、掉落、错误槽/环、残留、遮挡和无法确认；由A核实实物结果 |
| 性能和时效 | 本机等价比较；250ms拒绝及上下文绑定回归 | Pi唯一采集服务的捕获/推理/队列/编码/传输/消费时间戳，处理P50/P95、帧龄及过期率 |

第一批增量优先重拍上述5个漏检条件，并保留未参与拟合的新会话对照；不能把本轮ROI或旧图复制进独立验证。后续按照六色条件清单补覆盖，现场能达到的角度/高度由A确认并实测，不从文件名猜。所有样本绑定相机、分辨率、配置SHA、固定姿态ID/版本/SHA、目标平面、session/stream、时间与人工真值；姿态变更须分组。物理中心误差需实测标定，像素误差不能直接当毫米误差。

沿用网站现有唯一采集服务保存原图和记录；不启动第二采集进程，不恢复交接包的旧会话。A交接前，本分支继续只读离线回放。

## 回归记录与交付状态

本轮142项测试通过，0失败、0错误、0跳过；包括任务/证据、视觉工作区、迁移、样本审计、开发消融、得分等价、环号模板、到位观测与对中主机闭环。1,146项工程静态检查通过。日志在`output/vision-optimization/checks/regression-final.log`及同名JSON；原轮次日志保留。初轮对中主机C夹具因缺少ziglang未运行，按`tests/requirements.txt`补装固定的0.16.0后单独通过，再全量重跑上述142项全部通过。C主机夹具不是实板验收，没有编译/烧录目标固件。

复现测试时先配置当前仓库`pi`和`tests`到PYTHONPATH，并指定原样本只读位置：

```powershell
$env:PYTHONPATH = (Join-Path (Get-Location) 'pi') + ';' + (Join-Path (Get-Location) 'tests')
$env:ROBOT_VISION_SAMPLE_ROOT = 'C:\Users\pjb11\Desktop\协同\Computer_B_Handoff\data\output\vision'
.\.venv\Scripts\python.exe -B -m unittest test_task test_task_replay test_task_evidence test_vision_workspace test_vision_migration test_vision_color_ablation test_material_dataset test_material_score_equivalence test_ring_digit_performance test_arrival_vision test_centering -v
.\.venv\Scripts\python.exe -B tools/check_project.py
```

当前交付为本地视觉分支的离线候选。未连接Pi、相机、串口或控制网站；未部署、未应用颜色参数。A未交接前不进行设备操作，现场识别精度、独立样本覆盖、Pi性能和抓放实物确认均待验收。后续仍通过Fork交审，不直接修改或合并上游main。
