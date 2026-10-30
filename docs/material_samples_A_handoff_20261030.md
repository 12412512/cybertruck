# A端配合：原六色算法的样本补齐与参数对照

2026-10-30，B端；对应Fork分支`12412512:codex/vision-mission`及[PR #1](https://github.com/risiairline-dotcom/cybertruck/pull/1)。

## 本轮边界与当前实情

沿用当前`MaterialDetector`的LAB样本距离、HSV门限、轮廓与形状保护。原算法、阈值、识别入口、相机配置和正式颜色配置均未修改。扩充参数只调用原`sample_color`与`merge_color_sample`；后者仍最多保留8个代表样本，不能盲目放宽容差。没有换YOLO，没有新增灰参考/白平衡/颜色补偿。前轮灰参考候选已从本PR当前差异撤下，历史提交保留，不改写Git历史。

B最初从GitHub克隆时未收到原图。用户随后提供了旧交接包位置，实际找到在`C:/Users/pjb11/Desktop/协同/Computer_B_Handoff`。现已只读核验115个包文件及108个数据文件，全部SHA一致；包括56张历史图像（55张去重）与46份工作台元数据。无需A重采或重发这一批。尚无完整框真值、独立会话拆分、姿态绑定及实测角度/高度，所以严格完整覆盖仍不能登记，不能宣称六色现场样本已补齐或参数已优化。

本轮不连接Pi/相机/串口，不启动或重载服务、不部署/烧录，不改机械姿态或现场许可。相机安装与对焦保持；新角度/高度只能由A在已获许可的场景中采集并标注，不让B远程驱动机构。

## A现在先做什么（已有交接包已收到，不用重新摆场地）

把下面这一段交给A电脑上的Codex即可：

> B已收到原Computer_B_Handoff，115个包文件/108个数据文件SHA均通过，56张历史图已回放，不用重新导出这一批。请沿用原Web和当前视觉方案，先只读核对是否有包外新增样本，再核对B列出的5个未检出样本：黑色_稍转角_19、红色_反光_12、黄色_反光_10、蓝色_反光_08、绿色_反光_07。请补相机身份/配置、观察姿态、角度/高度实测值、同会话分组及画面内物料框人工真值；未知项保留未知，不从名称猜。确需补拍时仍通过原网站采集→冻结→保存同帧图与记录，集中安排，不要求用户重做全部机械调试。仅对新增资料运行B的只读导出器，检查数量/错误后交付，不部署/合并、不重启服务、不访问设备控制或调整相机参数，不传凭据。

A有包外增量时，可在原项目根目录使用独立只读审查工作树取得脚本；已有同名审查目录时选新名称，不能覆盖：

```powershell
git fetch https://github.com/12412512/cybertruck.git codex/vision-mission
git worktree add --detach ..\cybertruck-b-review FETCH_HEAD
python ..\cybertruck-b-review\tools\export_material_samples.py --project . --output output\B_material_export_20261030
```

导出器只用Python标准库，不需要安装OpenCV或启动服务。`--project .`必须是A原项目根目录，不是新审查工作树；输出会生成同名目录与`.zip`。请先看`export_report.json`：EXPORTED_NEEDS_ANNOTATION表示文件导出，不表示标注齐全；EMPTY表示没有图像；错误逐项保留。每张图先按原保存JSON与图像SHA核对，再复制。源文件不修改；重复图像只导出一次。ZIP由A检查后交付，B不自动上传原图到公开仓库。

目前支持两类已知目录。原交接包含脱敏记录，原记录内SHA可能保留脱敏前值，应该由交接包外层manifest校验；请用`replay_material_handoff.py`核验，不能重算内层SHA后冒充原始记录。增量导出器面向A原工作台保存记录。若原图实际在旧工程Docs或其它目录，由A列出路径并补原清单；不自动扫描无关文件夹。只有零散JPG、没有保存SHA的资料也不要补造“原始SHA”；另列为待核对来源。

## 包内还需要人工补哪些字段

导出包的`manifest.json`保存原图SHA、来源与少量白名单信息。没有的数据保留null，不能从文件名或旧候选自动填真值。已有`operator_label/color_id_hint`只是提示；须人工核对画面内全部目标。

| 字段 | A的填写方式 |
| --- | --- |
| `source_kind` | physical_capture为已保存实拍，historical_capture为旧现场图，synthetic_fixture为合成；不能混淆 |
| `capture_session` | 同一次采集会话统一代号，不用真实控制令牌；相邻帧/增强副本归同一会话 |
| `split` | calibration、validation、test；同会话不能跨组，验证/测试组不用于采样参数 |
| `context` | 实际相机身份、分辨率、相机配置SHA、观察姿态ID/修订/SHA、目标平面和view |
| `conditions.lighting` | normal/dim/bright/warm/cool/shadow/glare |
| `conditions.angle_bin` | front/left/right；记录实际观察方向，分组定义由A固定 |
| `conditions.height_bin` | low/nominal/high；A按实际工作范围定义，不能把示例数值当机械目标 |
| `view_angle_deg` | 光轴相对目标平面法线的实际角度；左右可用正负；0为正视目标平面 |
| `camera_height_mm` | 镜头到所选目标平面的实测高度，不是车高、台高或像素比例推算 |
| `annotations` | 每个物料的真实color_id及`bbox_px=[x,y,w,h]`，原图像素；只有确认整幅无物料才填[]，未标注保持null |
| `sample_roi` | 仅calibration组需要，`[x1,y1,x2,y2]`框选物料内均匀表面，避开背景、边缘和高光；验证/测试不参与参数拟合 |

颜色编号沿用1红、2黄、3蓝、4绿、5黑、6浅蓝。未知角度/高度或姿态配置不能凭印象补齐；可以先交原图包，缺项会被审计列出，不计完整覆盖。

导出时的`material_profile_at_export.json`只代表导出时配置。须核对其SHA与记录里的`observation.profile_sha256`来源，不能自动认为历史图使用同一配置。原基线样本会话信息不完整时，报告明确披露，不能宣称测试集与历史基线完全独立。

## 不足部分如何集中采集

先复用仓库`vision_preview_20261028.md`的最小清单：每色3张（正常、轻侧转、阴影/反光），共18张，加3张空背景。利用原Web保存同帧图与记录，不需要逐张发聊天；此阶段不是抓放成功验收。

补齐优先条件时，固定其它因素，每色分别采：7种光照（正视、标称高度），正常光下左/右斜视，正常光正视下低/高观察高度，共11条件×6色=66个正例条件；另有7种光照的负例。每条件准备calibration/validation/test各一张，来自独立采集会话，优先包共198张正例＋21张负例。不是要求一次立刻采219张，可先交已有包，B按缺口集中补单。不要将连续视频三帧分别充作三组。

负例保留夹爪、线缆、黑色印刷/环线、反光及相邻物料等干扰。混色、遮挡、边缘截断也保存并如实标注；不能只留成功图。完成单因素后再补光线×角度×高度交叉条件（工具`--full-matrix`列378正例条件）。高度/角度须在A的实际工作范围内，B不提供未经验收的机构动作值。

## 已完成的工具与约束

- `tools/export_material_samples.py`：只读打包，校验原保存记录和图像；白名单导出，不复制运行凭据。
- `tools/replay_material_handoff.py`：核对原交接包的package/data清单（含脱敏后的SHA），逐图调用现有识别器；不克隆旧bundle、不恢复运行时、不改原包。
- `tools/material_dataset.py plan/audit/compare`：生成计划、审计SHA/路径/标注/实测条件、统计真实覆盖、输出同算法参数扩充前后对照。重复图像、同会话跨拆分、原图/增强图跨拆分均报错。
- `pi/robot/vision/material_dataset.py`：固定调用原MaterialDetector，采样只用calibration组；validation/test分别输出每色精确率/召回、7×7混淆矩阵（0为背景/漏检）、光线/角度/高度分组及逐图结果。按IoU≥0.5一对一匹配，错颜色同时计漏检与误检，重复框额外计误检。黑色形状、边缘、质量门槛在两端相同。
- 参数草稿写入独立输出，不写`config/vision/material_profile.json`，不自动应用。不同相机/配置、不同来源域的数据分别建包。灰参考校色不参与本工具或对照。

空数据报告EMPTY/缺失单元，不产0/0“准确率”；任何审计错误都阻止对照，不丢弃坏样本后报通过。真实/历史/合成分开统计，合成永不填补真实覆盖。所有回放保持`physical_acceptance=false`，即使样本来自实拍，也不等于当前设备或抓放验收。

## 本次结果（旧资料已收到，完整条件与真值仍缺）

初始0张的本机盘点记录保留在忽略目录；收到包后已完成真实历史回放：

- 115个包文件及108个数据文件校验通过，56张图全部解码，55张唯一图像。
- 46份保存记录的profile SHA均与现有识别器配置一致，但完整观察姿态绑定0份；保存时快照不等于曝光同步状态。
- 24份按名称标为六色×正常光/稍转角/阴影/反光的记录，每色4份。名称只作为条件线索，不据此虚构实测角度、高度或独立会话。
- 颜色选择提示在有效候选中出现：红3/4、黄3/4、蓝3/4、绿3/4、黑3/4、浅蓝4/4。该19/24是提示一致性统计，不是基于完整标注的准确率，也不能统计全部误检。
- 需优先复核5张：黑色稍转角；红、黄、蓝、绿反光。没有修改阈值强行将它们算作通过。
- 9张旧测试图的六色/圆环18组，与包内原回放归档的items逐字段完全一致；当前OpenCV5.0.0/NumPy2.4.6，与归档环境不同仍保留版本说明。
- 原`test_vision_migration.py`现在支持测试专用环境变量`ROBOT_VISION_SAMPLE_ROOT`，显式读取外置旧图目录，不复制运行数据；9项全部通过。默认路径缺图仍报错，不自动skip。

没有基于缺失姿态/真值/会话的历史图生成参数提升结论；完整真实覆盖仍待补元数据和独立标注。

为验证数据流程，另生成219张明确标为synthetic_fixture的图（66正例条件＋7负例条件，各3组）。光线为图像颜色数值，角度/高度为渲染参数；不是实测角度/高度，不是独立现场泛化验证。夹具使用单独profile，不改仓库已有历史参数。两端算法代码完全相同，只扩充颜色样本：

| 保留组 | 正例/负例 | 基线TP/FP/FN | 扩充样本后TP/FP/FN |
| --- | --- | --- | --- |
| validation | 66/7 | 45/0/21 | 66/0/0 |
| test | 66/7 | 45/0/21 | 66/0/0 |

test分组基线：normal 30/30、dim 1/6、bright 1/6、warm 3/6、cool 4/6、shadow 0/6、glare 6/6；扩充后夹具各组全识别。这只证明原采样/合并和对照流水线可工作，简单合成数据不证明真实复杂阴影、反光和高度变化已解决。真实准确率、Pi吞吐与250ms全链路仍需A的数据和后续验收。

## B收到包后的复现

先解压到新目录并保留原包；完成明确的人工标注后执行：

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -B tools/material_dataset.py plan --output output/material-review/capture-plan.json
.\.venv\Scripts\python.exe -B tools/material_dataset.py audit --manifest output/A-material-pack/manifest.json --output output/material-review/audit.json
.\.venv\Scripts\python.exe -B tools/material_dataset.py compare --manifest output/A-material-pack/manifest.json --profile config/vision/material_profile.json --output output/material-review/comparison.json
```

输出必须选新名称，不覆盖旧证据。compare需三组存在、标定组覆盖六色、无泄漏与元数据错误；否则明确拒绝。相机设置与原profile来源不符时应先由A确认基线，不强行套用。

流程夹具可独立复现：`python -B tools/replay_material_dataset.py --output output/material-review/synthetic-fixture`。测试入口为`python -B -m unittest discover -s tests -p test_material_dataset.py -v`。

原包回放与测试：

```powershell
.\.venv\Scripts\python.exe -B tools/replay_material_handoff.py --package C:\Users\pjb11\Desktop\协同\Computer_B_Handoff --output output/material-review/handoff-replay.json
$env:ROBOT_VISION_SAMPLE_ROOT = 'C:\Users\pjb11\Desktop\协同\Computer_B_Handoff\data\output\vision'
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_vision_migration.py -v
```

本轮15项样本工具测试、52项任务/证据测试、33项视觉工作区、3项数字模板及9项迁移回归，共112项通过；1,146项工程静态检查通过。未编译/烧录或访问设备。

用户提供的A共享对话《分析ros小车功能与主控模式》中也记载：原网站为统一入口；以已验证旧六色/圆环/连续观测为基线，另在Pi的`/home/pi/Camera_Web_Preview/666`存在NCNN参考方案、`/home/pi/shu`有ONNX参考框架。历史记录不证明这些目录当前在线或已部署。该链接说明有108个数据文件的旧包，不是108张照片；本轮现已读到原包。后续仍不替换算法，若将来对比旧模型须另行按同一批资料审查。

交给A审查的当前内容是原算法的样本工具、真实历史兼容回放、隔离验证、剩余条件缺口及合成参数对照。真实条件样本的“补齐”和现场参数优化仍等待A补标注/角度高度/增量，不能登记为已完成。
