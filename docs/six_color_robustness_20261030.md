# 六色识别核对与离线鲁棒性候选

2026-10-30，B端，基于首日离线交付继续。首日交付已通过Fork上传至[PR #1](https://github.com/risiairline-dotcom/cybertruck/pull/1)，本候选作为后续独立提交供A分别审查。本模块未接入相机服务、Web或实车任务；由A评审后决定是否接入。所有数字为本机离线结果。

## 当前源代码是否为YOLO

当前GitHub仓库内的接入链路为`camera.py → observe.Detector → legacy.material_vision.core.MaterialDetector`；`FrameEngine`的material_local模式也实例化同一个MaterialDetector。该实现用LAB样本归一化距离、HSV门限、连通轮廓和形状参考，未加载神经网络。

对仓库源码和模型扩展名搜索，未找到YOLO/Ultralytics/Torch/ONNX推理入口或`.pt/.pth/.onnx/.weights/.tflite`权重。`web/static/workbench-vision.js`明确显示“当前使用旧六色/圆环算法；NCNN模型尚未接入本页”，`docs/web_vision_workspace.md`也记录NCNN比较未接入。NCNN是一种推理部署框架，仅有这个名称不能确认网络结构是YOLO。

用户可能在旧工程或A端模型目录中已有YOLO/NCNN版本；仓库内容不足以否定它的存在。B没有连接Pi寻找模型。需要原权重、推理入口、类别顺序/数据配置、训练数据与部署格式后，才能确认并在原模型上继续训练。本次不声称训练、替换或优化了YOLO模型。

## 开源方案选择依据

星数于2026-10-30通过GitHub API核对，仅反映受关注程度，不作为现场效果指标。

| 项目 | 当次星数/状态 | 本次采用的思路 |
| --- | --- | --- |
| [OpenCV](https://github.com/opencv/opencv) | 91,374，Apache-2.0 | 保留现有LAB与轮廓算法，用显式灰参考做受限通道补偿，避免盲目全图白平衡 |
| [OpenCV contrib](https://github.com/opencv/opencv_contrib) | 10,224，Apache-2.0 | 参考[GrayworldWB](https://docs.opencv.org/4.13.0/d7/d71/classcv_1_1xphoto_1_1GrayworldWB.html)的适用假设与[Color Correction Model](https://docs.opencv.org/4.13.0/d1/dc1/tutorial_ccm_color_correction_model.html)的已知参考色思路；未引入contrib依赖或声称实现完整CCM |
| [Albumentations原仓库](https://github.com/albumentations-team/albumentations) | 15,306，MIT，已归档 | 借鉴亮度/色偏、模糊、旋转、尺度、透视的可复现分组测试；测试变换由NumPy/OpenCV实现，未增加运行时增强库 |
| [Ultralytics](https://github.com/ultralytics/ultralytics) | 62,975，AGPL-3.0 | 若原YOLO模型到位，作为原模型训练/导出方案候选；本轮未下载权重或引入检测网络 |

灰世界假设在一画面以红/蓝物料为主时可能不成立。当前候选要求人工标注、与同一观察姿态绑定的灰块，不使用全图均值。灰块基准与颜色样本应在同一成像设置下采集；不同观察位分别建立参考。

## 已实现的可选候选

`pi/robot/vision/material_robust.py`提供：

- `make_reference(frame, roi, context, source_sha256)`：保存灰参考的BGR中位数、来源SHA、相机配置、分辨率、姿态版本/SHA、目标平面与视图。基准必须近中性、均匀且未过暗/过曝。
- `RobustMaterialDetector(profile, reference).process(frame, context)`：同帧参考块内区估计各通道增益，限定增益0.5～2.0、通道间增益比不超过1.65，再调用原MaterialDetector。内区采样避免轻度模糊时背景混入灰块边缘；若内区仍有纹理/遮挡、参考过暗/过曝、上下文变化或色偏越界，返回未知诊断，不能当空槽。
- `add_labelled_color(...)`：复用现有sample_color/merge_color_sample保存有来源SHA和观察上下文的多视角样本，最多保留原算法允许的8个代表例，不修改共享配置。当前六色编号1红、2黄、3蓝、4绿、5黑、6浅蓝保持不变。

低质量、边缘截断、未标定颜色保持不确定；黑色还必须通过外形参考，不能把黑色印刷线当物料。蓝/浅蓝连通混色仍拒绝强制分类。所有输出`confirmation_allowed=false`、`physical_acceptance=false`，像素位置不转换为毫米或夹持中心。

这不是自动万能校色：同灰块不同位置的局部阴影、非均匀照明、镜面反光、强曝光截断和非线性相机处理可能无法恢复颜色。均匀灰色遮挡物也可能通过纯图像灰块检查，真实参考身份和遮挡仍需A的场景约束。本轮没有扩大旧配置容差，也没有把模糊或错误候选降级成成功。

## 对照方法与结果

`tools/benchmark_six_colors.py`使用独立合成profile：六色调色板分别采样，固定灰块和截锥轮廓。对照两端共享完全相同的颜色和形状配置；不是直接评估仓库中的历史现场profile，也没有把夹具标定写入真实配置。

矩阵为6色×8类成像扰动×3种像素尺度×3组旋转/透视，合计432例。预测需颜色正确且与标注框IoU≥0.5，额外候选计误检。合成无纹理、单目标场景比现场容易，不估计实物准确率。

| 分组 | 例数 | 原算法正确 | 灰参考候选正确 |
| --- | --- | --- | --- |
| 正常 | 54 | 54 | 54 |
| 整体变暗×0.65 | 54 | 0 | 54 |
| 整体变亮×1.25 | 54 | 9 | 54 |
| 暖色偏 | 54 | 27 | 54 |
| 冷色偏 | 54 | 18 | 54 |
| Gamma 1.25 | 54 | 27 | 36 |
| 仅物料局部阴影×0.55 | 54 | 0 | 0 |
| 7×7轻度模糊 | 54 | 54 | 54 |
| 合计 | 432 | 189 | 360 |

矩阵两端误检均0，但不包含真实背景的丰富负样本。局部阴影54例全部漏检，Gamma仍有18例漏检，这是未解决项，不能从总分推断“各种光线均已解决”。首次候选曾因灰块边缘模糊导致54例拒识；改用内区并增加模糊回归后恢复，该退化及修复在本说明中记录，未修改真实链路。

本机中位耗时约原算法16.4ms、候选31.2ms，只是单次纯图像处理，不包含采集、编码、HTTP或时序确认，也不是树莓派FPS测试。尺度0.65/1/1.35和透视只是像素占比/形状代理，不是实物高度标定。

## 测试和复现

新增`test_material_robust.py`共12项通过：六色在整体亮暗/暖冷色偏/轻模糊，旋转/透视/尺度，灰块过暗/过曝/遮挡，姿态/相机/平面错配，黑线/空白负例，黑色缺外形、蓝浅蓝混色、非法输入、来源与输入不变性。与原52项任务/证据、33项视觉工作区、3项数字分类器回归合计100项通过；1,146项工程静态检查通过。

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'pi'
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_material_robust.py -v
.\.venv\Scripts\python.exe -B tools/benchmark_six_colors.py --output output/six-color/benchmark.json
```

原task/vision/ring相关回归继续运行；原历史视觉回放中缺`output/vision/samples.json`的一项仍未恢复，见首日交付文档。本轮只读静态检查，不编译/烧录、不连接实车。

## 现场数据与YOLO后续

A先补交原有模型和样本。若存在YOLO，应核对模型文件SHA、YOLO版本、六色类别顺序、输入尺寸、预处理/输出解码、量化格式与Pi推理入口；对原模型做基线后再决定微调，不能仅凭NCNN名称替换推理器。

采样按“六色×光照×视角×实际相机高度/观察位”分层：正常/弱光/强光/暖冷色温/侧影/反光；正视与左右斜视；覆盖实际低/中/高观察姿态；每类同时包含成功与失败、遮挡/相邻物料/黑印刷/蓝浅蓝混淆等负样本。具体角度和高度由A测量记录，不从像素尺度反推。

训练、验证、测试按采集会话分组，增强图必须与原图在同一组，避免相邻视频帧泄漏。增强亮度/模糊/透视不改变标签；色相增强必须受限并人工复核，不能把蓝色增强成浅蓝还保留原标签。每色分别统计精确率/召回、混淆矩阵、不同光线/角度/高度的漏检和误检；Pi再测完整链路P50/P95延迟和反馈时效。

若YOLO只负责找物料框，可比较“YOLO定位+框内六色判别”与原六类别模型，保留UNKNOWN、帧新鲜度和任务证据门禁。没有真实数据与原权重时，不承诺训练效果；当前候选可作为离线对照或前处理研究，是否用于原YOLO仍需独立评估。
