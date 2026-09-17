# FoundationPose：环境、胶带重建与离线诊断

## D435i 实时跟踪接入

`live_tracker.py` 是官方 FoundationPose 的相机和网页适配层：直接调用运行时 `FoundationPose.register(iteration=5)` 和 `track_one(iteration=2)`，不修改官方位姿网络、算法源码或上游快照。SAM ViT-B 负责首帧像素分割，提示可来自手动框选或自动水瓶检测。没有机械臂控制、TF 发布或抓取接口。

```bash
source tools/foundationpose/runtime_env.sh
# RealSense SDK 独立放在 work/，不替换已有 Python 环境依赖。
python -m pip install --no-deps --target work/foundationpose_realsense_py311 pyrealsense2==2.58.4.10922
python tools/foundationpose/live_tracker.py --mesh work/object_models/ob_yibao_350ml/model/model.obj --auto-start
```

页面默认仅监听 MSI 的 `http://127.0.0.1:8089`；远端浏览器可用 `ssh -N -L 18889:127.0.0.1:8089 msi`，然后打开本机 `http://127.0.0.1:18889`。选择目标会冻结一张 RGB-D 图；拖框后初始化，出现跟踪结果后可缓慢移动目标。框坐标绑定冻结帧，旧选择会被拒绝。默认丢失后会自动重新检测和初始化，手动重新框选也可恢复。暂停仅停跟踪，停止按钮或 SIGTERM 会释放相机并退出服务。

用户需要直接通过局域网访问时，指定 MSI 的局域网地址（无需 SSH 隧道）：

```bash
python tools/foundationpose/live_tracker.py --host 192.168.101.65 --port 8089 --auto-start
```

此时使用 `http://192.168.101.65:8089/`。服务只绑定指定网卡地址；该地址变化时需相应调整启动参数。

可优先点击“自动识别并跟踪”，或启动时传入 `--auto-start`：Torchvision 官方 SSDLite320 MobileNetV3 COCO 权重检测 bottle 类别，检测框交给 SAM 生成像素掩码，再调用官方 FoundationPose。权重缓存于 `work/foundationpose_setup/torch_hub`。这是外部初始化适配器，不是 FoundationPose 自带物体检测；不识别品牌。只自动选择明显占主导的前景瓶子，多个大小接近的候选需要点击目标；没有可靠候选时提示重试，保留手动框选。默认丢失后进入 recovering 状态，立即使用最新帧自动重新检测和初始化（默认恢复等待 0 秒）；无目标时继续等待，多个相近候选仍点选确认。自动检测不核验品牌，画面中应保留目标水瓶。暂停、手动选框和多候选选择期间不会自动恢复；可用 `--no-auto-recover` 禁用，`--recovery-interval` 可设置非负的额外等待秒数；不会并行启动多个恢复任务，也不会反复处理同一张失败帧。

预览使用 `/stream.mjpg` 连续传输最新结果，JPEG 质量为 75、限制发送缓存，替代原 400 ms 单张图片轮询。采集主动排空旧帧，使用可映射到系统时钟的设备时间戳记录帧年龄；CPU 推理线程数限制为 2、OpenCV 为 1，避免过度并行。页面的“服务端结果年龄”不包含浏览器显示和网络传输时间。

默认读取 D435i 彩色 1280×720/15 fps、深度 640×360/15 fps，对齐到彩色后缩至 640×360，内参同步缩放、深度按设备比例转米。相机线程持续取帧，正常跟踪只取最新帧；最多保留 90 帧短时缓存供恢复追帧，不无限积压视频；输出记录相机原生帧号。启动通过现有预览的 pause API 释放相机，不执行设备重置。

`GET /api/status` 给出状态和相机光学坐标系下的 `camera_T_object`（米）。过期、深度不足或不可靠时位姿为 null。深度残差/像素数/步间位移等只是启发式检查，不能证明透明物体跟踪准确；`pose_accepted_for_robot_control` 始终为 false。网页保留与估计对应的画面，过期时明确标示。仅同源浏览器可以发送控制 POST。

每次运行记录在 `work/foundationpose_live/<时间>/`：模型哈希、实际流配置、初始化 RGB-D/mask、连续帧位姿与质量指标、最新预览。怡宝模型高度 170 mm、直径 60 mm 是待实测的假设，近似旋转对称与透明瓶身会影响深度和朝向判断。

离线检查：`python -m unittest discover -s tools/foundationpose -p 'test_live_*.py' -v`。相机与实物跟踪的实际结果按每次运行单独记录。

2026-09-17 实测：4 项输入/深度/跳变/多目标候选检查通过。350 mL 怡宝实物完成连续视频跟踪，用户确认缓慢移动时框贴合；过程中曾触发丢失，重新初始化可恢复。自动初始化已在真实相机帧完成，未手动提供初始框。低延迟版本运行记录 `work/foundationpose_live/20260917_112942_816874/`；抽样最近 200 帧实际处理约 15 fps，中位推理约 34 ms、服务端结果年龄约 118 ms。经 SSH 隧道连续解码 20 个 MJPEG 帧约 15 fps，平均约 38 KB/帧。结果年龄不含浏览器显示延迟，局部深度残差不等于定位精度，未验证实机抓取。

## 最新：set3 重建结果（2026-09-15）

`tape_20260915_set3` 的 21 组候选、147 个原始文件哈希已检查。使用独立提示配置 `config/foundationpose/tape_20260915_set3_prompts.json`，修正了人手、桌面阴影和纸芯掩码；最终分割为 `work/object_scan/tape_20260915_set3/segmentation_final/`。最大连通区域清理去除了 1719 个孤立背景像素，原始采集未修改。

18 组拟合、3 组留出，参考视角使用目标深度较多的 `140609_a4cda5d8`，避免首帧少量异常深度作为参考。完整训练和纹理生成正常退出，结果在 `work/foundationpose_runs/tape_20260915_set3/nerf_v1/`；几何 6261 顶点、12486 面，外形约 106.5×91.5×59.1 mm。

**结果不合格**：模型明显分裂，未形成可靠完整胶带。原始 register 的三组留出 IoU 为 0.188、0.171、0.166；没有根据这些结果调整算法或重新拟合。不用于实时跟踪、抓取或机器人控制。组数增加没有解决胶带本体深度缺失，后续应先诊断采集质量。

```bash
source tools/foundationpose/runtime_env.sh
python tools/foundationpose/reconstruct_scan.py \
  --session tape_20260915_set3 \
  --segmentation work/object_scan/tape_20260915_set3/segmentation_final \
  --reference-view 140609_a4cda5d8 \
  --heldout 140501_d6fad679 141003_6769cd1e 141813_565e0bbd \
  --steps 1000 --texture --include-rgb-only-rays
```

重建默认不再套用旧会话的留出 ID；须显式传入本会话的 IDs。会话错配、不存在的留出 ID 和非拟合参考视角会拒绝处理；前两种拒绝路径已测试。

## 首组数据与环境结果（2026-09-15）

- MSI 独立 GPU 环境和 C++/CUDA 扩展已部署，模型权重可加载。GPU 矩阵、PyTorch3D KNN、NVDiffRast 光栅化、Kaolin 八叉树、编码器前后向、八叉树光线追踪、Warp 深度处理及 EGL 渲染均已运行。
- 胶带 13 个视角完成 SAM 分割与目视检查。11 组用于官方神经重建并生成纹理草模，另 2 组用于离线 register 和同帧 track_one 检查。
- **草模尚未验收**：存在多余孔洞和起伏。原始初始化轮廓 IoU 为 0.949 / 0.382；按观测掩码检查前 20 个候选后为 0.955 / 0.418，第二个视角仍不可靠。
- 两组检查图已经用于失败分析和候选策略开发，不再是独立验收集。连续实时跟踪、绝对定位精度、机器人 TF 发布及实机抓取均未验证。

## 启动与检查

在 MSI 项目根目录运行：

```bash
cd /home/pc/saydo
source tools/foundationpose/runtime_env.sh
python tools/foundationpose/check_source.py
python tools/foundationpose/check_gpu.py
```

独立环境为 `work/foundationpose_env/`，运行副本为 `work/foundationpose_runtime/`，日志和包清单在 `work/foundationpose_setup/`。采集仍使用原 `.venv`。源码检查只验证上游文件；GPU 检查才执行运算和权重加载。

已验证：Python 3.11、PyTorch 2.5.1+cu124、torchvision 0.20.1+cu124、CUDA Toolkit 12.4.131、PyTorch3D 0.7.9、NVDiffRast 0.3.3、Kaolin 0.18.0、Warp 1.6.2、NumPy 2.1.3、OpenCV 4.11.0.86、Open3D 0.19.0。显卡 RTX 4060 Laptop 8 GB，驱动 595.84。nvidia-smi 的 CUDA 13.2 是驱动能力，不是 nvcc 版本。

版本及验证记录：`config/foundationpose/environment_state.json`。环境声明和 Python 约束在本目录 `environment_msi.yml`、`constraints_msi.txt`。

Warp 1.6.2 在此驱动上有设备 UUID API 兼容警告，但深度 CUDA 核函数、重建和离线推理实际执行通过。它与下述已修复的退出异常不同。

## 固定源码和兼容补丁

官方原样快照：`third_party/FoundationPose/`，固定提交 `a1b694b83e633c2cb6115b9063d940a687759392`。来源及 54 个文件 SHA-256 在 `third_party/FoundationPose.upstream.json`，许可证和版权声明原样保留。

`prepare_runtime.py` 从清单复制原文件到独立运行目录，并记录兼容修改：

1. CUDA 扩展使用 C++17 和正确的 cxx 参数键。
2. 三处 tensor.type() 分派改用 scalar_type()。
3. 统一使用 `bundlesdf.mycuda` 包路径。上游以两种名字加载同一个 common 扩展，曾导致保存模型后退出时 SIGABRT；GDB 指向 `pybind11_module_def_common` 的重复释放。修复后，含 ray_trace 的检查和完整重建均正常退出。

运行目录的 `SAYDO_RUNTIME_PATCHES.json` 记录修改。准备脚本拒绝覆盖已有运行目录，避免丢失编译结果和现场修改。

其他源码固定版本：

- PyTorch3D：`33824be3cbc87a7dd1db0f6a9a9de9ac81b2d0ba`（v0.7.9），包含安装所需的 `projects/implicitron_trainer/`；本次取得的 405 个所需文件按 Git blob 哈希核对。
- NVDiffRast：`729261dc64c4241ea36efda84fbf532cc8b425b8`（v0.3.3）。
- Segment Anything：`dca509fe793f601edb92606367a655c15ac00fdf`。

它们位于 `work/foundationpose_deps/{pytorch3d-source,nvdiffrast-source,sam-source}/`。源码及环境备齐后，运行 `bash tools/foundationpose/install_extensions.sh` 构建并检查。系统还安装了 libegl-dev 和 libgl-dev。

基础 Python 包安装命令：

```bash
python -m pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu124
python -m pip install -r third_party/FoundationPose/requirements.txt -c tools/foundationpose/constraints_msi.txt xatlas rtree einops
```

## 权重来源

权重在 `third_party/FoundationPose/weights/`，运行副本通过符号链接使用。精修/评分目录为 `2023-10-28-18-33-37`、`2024-01-11-20-02-45`，各含 config.yml 与 model_best.pth。

官方 Google Drive 的模型和评分配置遇到配额限制。本次使用公开镜像 `https://huggingface.co/gpue/foundationpose-weights`，固定 revision `42d49e0633d245b3cf4dea6b1e7ec2b31d5b7654`。精修配置与取得的官方文件逐字节一致；模型没有官方哈希可对照，不声称已核实与官方原件一致。两组权重实际加载及推理通过，来源、尺寸和 SHA-256 在 `SAYDO_WEIGHTS_SOURCE.json` 及环境快照中记录。

`runtime_env.sh` 启用 `TORCH_FORCE_WEIGHTS_ONLY_LOAD=1`。SAM ViT-B 权重来自 Meta 官方地址，位于 `work/foundationpose_setup/sam_vit_b_01ec64.pth`，哈希记录在分割结果中。权重均保留本地。

## 数据和运行命令

原始采集 `work/object_scan/tape_20260915/` 未修改。提示和图像哈希为 `config/foundationpose/tape_20260915_prompts.json`。最终分割为 `work/object_scan/tape_20260915/segmentation_sam_v3/`，segmentation.json 已记录目视审阅，区分内孔背景与纸芯内壁。

```bash
# 新建分割输出，不覆盖已有目录
python tools/foundationpose/segment_views.py

# 重建：默认新建运行目录
python tools/foundationpose/reconstruct_scan.py \
  --segmentation work/object_scan/tape_20260915/segmentation_sam_v3 \
  --heldout 102616_65d32a20 102905_faba178f \
  --steps 1000 --texture --include-rgb-only-rays
```

完整运行结果：`work/foundationpose_runs/tape_20260915/nerf_v3_rgb/`。模型为 model.obj、material.mtl、material_0.png，原始几何为 model_geometry.obj。训练、提取、纹理生成和正常退出通过。几何约 16114 顶点、32252 面，包围盒约 109.6×107.8×67.4 mm；这不是实测尺寸。

物体坐标轴平行基座，原点是第一视角有效表面的稳健中值，不是抓取中心。run.json 保存参考变换及视角名单。图像缩放到 640×360 时同步缩放 K；无效深度保持无效，没有用圆环原始几何替代神经重建。旧 nerf_v1、nerf_v2_geometry 保留了退出异常记录，不视为完整运行通过。

```bash
python tools/foundationpose/evaluate_mesh.py \
  --run work/foundationpose_runs/tape_20260915/nerf_v3_rgb \
  --segmentation work/object_scan/tape_20260915/segmentation_sam_v3
```

已有 heldout_evaluation 目录不覆盖。追加 `--rerank-topk 20` 创建 development_rerank_top20；该选项用于候选诊断，不是机器人动作准入逻辑。已有两次检查结果均保留。

原始/候选重排后的 IoU 分别为 0.949→0.955、0.382→0.418。track_one 只重复同一静帧，不代表运动跟踪成功；局部深度残差不能当作整体定位精度。报告均不批准机器人控制。下一步需要改善胶带侧壁深度和模型质量，再采未参与开发的新视角验收。

官方依据：
- https://github.com/NVlabs/FoundationPose
- https://kaolin.readthedocs.io/en/latest/notes/installation.html
- https://github.com/facebookresearch/segment-anything

设备隔离：上游 refiner 的 predict 会设置全局 CUDA 默认张量类型，Torchvision SSD 的内部锚框缓存仍在 CPU。`live_detection.predict_detection` 在局部 CPU factory 上下文调用检测器，图像与网络仍在 CUDA，退出后恢复调用方默认设备，避免首次跟踪后重新识别出现 CPU/CUDA 拼接错误。回归检查使用实际 Torchvision DefaultBoxGenerator 复现该错误，并验证连续三次修复调用及默认设备恢复。

2026-09-17 重新初始化修复验证：7 项离线检查通过（含实际 CUDA/CPU 锚框冲突复现、设备上下文恢复、恢复冷却及暂停取消）。在已经执行过 register/track_one 的真实服务进程中，连续两次调用自动识别均成功回到 tracking，无设备冲突。当前恢复版日志为 `work/foundationpose_live/server_recovery.log`。现场移出/移回引发的自动恢复另待用户确认，不以状态机单测代替实物结果。

恢复期间预览保持实时：只有手动框选和多候选点选会冻结显示；自动检测、初始化及 catching_up 显示最新相机画面且不叠加旧姿态框。初始化仍使用一致的 RGB-D 快照。register 完成后，使用官方 track_one 沿短时缓存的中间帧追到距当前约 250 ms 内，再发布框；追帧预算 3 秒，缺少连续历史或质量不通过则放弃旧结果重新检测。视频流的 X-Preview-State/X-Preview-Sequence 标头可用于核验恢复期间仍在传新画面。

恢复预览修复实测：11 项回归检查通过。2026-09-17 同一现场重新初始化用时 3.764 s，其间 MJPEG 收到 57 个不同序号的画面；追帧用时 1.471 s、推进 39 个中间观测，其间又收到 23 个不同画面。恢复显示跟踪框时结果年龄 0.173 s。计算耗时依然存在，但自动恢复不再冻结预览。记录在 `work/foundationpose_live/20260917_124623_822927`，日志 `server_live_preview.log`。

可靠跟踪结果叠加官方 `draw_xyz_axis` 绘制的物体坐标轴：X 红、Y 绿、Z 蓝，轴长 0.05 m，并标注轴名。使用原始 `camera_T_object`，不是定向包围盒坐标系；怡宝模型原点位于包围盒中心、+Z 沿瓶身向上。恢复及质量不可靠时隐藏坐标轴。

## 网页切换物品

点击“切换物品”，列表实时读取 `work/object_models` 的直接子目录，隐藏目录与压缩包不参与。默认选择 `model/model.obj`、`model.obj`，也支持对应 GLB；不自动选择毫米版 `model_mm.obj`。缺少模型的目录显示原因并禁用。遥控器目录已从已有重建结果补入模型，原采集文件保留。

切换会取消旧初始化/追帧并清除旧位姿与覆盖图；GPU 推理线程使用官方 FoundationPose 构造新物体实例，复用原 scorer/refiner 网络及渲染上下文。新模型加载完成后页面更新名称、米制包围盒尺寸和自动检测类别。使用 `--auto-start` 启动时，支持检测的模型加载后自动初始化；其他模型手动框选。模型切换写入本次运行目录的 `model_changes.jsonl`，每条姿态记录包含当前 object_id。

每个物品可添加 `tracking.json`：

```json
{"name":"马克杯（iPhone）","mesh":"model/model.obj","units":"metres","detector_class":"cup"}
```

`detector_class` 使用 Torchvision COCO 类别名称，例如 bottle、cup、remote；null 或不支持的类别采用手动框选，丢失后也不自动检测错误类别。已有水瓶、杯子、遥控器目录在无配置时可按目录名识别类别；新物品建议显式配置。`--models-root` 可指定列表根目录，`--detector-class` 仅覆盖命令行初始模型的类别。

接口：`GET /api/objects` 列出目录，`POST /api/object` 携带 `{"id":"目录名"}` 请求切换。只接受列表内、路径位于该物品目录的模型；不接受任意文件路径。
