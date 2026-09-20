# FoundationPose：环境、胶带重建与离线诊断

## D435i 实时跟踪接入

`live_tracker.py` 是官方 FoundationPose 的相机和网页适配层：直接调用运行时 `FoundationPose.register(iteration=5)` 和 `track_one(iteration=2)`，不修改官方位姿网络、算法源码或上游快照。SAM ViT-B 负责首帧像素分割，提示可来自手动框选或自动检测。受保护的试抓接口见本文“首轮视觉抓取接入”；默认点击抓取按钮并通过检查后执行。

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

默认读取 D435i 彩色 1280×720/15 fps、深度 640×360/15 fps，对齐到彩色后缩至 640×360，内参同步缩放、深度按设备比例转米。相机线程持续取帧，正常跟踪只取最新帧；最多保留 180 帧短时缓存供恢复追帧，不无限积压视频；输出记录相机原生帧号。启动通过现有预览的 pause API 释放相机，不执行设备重置。

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

恢复期间预览保持实时：只有手动框选和多候选点选会冻结显示；自动检测、初始化及 catching_up 显示最新相机画面且不叠加旧姿态框。初始化仍使用一致的 RGB-D 快照。register 完成后，使用官方 track_one 沿短时缓存的中间帧追到距当前约 250 ms 内，再发布框；追帧按实际单步耗时调整步长（目标 120–300 ms，实际选帧间隔不超过 300 ms），预算根据初始延迟分配为 3–12 秒；缺少连续历史或质量不通过则放弃旧结果重新检测。视频流的 X-Preview-State/X-Preview-Sequence 标头可用于核验恢复期间仍在传新画面。

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

## 视觉抓取接入（默认执行模式，用户已确认首次纸杯夹取成功）

同型号、同安装位置和朝向的替换相机可按用户确认复用手眼关系，配置见
`config/calibration/camera_reuse.yaml` 和 [标定说明](../../config/calibration/README.md)。
启动不带 `--serial` 时自动选择唯一 D435I，多台时要求指定序列号。
每台相机使用自身内参（像素到空间方向的换算参数），不再要求等于旧相机内参；
抓取和记录夹法仍核对实际型号、观测来源、成像配置和原标定来源。
原始标定与已记录夹法不因序列号变化被改写。复用依赖安装关系不变，不代表换机精度已验收。

网页提供“抓取当前被跟踪物品”“打开夹爪”“读取夹爪状态”“停止抓取”。“打开夹爪”通过 POST /api/gripper/open 固定发送 hand_follow_pos、hand_pos=[1000]，不改变力度或机械臂位置，可用于释放已夹持物品；抓取期间、预览模式、设备故障或夹爪正在运动时拒绝执行，并与其他抓取进程共用设备锁。页面显示命令接受结果，不将接受应答当作实际到位，实际位置通过“读取夹爪状态”查看；失败不自动重试，日志保存于 work/grasp/open_commands/。默认 `--grasp-mode execute`，点击抓取按钮后读取状态、生成航点并检查航点边界，通过后逐段交给控制器规划和执行。启动服务、自动初始化和视觉恢复不会触发抓取。只预览时显式指定 `--grasp-mode preview`。支持当前被可靠识别并跟踪的任意已加载物品，不再按物品编号限制。使用 `config/grasp/trial.json` 的 `grasp_profile` 和按物品保存的夹法：工具中心（TCP，Tool Center Point，即程序控制的夹爪参考点）的目标朝向随物品旋转，夹持位置可采用模型原点或记录的指定部位，随后竖直抬升。模型原点不一定是合适的夹持中心，仍需确认模型定义。用户已接受名义 grip_tcp，但这不代表几何精度已实测。

配置为 `config/grasp/trial.json`。变换链严格使用：

```text
base_T_object = base_T_grip_tcp @ inverse(flange_T_grip_tcp)
                @ flange_T_camera @ camera_T_object
```

点击时要求目标连续稳定至少半秒、位姿新鲜，并立即复制当前目标身份、相机坐标下的位置与朝向及图像时刻。任务开始时连续查询机械臂状态验证静止，将这份固定快照换算为机械臂基座下的抓取点；绑定时快照不得超过 0.6 秒。之后不再读取目标位置，视觉丢失、目标移动或规划耗时不会触发重新选点，也不会单独阻塞执行。页面视觉预览可以继续更新，但不参与本次抓取；请保持物品原位。任务日志保存锁定快照与最终固定抓取点。检查型号、固件、关节使能/错误、工具/负载与 Base 工作坐标系及标定来源哈希。首次运动使用 movej_p 直接到沿目标工具 +Z 反向退开的预抓点，同时调整到目标朝向；随后保持该朝向沿 +Z 接近夹持点；闭合并连续确认夹持反馈后，沿机械臂基座的竖直方向抬升 0.10 米（10 厘米）。配置为 action=grasp_and_lift、lift_height_m=0.10。已删除固定的预抓上方必经点及 clearance_height_m 配置；不会自动绕过障碍。开始前检查起始点及预抓、夹持、抬升航点的工作区与离桌间距，并对三个目标逐个做逆解与正解回验；每段发令前再按实际关节角重查该目标。不生成抓取中间点，也不声称检查过完整路径。空夹、故障或闭合超时不进入抬升；抬升期间不再轮询夹持状态。软件不提供中间路径的环境碰撞检查。

抓取与姿态调整共同使用 `motion_validation.py` 和离线工具 `grasp_ik.cpp`。使用 `bash tools/foundationpose/build_ik_checker.sh` 构建配置 `ik_checker` 对应的默认可执行文件；工具仅调用算法接口，不连接机械臂。新版输出 `PROTOCOL IK_FK_V1` 和每个逆解的正向位姿；Python 独立重算误差，缺少协议、结果不完整或误差超出 2 毫米/2° 均拒绝。

执行模式为默认值。夹爪力度设置和位置开合已由用户实测确认，不再以版本门槛、数值读回或旧的夹爪验收标记阻塞。采用写入 1220 力度上限、`hand_follow_pos` 按 open_position 张开 / closed_position 夹取的链路（前端独立“打开夹爪”按钮固定为 1000）；不写 1180，不使用 1470。设备状态、开始时的视觉时效和航点工作区检查继续执行。停止请求及异常超时处理保留，不自动松开或抬起；状态 3 仅表明力控停止，仍需观察是否抓牢。每次任务记录于 `work/grasp/jobs`。 再次抓取前以实际开度判断人工复位：夹爪停稳且位置距配置的抓取张开目标或完全张开位置 1000 不超过 3 个原始位置单位时，允许残留的力控停止状态；未张开且存在力控停止或非零力度反馈时仍拒绝自动张开。故障和运动状态继续拦截，任务日志保存初始夹爪状态以便排查。

2026-09-18 用户实测确认：原始力度 10、目标位置 0 可以轻夹纸杯，力度 50 会使纸杯轻微变形；目标位置 1000 为张开。适用于当前控制器 V1.7.0 / 夹爪 V1.154。配置记录 `force_control_verified=true` 和用户证据，默认力度 10。原始数值尚不能换算为牛顿，寄存器数值读回仍不可用；这不否定已确认的力度设置效果。完整范围见 [夹爪验证记录](GRIPPER_VERIFICATION.md)。

夹持反馈判定依据 RM Plus 状态 3（力控到位）、停止速度和尚未全闭的位置连续出现；正力反馈标记为 force_contact，零力反馈标记为 force_stop，两者均可进入配置的抬升阶段；状态 2 仅表示位置到位，状态 5 是电流保护，均不能当作抓到物体。该判据仍需本设备验证；即使触发也只表示疑似力控接触，不证明抓牢。

厂商资料：
- https://develop.realman-robotics.com/robot/json/endeffector/
- https://develop.realman-robotics.com/robot/json/motionConfig/
- https://www.changingtek.com/diandong/147

检查：`python -m unittest discover -s tools/foundationpose -p 'test_grasp.py' -v`。不以模拟执行测试代替实机验收。

### 抓取张开位置与机械臂复位

“抓取时夹爪打开位置”接受 0–1000 的整数原始值，1000 表示完全张开，不是毫米。
按回车或离开输入框，通过 `POST /api/gripper/open-position` 携带 `{"value":300}` 保存
`config/grasp/trial.json` 的 `gripper.open_position`；只替换该字段，保留其他配置与格式。
保存不发送硬件命令，下一次抓取生效，重启后保留；独立“打开夹爪”仍固定完全张开。
保存与抓取、复位共享设备锁，任务进行中拒绝修改，日志位于 `work/grasp/open_position_commands/`。

“机械臂复位”调用 `POST /api/arm/reset`，读取 `return_after_lift.joint_deg`，
按现有 `arm_speed_percent` 回到记录的七关节复位姿态，保持夹爪开合不变。
复位不依赖物品跟踪或手眼标定；执行前检查机械臂型号、关节使能/错误、工具/负载、静止状态及夹爪状态。
使用独立的关节运动及到位检查，误差要求不超过 0.5 度；不新增离线路径或途中碰撞检查。
预览模式、未配置返回姿态及已有任务时拒绝复位；“停止运动”可中止抓取或复位。
任务结果沿用状态接口的 `grasp` 字段，`operation=reset` 区分复位；记录保存在 `work/grasp/reset_jobs/`。

### 网页力度配置

力度输入为 1–100 的整数原始值，不是牛顿。执行模式下，按回车或离开输入框通过 `/api/gripper/force-config` 实际写入 1220，匹配的成功回复到达后才更新本次服务配置；下一次抓取使用同一力度。输入本身不发送位置动作，抓取期间或夹爪运动时拒绝更新，设备锁防止与其他抓取进程冲突。失败或超时显示“下发未确认”，不自动重试、不假报已生效。预览模式只改配置、不写硬件。服务重启后恢复文件配置但不自动下发；日志在 `work/grasp/force_commands/`。

2026-09-17 再次只读查询实机：`force:true, touch:false`。官方文档中 `touch` 是触觉支持能力，`touch_sw` 才是触觉开关；通用 `set_rm_plus_touch` 接口的存在不代表当前设备支持触觉。此结论依据协议反馈，不等同于拆机鉴定。

### 初始化和追帧诊断日志

每次服务运行在 `work/foundationpose_live/<时间>/events.jsonl` 自动记录结构化事件，即使尚无一次成功定位也会写入。每个文件最多约 5 MiB，保留 3 个轮转备份（`.1` 至 `.3`）。原有 `poses.jsonl` 仍记录输出位姿，轮转仅针对新增事件日志。

- `attempt_started`：物品、尝试代号 generation、输入序号及图像年龄。
- `detection_completed` / `segmentation_completed` / `registration_completed`：各阶段耗时、候选、掩膜和有效深度像素。
- `catchup_started` / `catchup_progress` / `catchup_completed`：初始延迟、预算、步长、计算耗时、缓存覆盖范围及最终延迟；进度最多每秒一条。
- `attempt_failed` / `frame_failed`：失败阶段、此前阶段耗时、追帧进度、最后质量指标、错误类型和完整 Python 堆栈。
- `attempt_completed`：本次总耗时、各阶段耗时、最终质量与结果年龄；`tracking_summary` 每 5 秒记录一次连续跟踪摘要。
- 同时记录相机/模型失败、物品切换、暂停和服务启停。网页保留最近一次失败及时间，重试不会立即抹去，并显示当前追帧进度和日志路径。

仍直接调用官方 FoundationPose `register` 和 `track_one`；没有放宽深度/轮廓/位姿跳变质量阈值，也未改变机械臂运动授权。初始输入过旧或计算持续慢于实时需求仍会明确失败，不发布陈旧位姿。

最新夹爪验证结论以 [2026-09-18 用户实测记录](GRIPPER_VERIFICATION.md) 为准，旧版“无法写入”的诊断已撤销。

自适应抓取（2026-09-18）：机械臂速度比例保留用户设置 100%，预抓距离 0.05 米（5 厘米），夹持后沿底座竖直方向抬升 0.10 米（10 厘米）。用户确认工具 +Z 是夹爪前进方向。本轮只有模拟验证，未下发实机动作；多朝向精度、控制器实际旋转轨迹及物品/环境碰撞检查尚待验证。

### 使用自适应朝向和指定夹持部位

流程：稳定跟踪 → 示教器摆好夹法 → 记录 → 移开夹爪并重新停稳 → 预览方案 → 抓取。

1. 重启 8089 并刷新页面，选择物品并保持可靠跟踪。通过示教器调整夹爪，使它相对物品的方向符合所需夹法；工具 +Z 指向希望接近物品的方向。
2. 点击“记录当前夹法”。只读取状态，不发送动作。不勾选位置选项时，抓取点采用记录时配置的 grasp_point_object_m，默认模型原点；勾选“同时记录当前位置为夹持点”时，先把夹爪参考点放到希望夹持的实际位置，记录会保存它相对物品的位置。不能把预抓位置误记成夹持点。
3. 同一物品后续平移或旋转，夹持点与夹爪目标朝向一起跟随物品变化。点击“预览抓取方案”只生成并检查方案，不写设备；页面显示目标角度、接近方向和路径点。再点击抓取会使用新的开始快照重新计算并检查，不会盲目执行旧预览。
4. 每个物品保存一份夹法，再次记录会替换。记录持久化于 work/grasp/taught_profiles.json，重启后仍在；该文件未纳入 Git，需要另行备份。模型、工具参数或手眼变换改变时，旧夹法拒绝使用，需要重新记录。

计算：底座下目标朝向 = 底座下物品朝向 × 记录的物品到工具相对朝向；预抓点 = 夹持点 − 目标工具 +Z × 预抓距离。执行中不再读取视觉。工具姿态按项目既有 Rz(yaw) @ Ry(pitch) @ Rx(roll) 约定转换成控制器角度参数。

配置 orientation=follow_object，object_R_tcp 为相对旋转矩阵（3×3 的方向换算表），approach_axis_tcp=[0,0,1]。默认 object_R_tcp=null，未记录时拒绝猜测朝向；也可由已核对的配置提供该矩阵。已记录夹法优先于文件中的方向和夹持点；修改夹持点后需重新记录，或删除该物品的记录后使用明确的文件配置。keep_current_tcp 保留为兼容模式，但同样不再经过固定上方点。

当前不生成平移或旋转中间采样点。程序将预抓点的位置和朝向交给 movej_p（根据目标位置和朝向执行关节运动，中途不保证直线），接近夹持点与抬升交给 movel（直线运动指令），手动复位按钮才使用 movej 返回七关节目标，处理控制器接受/拒绝、完成/失败与超时；控制器返回完成后，读取连续稳定的实际位置及朝向（2 毫米、2°）。运动途中不再做软件几何、工具变化或夹持状态检查，只响应取消请求。圆杯等对称物品的朝向可能不唯一，当前不会自动消除对称跳变或挑选等价夹法，预览阶段需核对方向。

接口：POST /api/grasp/teach，参数 {"use_current_point":false}；POST /api/grasp/preview 无需参数。记录与抓取共享设备锁；记录失败保留上一份夹法，日志在 work/grasp/teach_commands/。物品模型内容摘要随视觉快照保存，防止更换模型后套用旧抓取点。

### 抓取结束与手动复位

抓取流程为：**预抓 → 接近 → 闭合 → 竖直抬升 → 停留并保持夹持**。已删除抓取后的自动返回动作，方案不再包含 `return_joint_raw`，网页也不再显示抬升后返回关节角度。即使旧配置或旧方案中存在返回字段，也不会触发自动返回。

“机械臂复位”按钮独立保留，使用最近记录的七关节目标，夹爪开合保持不变。为兼容已有记录，目标仍存放于 `config/grasp/trial.json` 的 `return_after_lift.joint_deg`，该历史字段名现在仅用于手动复位；来源记录在 `return_after_lift.source`。手动复位以 movej 执行，到位要求每个关节误差不超过 0.5°，不代表完整环境碰撞已检查。复位目标缺失或无效不再影响抓取方案，只影响手动复位是否可用。

### 未发送运动的具体原因

方案的 execution_blockers 和页面提示按条列出所有航点工作区阻塞：航点名称、X/Y/Z 实际坐标、允许范围，或工具参考点距配置桌面的间距、要求值和差值，单位均为毫米。抓取不采样中间轨迹，但航点会检查逆解、自身碰撞和正解残差；错误消息说明失败航点及偏差。预览通过仍不代表控制器已经接受或整条轨迹可达。

最小离桌间距由 config/grasp/trial.json 的 min_table_clearance_m 配置，单位米，缺省为 0.01 米（10 毫米）；table_z_m 仍表示桌面高度。起始点和预抓、夹持、抬升航点要求 Z ≥ table_z_m + min_table_clearance_m；当前不再检查运动中的间距，也不检查关节返回路径的间距。允许有限非负数，负数、布尔值、字符串及非有限数值拒绝。修改配置在下一次任务生效，不改变进行中的任务。

### 运动失败日志

每次任务结束后写入 `work/grasp/jobs/*.json`。`active_stage` 记录最后执行阶段，`motion_events` 按发送顺序记录各段阶段、完整指令、开始时间、超时值、发送完成标记、接受确认、完成反馈、结果与耗时。`responses` 保存最近 32 条原始应答（每条最多 2048 字符），并记录应答到达耗时、截断和丢弃计数；接受及完成应答单独保留。`arrival_checks` 保存目标和段后实际位置/关节状态；异常保存类型和完整调用栈，停止失败单独保存，避免覆盖原始错误。

前端错误包含阶段及具体原因，区分控制器拒绝、未收到接受确认便收到运动结果、控制器报告轨迹失败、超时或断连。原始应答未提供具体硬件错误时，不推断为碰撞或限位。日志不引入额外运动途中查询；需要重启 8089 以加载这次代码修改。历史日志无法补回当时未保存的应答。

## 将物品摆至指定姿态

网页默认选择“自适应位置”：优先尝试终点 XYZ 不变的姿态调整，需要移位时向各方向扩展搜索可达位置，**保持夹持移位 → 转至指定物品朝向 → 在新位置继续夹持**。可切回“固定终点位置”，此时只固定目标端点的 `grip_tcp` XYZ，关节运动中途的位置仍可能变化。

- 朝向始终是硬目标：`base_R_tcp_target = requested_object_rotation × object_R_tcp`。搜索只改变目标位置，不放宽用户指定的物品朝向。物品原点未必等于夹持中心，模型按真实 `object_T_tcp` 显示。
- 自适应模式取消位移半径、目标 XYZ 工作区和“不低于起点”的限制，不使用 `trial.json` 的 `workspace_min_m` / `workspace_max_m` / `workspace_reviewed` / `min_table_clearance_m` / `max_target_distance_m` 限制末端位置。唯一额外的终点位置条件是实际 `grip_tcp` 高度严格大于 `table_z_m + 0.10` 米；初始位置和途中位置不要求达到该高度。固定模式保留原检查。
- 搜索没有距离截止：起点高度足够时先试终点位置不变，否则先试同 XY 上移至最低候选高度；随后不断扩展距离层，包含向下方向和每层变化的球面方向，跳过不满足终点高度的候选。受现有总规划时间预算限制，超时只表示本轮未找到方案，不表示完整空间都无解。选取第一个合格候选，不保证全局最小位移。
- 每个候选检查实际发送的一个或两个目标端点：终点位置不变时直接调整姿态；需要移位时先到选定位置，再到最终姿态。检查端点物品/桌面关系、逆解、自身碰撞和正解残差。两段都使用 `movej_p`，不再把笛卡尔直线或定点旋转插值当成实际路径，也不把整段关节目标差值当成小步跳变拒绝。总规划预算覆盖全部搜索；单次逆解最多 3 秒且不超过剩余预算，候选超时后继续搜索，耗尽或总超时均不发动作。
- 执行仅使用 `movej_p`，由控制器规划关节运动。每段发令前按实际关节状态重做目标正逆解检查，段末等待位置与朝向稳定到位。中途不保证直线、原地旋转或朝向不变，故不再以这些假定误停；watchdog 检查取消、机械臂错误、夹持反馈，并根据实际末端反馈监测物品模型低于桌面的情况。自适应完成后还独立核对实际末端高度严格高于桌面 10 厘米，再保留夹持关系并更新当前物品朝向。候选最低高度额外留出位置容差和一个控制器位置单位的余量。不会自动返回原位置，也不会松开夹爪。
- 模型预览保留原始纹理、物品轴、基座方向和物品自身轴拖拽；自适应方案显示起点/目标 TCP 及位移提示，文字列出基座 ΔX/ΔY/ΔZ、最终 XYZ 和终点离桌高度。调整朝向或位置模式后清除旧位移预览。没有本次有效夹持记录时只可查看模型。

配置仍位于 `config/grasp/reorientation.json`；可选项省略时使用下表默认值，已有速度等用户设置不会被覆盖。旧 `adaptive_max_shift_m`、`rotation_step_deg`、`translation_step_m`、`max_joint_step_deg` 在此功能中已弃用并忽略，前端不再提供最大位移输入。

| 配置 | 默认值 |
|---|---|
| `arm_speed_percent` | 配置文件原值，原默认 10% |
| `planning_timeout_s` | 20 秒，全部候选的总规划预算 |
| `position_tolerance_m` | 0.002 米，段末位置及发令前起点的容差 |
| `orientation_tolerance_deg` | 2°，段末朝向容差 |
| `search_step_m` | 0.025 米，候选距离层间隔 |

接口：`GET /api/reorient/model?object_id=...` 返回模型、夹持中心及 `minimum_final_clearance_m: 0.10`；`POST /api/reorient/preview` 和 `POST /api/reorient` 接收 `{object_id, model_sha256, hold_id, rotation, position_mode}`。`position_mode` 为 `adaptive` 或 `fixed`，缺省 `fixed`；旧请求仍带 `max_shift_m` 时拒绝并提示刷新页面，避免旧界面误导位置许可范围。夹持编号与模型须匹配，所有动作仍共用任务锁、设备锁和“停止运动”。

日志 `work/grasp/reorient_jobs/` 保存规划前真实关节/位姿、工具参数、请求模式、各候选结果、起终点/位移、目标端点、控制器响应、段末反馈及最小观测物品高度。候选日志只保留最初和最近各 64 条，并附候选序号，避免扩大搜索时产生无界日志。旧放置接口、落点和释放流程继续保持删除状态。

验证：`python -m unittest discover -s tools/foundationpose -p 'test_*.py' -q`；`node tools/foundationpose/test_reorientation_viewer.cjs`。软件和离线验证不发送真实动作。端点检查不等于完整关节轨迹或环境碰撞证明，控制器可能选择不同的关节解；预览连线只表示端点位移；夹指/外部障碍与物品滑动未被完整建模，需现场验证实际运动。

### 正解回验与稳定到位

- 逆解返回成功仍须验证：将所得关节角重新做正运动学，比较目标位置和朝向。C++ 拒绝残差超过 2 毫米/2° 的结果；Python 根据输出的正解位姿独立计算，且要求新版协议及全部回验结果。初始关节的正解还须与控制器当前位姿一致，防止离线模型/工具不匹配。
- 控制器完成反馈后，位置误差大于 10 毫米或朝向误差大于 10° 立即报错，由任务异常路径发停止指令。较小偏差最多等待约 0.8 秒（不含无法避免的阻塞查询耗时），每 50 毫秒读取一次；至少连续三次满足最终容差，并覆盖至少 100 毫秒、相邻位置变化 ≤0.5 毫米、朝向变化 ≤0.5° 才继续。
- 抓取最终容差仍为 2 毫米/2°；姿态调整使用其现有配置容差。稳定窗口未形成、误差持续、取消或状态查询失败都停止后续动作，不增加重试运动，不放宽最终容差。姿态调整采用 movej_p，运动期间不再做假定直线路径偏差监测，仍检查实际状态和夹持反馈。
- `kinematics_checks` 记录每段发令前的实际状态与正解回验；`arrival_checks[].samples` 记录有界采样、误差和耗时，`settled` 标明是否完成稳定到位。自适应搜索日志也保留具体正解失败原因。
- 包含历史错误解的完整回归：`SAYDO_IK_CHECKER=$PWD/work/grasp/rm75_ik_check python -m unittest discover -s tools/foundationpose -p 'test_*.py' -q`。未设置此环境变量时，仅跳过调用实际离线 SDK 可执行文件的历史回归项；其余均为模拟测试。
