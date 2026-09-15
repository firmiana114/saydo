# MSI 睿尔曼机械臂驱动验证

验证日期：2026-09-11。工作区：`/home/pc/saydo/ros2_rm_ws`，SSH 别名：`msi`。

## 相机与夹爪接入状态（2026-09-14）

- 用户确认：Intel RealSense D435i 已连接本机，刚性固定在机械臂末端夹爪旁、随末端运动；夹爪为知行机器人 CTAG2F90D，官网写作 CTAG2F90-D，已连接到机械臂。
- 本次会话的只读设备枚举已识别 D435i；未启动图像采集，未发送夹爪或机械臂运动指令。相机采集、夹爪控制和标定结果均未验证。
- 现场已有普通黑白棋盘格标定板；横纵格数、单格实际边长、夹指及转接件尺寸、夹爪实际通信配置未确认。
- 型号、安装关系及官方资料入口已纳入根目录 [README.md](../README.md#相机夹爪与官方资料)，包括 D435i 开发与驱动说明，以及知行《CRT 睿尔曼+知行协作手操作说明》的资料中心入口。
- 资料核对：RealSense 开发与驱动页面可读取；知行产品与资料中心信息已通过官方网页的检索索引核对，直接访问暂未成功，未验证其中各下载文件。
- 本次仅修改项目说明与本报告，`git diff --check` 格式检查通过；未修改驱动、配置或本地辅助脚本。
- 下一步：确认棋盘格尺寸，验证相机采集，核实机械臂反馈的坐标参照后采集标定数据；夹爪实际抓取中心另行确认。以下机械臂运行与运动结果仍为 2026-09-11 的历史记录。

## 结果与证据

- 设备实际为 **RM75-BI，7 自由度，第三代控制器，固件 V1.7.0**，不是最初猜测的 RM65。
- 控制器 TCP：`192.168.1.19:8080`；MSI 有线网口 `enp3s0`：`192.168.1.10/24`。
- ROS 驱动成功连接、查询设备身份并发布真实的 7 关节反馈。
- 实机先完成第 7 关节 2° 往返，再完成更便于观察的 **10° 往返，速度 1%，到位停留 3 秒**；10° 转出、返回各约 5.1 秒。
- 10° 测试采集 96 组 ROS/控制器对照样本，最大角度差约 0.0342°；其余六关节相对起点最大变化 0.004°。
- 最后一次控制器查询中，第 7 关节为 0.000°，系统错误为 0，全部关节已使能，关节错误码均为 0。
- 现场观察确认：用户已确认第 7 关节 10° 往返“看清了，运动正常”；随后明确要求验证全部关节。
- 追加验证：第 1～7 关节逐一以 1% 速度转动 +3°，停留 2 秒，再返回起点；7/7 关节、14/14 运动段全部通过。每次只移动一个关节。
- 全关节验证采集 260 组 ROS/控制器对照样本，最大角度差 0.0478°，非目标关节最大变化 0.0070°。
- 验证范围是基本驱动通信、全部七关节的小幅指令执行及反馈闭环；未验证全行程、MoveIt、笛卡尔轨迹、负载抓取或全部驱动功能。

## 环境与构建

系统为 Ubuntu 24.04 x86_64，安装了官方 ROS 2 Jazzy ROS Base、ros-dev-tools、build-essential、CMake、arping 及依赖。`dpkg --audit` 未报告问题。

保留复制过来的 Humble `build/`、`install/`、`log/`。重新编译使用独立目录：

```bash
cd /home/pc/saydo/ros2_rm_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base log_jazzy build --base-paths src/ros2_rm_robot \
  --build-base build_jazzy --install-base install_jazzy \
  --packages-select rm_ros_interfaces rm_driver --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
```

两个包编译通过。已比较官方 Jazzy 分支：驱动核心源码与现有 Humble 分支一致，差异集中在配置和接口说明，因而直接沿用现有源码编译。未修改源码仓库；原有 `rm_moveit2/CMakeLists.txt` 修改和安装日志未动。

## 启动与当前状态

当前驱动在后台运行，启动 PID 记录在 `work/rm75-launch.pid`。未配置开机自动启动，不会自动重复运动测试。

下次启动前先确认没有已有驱动进程；已停止时运行：

```bash
cd /home/pc/saydo/ros2_rm_ws
./start_rm75_driver.sh
```

脚本加载 Jazzy 与 `install_jazzy/local_setup.bash`，设置 `ROS_DOMAIN_ID=75` 和 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，启动 `rm_75_driver.launch.py`。需要使用 ROS CLI 时，同样先加载这两个环境文件并设置这两个变量。

驱动沿用 `src/ros2_rm_robot/rm_driver/config/rm_75_config.yaml` 中正确的 RM75、7 关节、TCP `.19`、UDP `.10:8089` 配置。启动时将控制器状态反馈目标从原先 `.234:8090` 改为 MSI 的 `.10:8089`；API 日志报告反馈周期 5 ms，JSON 协议查询中的 `cycle=1` 表示一个 5 ms 单位。

网络新增 NetworkManager 配置 `realman-direct`，有线静态地址 `.10/24`、不设置默认路由、关闭该配置的 IPv6。原 Wi-Fi 和原有有线配置均保留。到机械臂的路由已经直接走 `enp3s0`，不再经过 Clash 的 Meta 虚拟网卡。

## 故障处理记录

最初系统错误 `4116 / 0x1014` 为关节超出软限位：第 6 关节约 -130.462°，下限 -128°。用户按照设备说明现场恢复后，重新读取确认错误清零、关节回到零位附近。未放宽限位、未关闭碰撞保护。

第一次 2° 检查已使关节到位，但独立 TCP 观察脚本收到控制器广播的异步到位事件，误按状态查询响应解析，触发停止。已修复观察脚本，使其按响应类型匹配查询，保留异步事件记录；随后返回原点及完整 10° 往返均通过。此修复不涉及厂商驱动源码。

## 日志与辅助文件

- `work/rm75-driver.log`：驱动运行日志。
- `work/build-jazzy.log`、`work/apt-install.log`：构建与安装记录。
- `work/all-joints-validation.json`、`work/all-joints-validation.log`、`work/all-joints-summary.json`：最终七关节逐一验证的全部样本、结果与汇总。
- `work/motion-validation.json`、`work/motion-validation.log`：之前第 7 关节 10° 测试完整样本与结果。
- `work/verification-summary.json`、`work/final-controller-state.json`：最终汇总与直接状态查询。
- `work/motion-first-attempt.json`、`work/motion-2deg-validation.json`：最初停止及后续 2° 恢复记录。
- `work/msi_motion_smoke.py`：单独第 7 关节有界检查；`work/msi_all_joints_smoke.py`：七关节逐一检查。无 `--execute` 时均不运动。脚本会真实驱动机械臂，后续运行须重新确认现场条件。

官方依据：[驱动说明](https://develop.realman-robotics.com/robot/ros2/driver/)、[错误代码](https://develop.realman-robotics.com/en/robot/json/errorList/)、[Jazzy 分支](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy)。

## 项目目录迁移

工作区已迁入 SayDo（言动） 项目。原路径 `/home/pc/ros2_rm_ws` 保留为兼容符号链接，指向当前工作区，以保留运行中驱动及历史绝对路径的可用性。迁移过程中未重启驱动、未发送运动指令。

## GitHub 版本管理

项目根目录纳入 `firmiana114/saydo` 私有仓库，默认分支 `main`。根目录 `doc/`、编译与安装目录、日志、运行记录、模型权重和凭据文件由 `.gitignore` 排除。GitHub 保存第三方源码实际文件及必要 SDK 动态库，来源记录见项目根目录 `VENDORED_SOURCES.md`。嵌套的上游 Git 元数据和既有 Boost 修改在 MSI 原地保留。

首次推送通过已认证的本机 GitHub CLI 完成，没有把本机 GitHub 凭据复制到 MSI。以后从 MSI 推送需使用该主机自己的 GitHub 认证，或继续通过已认证的本机同步。

## 2026-09-14：手眼标定静态采集准备

新增 `../tools/handeye/` 只读配对采集工具，使用项目 `.venv`，详见该目录 README。7 项测试通过；第一组真实样本已保存为候选，88 角点齐全、重投影 RMS 0.5935 px、法兰位置跨度 0.03454 mm。数据位于项目根目录 `work/handeye/initial_static_check/134310_cc77b0ad/`，不纳入 Git。没有发送机械臂运动、夹爪或负载写入命令。

本次为采集原始 PNG 而调用相机监控的 pause 接口释放相机；采集完毕设备已释放，恢复网页预览需要点击“开始预览”。现有未提交的 `tools/camera_monitor/` 保留，不纳入本次提交。

尚未完成手眼标定：只有一个姿态，棋盘实测尺寸/角点方向待复核，仍缺多姿态采集和独立验收。工具当前负载为 0；厂家公开参数可确认 CTAG2F90D 夹爪 0.8 kg 和 D435i 相机 75 g，但不能据此推定整套附件质量及质心。已查到睿尔曼官方负载辨识流程，需结合实际安装和安全空间实施，不会自动执行默认大范围运动。检索补充见本地 `../doc/标定准备补充检索.md`。

## 2026-09-14：he_ref 负载辨识结果与采集适配

用户完成 `he_ref` 的负载辨识；指定工具查询确认 pose=[0,0,0,0,0,0]，JSON payload=1032.6759033203125 对应约 1.032676 kg，position=[-19000,6000,110000] 对应质心 (-19,6,110) mm。位姿零偏移与非零质心是不同字段，不应混淆。记录的是控制器辨识值，尚未做实际质量校核。

读回时当前工具仍为 `Arm_Tip`、负载 0，工作坐标系为零变换 `Base`；可见工具已保存不代表已经选用。本次未切换工具、未移动机器人。采集工具现在支持零偏移 `he_ref`，仍拒绝非零工具/工作变换和未知工具名称。新增相关测试后共 10 项测试通过。下一步在示教器选择 `he_ref` 并确认生效，再将相机带回可见棋盘的初始姿态进行采集。

## 2026-09-14：单次自动去重与验证集采集

新增 `../tools/handeye/collect.py`，每次运行仅进行一次检查/采集并退出。手动调整后运行 `.venv/bin/python tools/handeye/collect.py --session he_ref_calibration_20260914`（从项目根目录执行）。不发送运动命令、不循环或自动重试、不恢复预览。采集前后均按位置差 <10 mm 且旋转差 <5° 筛查重复，沿用原始采集质量检查，并核对设备/坐标系一致性。通过的样本只登记到独立验证候选集，不改变冻结的前 20 组求解名单。

19 项测试通过，包括去重跳过、采集后去重、拒收不重试、冻结分组不变、满额不访问硬件及中断登记恢复。实机只运行一次，成功新增 `171206_e8bee9d1`，88 角点、RMS 0.5103 px；现有求解候选 20 组、验证候选 1/5 组、拒收记录 4 次。后续质量审阅与独立验收尚未完成。现有未提交 `tools/camera_monitor/` 保留。

## 2026-09-14：25 组数据的离线求解与独立验证

已核对求解候选 20 组、独立验证候选 5 组，另有 4 次拒收；源图像哈希、角点、相机和坐标系配置检查通过。新增 `../tools/handeye/solve.py`，固定 PARK 方法，只用冻结的 20 组拟合及建立棋盘参考位姿，验证集不用于拟合、算法选择或调参。

求解环境单独位于项目根目录 `work/handeye_solver_venv`（OpenCV 4.14.0），原采集 `.venv` 保持 OpenCV 5.0.0。求解测试 6 项通过；采集环境回归测试 19 项通过，求解专用测试在采集环境按预期跳过。依赖见 `tools/handeye/requirements-solve.txt`。

初步结果保存在 `work/handeye/he_ref_calibration_20260914/solutions/172825_bd2eabc9/result.json`（路径相对项目根目录）。独立验证位置 RMS 6.859 mm、最大 8.911 mm，朝向 RMS 0.319 度、最大 0.494 度；求解集位置 RMS 6.183 mm、最大 10.580 mm。留一法最大变换变化 1.835 mm / 0.094 度。矩阵方向为彩色光学坐标系到法兰，平移单位米。

结果未部署，未发布 TF、未修改控制器。用户现已明确将该组标定标记为可用；这一接受决定未改变已有误差指标，也未新增棋盘尺寸或安装几何的实测证据。不能把单帧重投影误差或固定板一致性等同于绝对抓取精度。本地详细报告见 `../doc/手眼标定初步结果.md`，不纳入 Git。

## 2026-09-14：用户确认该组标定可用

用户明确要求将 `172825_bd2eabc9` 标记为可用。MSI 上该次 `result.json`、`calibration_progress.json`、本地报告和待办清单均已同步状态：`acceptance_status=user_accepted_usable`，`acceptance.source=explicit_user_confirmation`，确认时间为 `2026-09-14T17:48:11.546354+08:00`。

变换矩阵、20/5 数据分组和误差统计完整保留，`deployed=false`。原结果备份为同目录 `result.before_user_acceptance.json`。此次仅记录接受状态，没有重新求解、部署 TF、修改控制器或移动机械臂。该决定仅适用于此组标定，不更改求解器对未来结果的默认待确认状态。

## 2026-09-15：统一保存手眼与抓取工具配置

参数入口为项目根目录 `config/calibration/`，使用方法与来源见 [标定配置说明](../config/calibration/README.md)。`handeye.yaml` 精确保留已接受的 `172825_bd2eabc9` 矩阵及验证统计；`tool_grip_tcp.yaml` 保存当前控制器读回的 Z=0.2225 m、零旋转工具位姿，负载约 1.060534 kg、质心读回值 (-17,-14,86) mm。TCP 为手动名义尺寸偏移，几何误差未验证；夹爪逻辑位置 500 仅为用户报告的负载辨识条件，本次实际位置读回为 499。

两份 YAML 的 SI 单位换算、矩阵合法性、来源哈希及与源数据的一致性已检查。原始手眼数据未修改，控制器快照保存在被忽略的 `work/calibration/`。本次只保存配置和说明，未实现运行时自动检查、未接入 FoundationPose、未发布 TF、未写入机器人或发送运动指令。后续使用 `grip_tcp` 反馈时，须先右乘 `inverse(flange_T_grip_tcp)` 还原法兰位姿。

## 2026-09-15：胶带无 CAD 建模的单视角采集入口

用户选择从照片建模，并明确要求每次脚本只采一个视角，由现场手动拖动机械臂改变相机视角。新增 `../tools/object_scan/`；项目根目录运行 `.venv/bin/python tools/object_scan/capture_view.py --session tape_20260915`。胶带和机械臂基座须保持固定，采集结束不自动恢复相机预览。它复用已有只读 Robot 查询和静止检查，不发送运动或夹爪命令。

入口校验已接受手眼矩阵及来源哈希、当前完整 grip_tcp 工具/负载、零偏移 Base、相机序列号与内参。先按 TCP 逆变换还原法兰，再求 base_T_camera；静止采集原始彩色、对齐深度原值/毫米 PNG/米制 NPY、K、时间戳、配置与数据哈希。会话冻结标定配置/相机参数；前后两次去重，重复或拒收只退出、不重试。

MSI 新增采集依赖 PyYAML 6.0.3，4 项几何/深度测试通过。`work/object_scan/tape_20260915/views/101121_b43524bc/` 为首个真实候选视角：1280×720，7 个文件哈希、形状、深度单位和位姿互逆回读通过；整幅图深度有效率 95.57%，不是目标胶带区域指标。相同姿态再次运行返回 duplicate，未再次占用相机。现在有 1 个候选视角，仍需现场逐次采集更多视角。

FoundationPose 官方 `bundlesdf/run_nerf.py` 支持带相机位姿的多视角神经重建。后续仍需逐视角目标 mask（内孔为背景）、目标深度/配准质量检查、物体参考系选取、重建环境与模型权重部署、网格重建和 register/track_one 验证。目前只有采集入口完成，未生成胶带模型、未运行 FoundationPose、未发布 TF 或启用抓取。MSI RTX 4060 Laptop 8 GB、驱动 595.84；本次检查未发现 Docker、Conda、nvcc 或 torch，未安装重建依赖。原有未提交 `tools/camera_monitor/` 保留。

## 2026-09-15：接入 FoundationPose 官方源码

用户明确要求接入源码。完整上游快照位于 `../third_party/FoundationPose/`，固定提交 `a1b694b83e633c2cb6115b9063d940a687759392`，含 54 个原文件及许可证；未修改上游代码。来源及 SHA-256 清单位于 `../third_party/FoundationPose.upstream.json`，项目根目录运行 `python3 tools/foundationpose/check_source.py` 校验。说明见 `../tools/foundationpose/README.md`。

本次接入包含 estimater/register/track_one 与 bundlesdf/run_nerf 神经重建入口，不包括 GPU 环境、权重安装或胶带重建运行。当前无 CAD 路线还需 bundlesdf/mycuda 和 Kaolin，不能把只构建 mycpp 的 build_all_conda.sh 当作完整重建环境。相机采集环境、既有数据、机器人参数和未提交 camera_monitor 未修改。

## 2026-09-15：GPU 环境、胶带重建和离线推理

当前入口与复现命令以 `../tools/foundationpose/README.md` 为准。独立环境 `work/foundationpose_env` 和运行副本 `work/foundationpose_runtime` 均在项目根目录；保留原相机 `.venv`。实际版本为 PyTorch 2.5.1+cu124、nvcc 12.4.131、PyTorch3D 0.7.9、NVDiffRast 0.3.3、Kaolin 0.18.0、Warp 1.6.2；RTX 4060 Laptop 8 GB。完整记录在 `config/foundationpose/environment_state.json`，Conda 包 URL 清单为 `conda-linux-64.lock`。pip check、54 文件源码校验及 GPU 核函数/权重检查通过。

运行副本应用可追溯的 C++17、scalar_type 兼容补丁及 mycuda 包路径统一。曾出现模型已保存但解释器退出 SIGABRT；GDB 定位到同一 common 扩展以两个名字加载，释放静态 pybind11_module_def_common 时异常。统一导入后，实际 ray_trace 回归和完整神经重建均正常退出，原始上游快照未改动。Warp 设备 UUID API 仍有非阻塞兼容警告，实际深度 CUDA 运算已通过。

官方 Google Drive 部分权重配额耗尽。当前推理权重来自固定公开镜像 gpue/foundationpose-weights 的 revision 42d49e0633d245b3cf4dea6b1e7ec2b31d5b7654，记录了 SHA-256、大小与来源；精修配置与官方下载文件相同，但模型本体没有官方哈希对照，不声称已核实为官方原件。启用 weights_only 加载；两组网络加载和推理通过。SAM ViT-B 权重来自 Meta 官方地址。权重不纳入 Git。

使用 tape_20260915 的 13 组图像，以 SAM 提示分割并修正了两个视角的阴影/纸芯错误。最终 `work/object_scan/tape_20260915/segmentation_sam_v3` 已记录目视审阅，原始采集未改动。11 组拟合，102616_65d32a20 和 102905_faba178f 最初留作检查。

`work/foundationpose_runs/tape_20260915/nerf_v3_rgb` 完成 1000 配置步数的神经场训练、网格提取和纹理生成，正常退出。model.obj、material.mtl、material_0.png 为纹理模型；model_geometry.obj 为原始几何。几何约 16114 顶点、32252 面，外形范围约 109.6×107.8×67.4 mm，存在多余孔洞和起伏，未验收。它来自真实多视角神经重建，没有替换为圆环原始模型。

离线 register 加同帧 track_one 运行通过，但原始两视角轮廓 IoU 为 0.949/0.382；按观测 mask 诊断前 20 个候选后为 0.955/0.418，第二视角仍不可靠。报告分别在 heldout_evaluation 与 development_rerank_top20。局部深度误差仅统计重合区域，不代表整体定位精度。两组已用于开发分析，不再是独立验收集，需补新视角检查。

连续实时跟踪、绝对位姿精度、ROS TF 和实机抓取未完成。下一步应改善胶带侧壁深度与草模质量，并重新采集未参与开发的验证视角。本次没有发送机器人或夹爪运动/参数命令；未提交的 tools/camera_monitor 保留。
