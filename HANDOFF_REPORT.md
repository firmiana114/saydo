# SayDo（言动）交接报告

更新于 2026-10-08。本文件按“项目概述 → 当前状态 → 阻塞与下一步 → 历史索引”组织；2026-10-08 之前的逐轮详细记录已压缩为第 9 节的单行索引，原文可用 `git show 4340258:HANDOFF_REPORT.md` 查看。硬件接入与 ROS 2 驱动的早期验证见 `ros2_rm_ws/HANDOFF_REPORT.md`。

## 1. 项目概述

- 目标：说一句话，让机械臂完成操作。当前聚焦场景是把 150 mm 透明试管抓起并放入 40 孔试管架。
- 核心链路：自然语言 → 任务执行器调用大模型拆成工具调用 → 定位与抓取后端（相机取图、找框、分割、位姿估计、路径规划与逆解/正解回验）→ 机械臂控制器执行 → 动作后观察核验。
- 技术栈：Python（后端与服务均为标准库 HTTP 服务 + 线程）、原生 JavaScript 前端、C++ 离线逆解检查器（睿尔曼 SDK）、FoundationPose + SAM（位姿估计与分割）、本地 Qwen3-VL-4B NF4 与 Grounding DINO Tiny（找框）、OpenAI Responses 兼容接口（任务编排与可选的 LLM 找框）、MCP stdio 服务、ROS 2 Jazzy（仅驱动验证阶段使用）。
- 仓库：私有 `https://github.com/firmiana114/saydo`。主分支 `main`；`codex/task-runner` 已于 2026-10-08 合并回 `main`。

## 2. 硬件与网络

- 机械臂：睿尔曼 RM75-BI，7 自由度，三代控制器 `192.168.1.19`；示教器网页访问方式见 `AGENTS.md`。
- 夹爪：知行 CTAG2F90D，RM Plus 通信；工具 `grip_tcp` 原点为法兰 Z+222.5 mm，RZ=2.611 rad。
- 眼在手相机：RealSense D435i，序列号 `405622073572`，刚性固定在末端。手眼关系沿用旧相机 `135122077750` 的标定（同型号同安装复用，见 `config/calibration/camera_reuse.yaml`）。
- 运行主机：MSI 笔记本（SSH 别名 `msi`，局域网 `192.168.101.65`，RTX 4060 Laptop 8 GB，NVIDIA 驱动 595.91.07，内核 7.0.0-34）。项目路径 `msi:/home/pc/saydo`；本机仅作开发副本，缺 torch/OpenCV/RealSense，完整测试以 MSI 为准。
- 固定全局相机 D405（`218622270784`）已被拿走：2026-10-08 MSI `lsusb` 只枚举到 D435i。见第 6 节阻塞项。

## 3. 目录结构与主要模块

- `tools/foundationpose/`：核心后端。`live_tracker.py`（8089 服务、相机采集、跟踪、网页）、`live_detection.py`/`local_qwen.py`/`qwen_worker.py`/`local_grounding.py`/`llm_detection.py`（找框）、`scene_*.py`（动作前多目标场景快照）、`grasp_*.py`（夹法库、抓取规划与执行、控制器通信）、`object_motion_*.py`（通用持物运动与释放）、`placement_*.py`、`reorientation_*.py`、`fixed_pick_place.py`（固定试管抓放后端）、`motion_validation.py` + `grasp_ik.cpp` + `ik_beam_search.hpp`（逆解/正解回验与多构型路径搜索）、`agent_api.py`（`/api/agent/` 认证接口）、各 `*.js`/`live_tracker.html`（六页工作台前端）。说明见同目录 `README.md`、`OBJECT_MOTION.md`、`SCENE_SNAPSHOTS.md`、`GRIPPER_VERIFICATION.md`。
- `tools/task_runner/`：自然语言任务执行器（8091，工作台第 5 页）。`tools/robot_agent/`：MCP 服务、Codex 启动器与全局观察 `global_camera.py`。`tools/codex_terminal/`：网页 Codex 终端（8092，第 6 页）。
- `tools/camera_monitor/`：8088 双路相机预览。`tools/handeye/`：眼在手静态配对采集与手眼求解。`tools/object_scan/`：单视角 RGB-D 物品采集。
- `tools/fixed_pick_place.py`：不经 LLM 的固定试管抓放命令行，说明见 `tools/FIXED_PICK_PLACE.md`。`tools/teach_pendant_gateway.py`：本机访问示教器的兼容入口。
- `third_party/FoundationPose/`：固定版本上游源码。`ros2_rm_ws/`：睿尔曼 ROS 2 工作区与驱动启动脚本。
- `config/`：机器本地配置，Git 只保留 `.gitkeep`。`work/`：模型、权重、环境、日志、验证证据，全部不入 Git。

## 4. 服务与数据流

| 端口 | 进程 | 作用 | 监听 |
| --- | --- | --- | --- |
| 8089 | `tools/foundationpose/live_tracker.py` | 定位/抓取后端与工作台网页；启动时拉起常驻 Qwen 子进程 | `192.168.101.65` |
| 8091 | `python -m tools.task_runner.api` | 任务执行器，经 8089 代理到第 5 页 | 回环 |
| 8092 | `python -m tools.codex_terminal.api` | Codex 终端，经 8089 白名单代理到第 6 页 | 回环 |
| 8088 | `tools/camera_monitor/server.py` | 相机预览；D435i 在 8089 运行时共享其画面 | 全部地址 |

数据流：D435i RGB-D → 找框（默认本地 Qwen）→ SAM 分割 → FoundationPose 位姿 → 手眼/工具标定换算到 Base → 夹法库筛选与路径规划（逐采样点逆解→正解回验、10° 连续性、自碰撞、工作区/离桌）→ 控制器 movej/movel → 到位稳定窗口核验 → 执行器取动作后图像核验。任务与持物记录在 `work/agent/tasks.sqlite` 和 `work/grasp/`。

## 5. 运行与常用命令（均在 MSI 项目根目录）

- 启动 8089：`source tools/foundationpose/runtime_env.sh` 后 `python tools/foundationpose/live_tracker.py --host 192.168.101.65 --port 8089 --serial 405622073572 --agent-token-file ~/.local/share/saydo-mcp/robot-backend-token --mesh work/object_models/transparent_test_tube_150mm/model/model.obj --auto-start`。缺省找框方式为 qwen，`--detector llm/ssdlite` 可覆盖。启动不会自动抓取。
- 启动 8088：`./tools/camera_monitor/run.sh --port 8088 --tracker-url http://192.168.101.65:8089`。
- 启动 8091/8092：`work/task_runner_env/bin/python -m tools.task_runner.api`、`work/task_runner_env/bin/python -m tools.codex_terminal.api`。
- 后端测试：`work/foundationpose_env/bin/python -m unittest discover -s tools/foundationpose -p 'test_*.py' -q`（前置 `SAYDO_IK_CHECKER="$PWD/work/grasp/rm75_ik_check"`）。执行器测试：`work/task_runner_env/bin/python -m unittest discover -s tools/task_runner -t . -p 'test_*.py' -v`。MCP 测试：`~/.local/share/saydo-mcp/venv/bin/python -m unittest discover -s tools/robot_agent -p 'test_*.py' -v`。预览测试：`.venv/bin/python -m unittest tools.camera_monitor.test_monitor`。MSI 未装 node，前端 `.cjs` 测试在本机运行。
- 逆解检查器重建：`bash tools/foundationpose/build_ik_checker.sh`，须原子替换原路径二进制并保留备份。
- Python 环境：`work/foundationpose_env`（FoundationPose/CUDA）、`.venv`（RealSense/OpenCV）、`work/task_runner_env`、`~/.local/share/saydo-mcp/venv`（MCP）、Qwen 使用的 `work/vlm_benchmark_20260930/hf_env` 及同目录 `models/Qwen3-VL-4B-Instruct`（torch 2.5.1+cu124、Transformers 5.17，路径由 `config/vision/qwen_local.json` 指定）。均为运行依赖，不要清理。
- 各服务 PID/日志：`work/camera_monitor/`、`work/task_runner/`、`work/codex_terminal/` 下的 `server.pid`、`server.log`。均未配置开机自启。

## 6. 当前状态（2026-10-08）

- 8089 自 2026-09-30 起处于停止状态（当时为双相机标定采集而关闭），因此目前无定位、无抓取；8088（PID 214977）、8091（PID 162930）、8092（PID 115425）仍在运行。停服前无夹持记录，原启动参数另存于 MSI `work/camera_monitor/services_before_native_capture.json`。
- MSI 上运行中的 8088 仍是旧代码（内存中保留已删除的标定取图接口，且在直接采集 D435i），重启后才加载当前代码。恢复 8089 前需先让 8088 释放 D435i（`POST /api/pause`）或重启 8088。
- 2026-10-08 按用户要求删除全部 D405 标定内容：`tools/handeye/capture_pair.py` 及其测试与说明、8088 的 `calibration-frame` 接口、两机 `work/dual_camera_calibration/` 数据（含两版未验收的候选外参）。原因：D405 已被拿走，原有标定作废。候选外参从未写入运行配置，无需回滚配置。
- 用户于 2026-10-08 确认：D435i 手眼标定是精准的，依据是多次实机抓取均可靠执行。后续不再把“换装相机后手眼精度未验证”列为待排查风险。注意本机配置里的 `replacement_accuracy_verified=false` 标记未改动，2026-09-30 的 SayDo 任务 `db8ff066…` 中模型曾据此主动放弃执行。
- 两机配置、物品模型、夹法数据未改动；MSI 未跟踪目录 `config/grasp/observation_pose_backups/` 保留。

## 7. 已验证与未验证

- 已验证：七关节驱动；D435i 采集与手眼标定（用户确认实机抓取可靠）；FoundationPose GPU 环境；本地 Qwen 常驻与旧管线共存（整卡峰值约 6059 MiB，两个双物品场景约 17 秒完成且质量通过）；试管与试管架本地找框（小样本）；场景快照、通用持物运动、放置、调姿、固定抓放的规划与执行逻辑（模拟设备及离线 SDK 回放，最近一次 MSI 全量后端 306 项、执行器 130 项通过）；任务执行器的重试、纠错、证据核验与停止链路。
- 未验证：试管入架的实机成功；夹指与架体、环境的碰撞；力控插入；释放后物品的独立位姿核验；新视角、遮挡、多管场景下的找框泛化。规划通过不等于实物入架通过。

## 8. 阻塞问题与下一步

1. 恢复 8089（需用户确认时机）。恢复后 D435i 回到 640×360 推理分辨率共享模式。
2. D405 已不在：`config/cameras/global.json` 仍指向它，`observe_global` 会失败；而执行器要求每次真实动作后必须取得有效全局图片，否则拒绝继续动作与成功收尾。恢复实机任务前须由用户决定：换一台固定相机并更新配置，或调整/关闭动作后全局观察要求。8088 页面的 D405 一路同样不可用。
3. 试管入架路径：历史 4～5 个下降目标在多构型搜索（每层保留 8 个构型）下仍找不到满足 10° 连续性的完整解，尚需手动示教成功的完整 TCP/关节状态作对照。
4. 夹指/架体碰撞模型缺失；试管架 180° 对称下的孔位编号需物理方向基准。
5. `gpt-5.6-luna` 在服务商侧返回 503（上游负载），未恢复。
6. GitHub 推送状态见最终汇报；`main` 合并后是否推送由用户决定。

## 9. 注意事项

- 文档、网络、登录排查不授权机械臂运动；验证时不触发使能、点动、复位、抓取或夹爪动作。重启 8089/8091 前确认无活动任务、无夹持记录。
- 与物品绑定的数据只放 `work/object_models/<物品ID>/`；不新增跨物品集中配置。`config/` 与 `.env`（权限 600）只在各机本地维护，不提交、不覆盖。
- 重启 8089 必须使用上面的 `--agent-token-file` 新路径；旧进程命令行里的旧路径已失效。已有 MCP/Codex 会话在后端更新后需重新连接才能刷新工具说明。
- 软件停止不能替代现场急停；未知动作结果会阻断新任务，人工核对入口见 `tools/task_runner/README.md`。
- 本机与 MSI 之间不经 GitHub 同步时，可向 MSI 推送临时引用后在 MSI 上快进合并；`work/*.bundle` 是此前用 Git bundle 同步留下的文件。

## 10. 历史工作索引（每项一行，详情见上述旧版报告与对应 `work/*_validation/` 证据）

- 10-08 删除 D405 标定脚本、接口与数据；记录用户对 D435i 标定精度的确认；`codex/task-runner` 合并回 `main`。
- 09-30 双相机标定：新增单组原生采集脚本并离线求解两版 D405 外参，均未验收（两版相差约 25 mm）；已于 10-08 全部删除。
- 09-30 8088 改为 D435i + D405 并排预览，D435i 共享 8089 画面，D405 独立采集并与全局观察共用设备锁。
- 09-30 Qwen3-VL-4B NF4 常驻找框接入 8089 并设为默认；评分前深度反投影按 32 个候选分批以避免显存保护。
- 09-30 试管架 Grounding DINO 本地检测；`plan_release` 引用场景对象时必须给出 `target_position_m`；一次授权实机任务被模型主动终止，无动作。
- 09-30 执行器在每次真实动作后自动 `observe_global`，缺少有效全局图片则拒绝继续；两机位图像来源分别标注。
- 09-30 取消持物/释放规划 20 秒总时限与固定抓放 120 秒外层时限，单次逆解子进程限时保留；历史案例 29 秒找到完整方案（离线）。
- 09-30 透明试管 Grounding DINO Tiny 本地检测，含低对比度拒绝与 15% 框面积上限。
- 09-30 服务商地址改为 `https://trendengine.best/v1`，两机 `.env`、代码默认值与专用 Codex 配置同步。
- 09-29 固定试管抓放脚本：先定位并规划完整路线，默认只规划，`--execute` 才运动；空孔由操作员声明。
- 09-29 MSI 重启后内核缺 NVIDIA 模块，安装对应预编译模块恢复 GPU。
- 09-29 新增 `observe_global` 固定相机观察工具（MCP 与执行器），随后全局机位由 D435i 换为 D405。
- 09-28 多构型联合路径搜索（`BEAM_IK_V1`）；保持姿态的搬运候选与多初值逆解；linear 采样 10° 关节跳变上限。
- 09-28 持物方案严格“先调姿、再保持朝向搬运”；泛化放置流程（上方 100 mm 预放置 → 直线下降 → 松爪 → 可选退出），取代孔位专用检查；特征统一为 `features.json`。
- 09-28 执行器：场景来源纠正、只读规划参数纠错、共享 5 次规划/3 次执行预算、可恢复失败返回 `MOTION_REPLAN_REQUIRED`、`get_task_context` 批量只读与上下文精简。
- 09-28 模型请求：连接未发出、响应超时、响应协议/读取错误三类有限重试（每轮最多 5 次，退避 1/2/4/8 秒）；新增请求阶段诊断。
- 09-28 动作前多目标场景快照（`capture_scene_snapshot`），同一冻结帧定位全部目标，持物后不再重跟踪参照物。
- 09-28 调姿分段统一为默认/最大 15°；模型升级为 6 系列，默认 `gpt-6-sol`。
- 09-24 通用持物运动工具（`plan_held_object_motion`、`plan_release`、`execute_plan` 等）；全流程统一运动速度 25%；闭合接触诊断记录。
- 09-24 工具原点改为 222.5 mm，每套夹法独立 TCP 偏移与最小离桌距离；横躺 5D 物品强制竖直抓取；桌面高度确认为 Base Z −0.02609 m。
- 09-24 40 孔试管架参数模型（272×112×85 mm，孔径 20 mm，孔距 25 mm）；LLM 找框与每物品提示词；切换物品弹窗重排；越界提示区分工作区与桌面。
- 09-24 第 6 页 Codex 终端上线（真实 PTY、只读沙箱、独立 CODEX_HOME）。
- 09-23 自然语言任务执行器上线（第 5 页），默认 execute 模式，含时间线、模型与强度选择、调姿工具；按模型启用 5D 位姿。
- 09-23 Codex 清理重装后恢复，MCP 独立环境与后端令牌迁至 `~/.local/share/saydo-mcp/`。
- 09-22 机械臂 MCP 工具与 `/api/agent/` 认证接口；夹爪力度与张开位置绑定到每套夹法。
- 09-21 三维夹法编辑器与多夹法库；模型绑定数据迁入各物品目录；桌面自适应放置及其后续调整（关节转移、取消边缘余量、松爪后沿工具轴后撤）；两种复位；调姿失败后保留夹持以便重试；三栏工作台与各类观察/旋转圆环控件。
- 09-20 同型号同安装相机复用手眼关系；抓取/调姿的正解回验与到位稳定窗口；指定姿态改用 movej_p；取消抓取后自动返回；手动复位目标更新。
