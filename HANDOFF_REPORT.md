# SayDo（言动）交接报告

更新于 2026-10-08。本文件按“项目概述 → 当前状态 → 阻塞与下一步 → 历史索引”组织；2026-10-08 之前的逐轮详细记录已压缩为第 9 节的单行索引，原文可用 `git show 4340258:HANDOFF_REPORT.md` 查看。硬件接入与 ROS 2 驱动的早期验证见 `ros2_rm_ws/HANDOFF_REPORT.md`。

## 1. 项目概述

- 目标：说一句话，让机械臂完成操作。当前聚焦场景是把 150 mm 透明试管抓起并放入 40 孔试管架。
- 核心链路：自然语言 → 任务执行器调用大模型拆成工具调用 → 定位与抓取后端（相机取图、找框、分割、位姿估计、路径规划与逆解/正解回验）→ 机械臂控制器执行 → 动作后观察核验。
- 技术栈：Python（后端与服务均为标准库 HTTP 服务 + 线程）、原生 JavaScript 前端、C++ 离线逆解检查器（睿尔曼 SDK）、FoundationPose + SAM（位姿估计与分割）、本地 Qwen3-VL-4B NF4 与 Grounding DINO Tiny（找框）、OpenAI Responses 兼容接口（任务编排与可选的 LLM 找框）、MCP stdio 服务、ROS 2 Jazzy（仅驱动验证阶段使用）。
- 仓库：私有 `https://github.com/firmiana114/saydo`。主分支 `main`；`codex/task-runner` 已于 2026-10-08 合并回 `main`，两个分支均已推送，本机、MSI、GitHub 处于同一提交。

## 2. 硬件与网络

- 机械臂：睿尔曼 RM75-BI，7 自由度，三代控制器 `192.168.1.19`；示教器网页访问方式见 `AGENTS.md`。
- 夹爪：知行 CTAG2F90D，RM Plus 通信；工具 `grip_tcp` 原点为法兰 Z+222.5 mm，RZ=2.611 rad。
- 眼在手相机：RealSense D435i，序列号 `405622073572`，刚性固定在末端。手眼关系沿用旧相机 `135122077750` 的标定（同型号同安装复用，见 `config/calibration/camera_reuse.yaml`），换装后的精度已由用户确认。
- 运行主机：MSI 笔记本（SSH 别名 `msi`，局域网 `192.168.101.65`，RTX 4060 Laptop 8 GB，NVIDIA 驱动 595.91.07，内核 7.0.0-34）。项目路径 `msi:/home/pc/saydo`；本机仅作开发副本，缺 torch/OpenCV/RealSense，完整测试以 MSI 为准。
- 不固定的全局相机：RealSense D405，序列号 `218622270784`，装在一个普通的、位置可能变动的支架上，只承担全局观察，不做到基座的标定。2026-10-08 重新接入并实拍验证通过。

## 3. 目录结构与主要模块

- `tools/foundationpose/`：核心后端。`live_tracker.py`（8089 服务、相机采集、跟踪、网页）、`live_detection.py`/`local_qwen.py`/`qwen_worker.py`/`local_grounding.py`/`llm_detection.py`（找框）、`scene_*.py`（动作前多目标场景快照）、`grasp_*.py`（夹法库、抓取规划与执行、控制器通信）、`object_motion_*.py`（通用持物运动与释放）、`placement_*.py`、`reorientation_*.py`、`fixed_pick_place.py`（固定试管抓放后端）、`motion_validation.py` + `grasp_ik.cpp` + `ik_beam_search.hpp`（逆解/正解回验与多构型路径搜索）、`agent_api.py`（`/api/agent/` 认证接口）、各 `*.js`/`live_tracker.html`（五页工作台前端）。说明见同目录 `README.md`、`OBJECT_MOTION.md`、`SCENE_SNAPSHOTS.md`、`GRIPPER_VERIFICATION.md`。
- `tools/task_runner/`：自然语言任务执行器（8091，工作台第 5 页）。`tools/robot_agent/`：MCP 服务、命令行 Codex 启动器 `codex_launcher.py` 与全局观察 `global_camera.py`。
- `tools/camera_monitor/`：8088 双路相机预览。`tools/handeye/`：眼在手静态配对采集与手眼求解。`tools/object_scan/`：单视角 RGB-D 物品采集。
- `tools/fixed_pick_place.py`：不经 LLM 的固定试管抓放命令行，说明见 `tools/FIXED_PICK_PLACE.md`。`tools/teach_pendant_gateway.py`：本机访问示教器的兼容入口。
- `third_party/FoundationPose/`：固定版本上游源码。`ros2_rm_ws/`：睿尔曼 ROS 2 工作区与驱动启动脚本。
- `config/`：机器本地配置，Git 只保留 `.gitkeep`。`work/`：模型、权重、环境、日志、验证证据，全部不入 Git。

## 4. 服务与数据流

| 端口 | 进程 | 作用 | 监听 |
| --- | --- | --- | --- |
| 8089 | `tools/foundationpose/live_tracker.py` | 定位/抓取后端与工作台网页；启动时拉起常驻 Qwen 子进程 | `192.168.101.65` |
| 8091 | `python -m tools.task_runner.api` | 任务执行器，经 8089 代理到第 5 页 | 回环 |
| 8088 | `tools/camera_monitor/server.py` | 相机预览；D435i 在 8089 运行时共享其画面 | 全部地址 |

数据流：D435i RGB-D → 找框（默认本地 Qwen）→ SAM 分割 → FoundationPose 位姿 → 手眼/工具标定换算到 Base → 夹法库筛选与路径规划（逐采样点逆解→正解回验、10° 连续性、自碰撞、工作区/离桌）→ 控制器 movej/movel → 到位稳定窗口核验 → 执行器取动作后图像核验。任务与持物记录在 `work/agent/tasks.sqlite` 和 `work/grasp/`。

## 5. 运行与常用命令（均在 MSI 项目根目录）

- 关闭并启动 8089（推荐）：`ssh msi /home/pc/saydo/tools/foundationpose/restart_tracker.py`。先检查无任务、无夹持再停旧进程，后台启动并等待模型就绪；被拒绝时按提示处理或加 `--force`。说明见 `tools/foundationpose/README.md` 的“一条命令关闭并启动 8089”。
- 启动 8089：`source tools/foundationpose/runtime_env.sh` 后 `python tools/foundationpose/live_tracker.py --host 192.168.101.65 --port 8089 --serial 405622073572 --agent-token-file ~/.local/share/saydo-mcp/robot-backend-token --mesh work/object_models/transparent_test_tube_150mm/model/model.obj --auto-start`。缺省找框方式为 qwen，`--detector llm/ssdlite` 可覆盖。启动不会自动抓取。经 `ssh msi '…'` 一行后台启动时，须把 `nohup setsid python … > 日志 2>&1 < /dev/null &` 整体放进括号子壳 `( … & )`，否则等待它的外层壳仍占着 ssh 的输出管道，ssh 命令不会返回（服务本身不受影响）。
- 启动 8088：`./tools/camera_monitor/run.sh --port 8088 --tracker-url http://192.168.101.65:8089`。
- 启动 8091：`work/task_runner_env/bin/python -m tools.task_runner.api`。
- 后端测试：`work/foundationpose_env/bin/python -m unittest discover -s tools/foundationpose -p 'test_*.py' -q`（前置 `SAYDO_IK_CHECKER="$PWD/work/grasp/rm75_ik_check"`）。执行器测试：`work/task_runner_env/bin/python -m unittest discover -s tools/task_runner -t . -p 'test_*.py' -v`。MCP 测试：`~/.local/share/saydo-mcp/venv/bin/python -m unittest discover -s tools/robot_agent -p 'test_*.py' -v`。预览测试：`.venv/bin/python -m unittest tools.camera_monitor.test_monitor`。MSI 未装 node，前端 `.cjs` 测试在本机运行。
- 逆解检查器重建：`bash tools/foundationpose/build_ik_checker.sh`，须原子替换原路径二进制并保留备份。
- Python 环境：`work/foundationpose_env`（FoundationPose/CUDA）、`.venv`（RealSense/OpenCV）、`work/task_runner_env`、`~/.local/share/saydo-mcp/venv`（MCP）、Qwen 使用的 `work/vlm_benchmark_20260930/hf_env` 及同目录 `models/Qwen3-VL-4B-Instruct`（torch 2.5.1+cu124、Transformers 5.17，路径由 `config/vision/qwen_local.json` 指定）。均为运行依赖，不要清理。
- 各服务 PID/日志：`work/foundationpose_live/server.pid`（8089，日志为同目录各次 `restart_*.log`）、`work/camera_monitor/`、`work/task_runner/` 下的 `server.pid` 与日志。均未配置开机自启。

## 6. 当前状态（2026-10-08）

- 三个服务在运行：8089（PID 303337，常驻 Qwen 子进程 PID 303485）、8088（PID 273955）、8091（PID 273957）。8089 最近一次于 2026-10-08 10:57 由重启脚本启动，日志 `work/foundationpose_live/restart_script_20261008_105711.log`；PID 文件均已更新。8092 已随网页 Codex 终端一并删除。
- 眼在手相机拔插后深度流不出帧（2026-10-08，已恢复）：D435i 于 10:50:44 从 USB 断开、10:54:05 重新枚举（USB 3.2）。此后 8089 报 `camera_error`、“Frame didn't arrive within 4000”，重启 8089 无效。单独测试发现仅彩色正常、仅深度及彩色+深度均取不到帧；对相机执行 SDK `hardware_reset()` 后三种配置都恢复，再用重启脚本启动 8089 即回到 tracking、帧龄约 0.1 秒。断开原因未确认。经验：热插拔后若深度不出帧，先硬件复位相机再重启 8089；8089 的采集线程出错后不会自行重试。
- 未确认事项：10:05 启动的上一个 8089 进程（PID 275471）在 10:19:12 左右退出，日志无报错堆栈，系统无内存不足杀进程或崩溃转储记录，退出前状态为正常跟踪、无夹持；迹象符合被外部信号停止，但是否为人工操作未确认。
- 重启脚本 `tools/foundationpose/restart_tracker.py`（2026-10-08）：8 项单元测试通过；已对真实 8089 实跑“检查→停止→启动→等待就绪”，约 22 秒、退出码 0；任务进行中或有夹持时按设计拒绝。曾发生两处同时重启、后一次停掉前一次刚启动的进程，因此加了 `work/foundationpose_live/restart.lock` 互斥锁。
- 固定抓放脚本的 `--rack-box` 改为可选（2026-10-08）：省略时由 8089 当前的自动检测器（缺省本地 Qwen）给框，可用 `work/task_runner_env/bin/python -m tools.fixed_pick_place --empty-rack` 全自动找框。实测（只规划，记录 `work/fixed_pick_place/fixed-f567fc7c7f12fb407a03/`）：两个框均未传入，场景采集 ready，试管框 `[488,44,548,159]`、试管架框 `[0,41,184,360]`，两者位姿质量均通过，未发送动作。规划随即失败：“没有通过模型直径与竖直下降几何检查的孔位”。原因是估计的试管架 Z 轴偏离竖直约 2.6°，按竖直下降 70 mm 计算横向偏移约 3.1 mm，超过孔与试管的单侧余量 3 mm（对应倾角上限约 2.45°），全部孔位被排除。当时试管架贴着画面左缘和下缘、只有部分入镜，倾角更可能来自位姿估计误差而非架子真的倾斜（未确认）。
- 固定抓放首次规划出完整方案（2026-10-08，用户运行，只规划未执行，记录 `work/fixed_pick_place/fixed-341934599244bb7463bf/`）：试管架完整入镜后（自动框 `[10,55,346,267]`，不再贴边）估计倾角降为 0.89°，通过几何检查；选中孔位 A08，`complete_path_prechecked=true`，放置段为 9 段调姿 + 预放置 + 竖直下降，速度 25%。这印证了上一条关于“部分入镜导致倾角偏大”的推断。未检查项：关节运动中间路径、夹指碰撞、环境碰撞、物品支撑。**夹指余量风险**：方案的夹持点在试管模型原点（长度中点，距底端 75 mm），入架后试管底端距架底 15 mm，夹持中心只比 85 mm 高的顶板高出约 5 mm；夹指沿试管轴向的半宽若超过 5 mm 就会碰到顶板，执行前须实物核对或改用更靠近管口的夹法。
- 8089 读回：`model_ready=true`、`error=null`、找框方式 qwen、相机 D435i `405622073572` 640×360、帧龄约 0.11 秒、抓取模式 execute、`held_object=null`、不忙。当时画面内没有试管，状态为 `detecting`（持续寻找目标），不是故障。8091 重启后读回 execute、`gpt-6-sol`/medium、无停止中、无不确定阻断，20 条历史任务仍在。整卡显存占用约 3511 MiB。
- 2026-10-08 按用户要求删除全部 D405 标定内容：`tools/handeye/capture_pair.py` 及其测试与说明、8088 的 `calibration-frame` 接口、两机 `work/dual_camera_calibration/` 数据（含两版未验收的候选外参）。原因：D405 已被拿走，原有标定作废。候选外参从未写入运行配置。9 月 30 日关停 8089 前的启动参数记录移至 MSI `work/camera_monitor/services_before_native_capture.json`。
- 用户于 2026-10-08 确认：D435i 手眼标定是精准的，依据是换装后多次实机抓取均可靠执行。`camera_checks.camera_binding` 原先把 `replacement_accuracy_verified` 固定为 `False`，现改为读取 `camera_reuse.yaml` 的同名字段（仅显式 `true` 生效）；两机该配置已设为 `true` 并写明依据，原文件与标定说明备份在各自 `work/calibration/camera_reuse_accuracy_20261008/`。这是用户依据实际使用作出的确认，不是软件测得的误差数值；安装关系再变时应改回 `false`。
- D405 重新接入验证（2026-10-08）：MSI `lsusb` 枚举到 `8086:0b5b`；用项目的 `observe_global` 入口实拍两次，均返回 `218622270784`、1280×720、帧龄约 1.02 秒（上限 2 秒）、耗时约 2.9 秒、`motion_sent=false`；画面可见桌面、两台机械臂和夹爪。证据在两机 `work/global_camera/reconnect_20261008/`。未经执行器跑完整任务验证动作后自动观察。
- 说法统一为“不固定的全局相机”（2026-10-08）：模型提示词、MCP 指令、工具说明、8088 页面和文档中的“固定机位/固定相机”全部替换；相机角色标识由 `fixed_global` 改为 `movable_global`（8088、`global_camera.py`、MCP 服务、执行器校验同步）。采集方式、2 秒时效、身份核对和“未标定、仅供观察”的边界未变。旧任务记录里仍是 `fixed_global`，仅作历史展示。
- 删除网页 Codex 终端（2026-10-08，按用户要求）：移除 `tools/codex_terminal/`（8092 服务）、8089 的 `codex_terminal_proxy.py` 代理与路由、工作台“06 Codex执行器”标签，工作台恢复为五页。MSI 上已停止 8092 并删除其运行目录 `work/codex_terminal/`（含访问码）。重启后读回：页面只有 01～05 五个标签、不含 codex 字样，`/codex-terminal/` 与 `/api/codex-terminal/output` 返回 404，第 5 页 200。保留的部分：MCP 服务、命令行 `~/.local/bin/codex` 与 `codex_launcher.py`、专用 `~/.local/share/saydo-codex/` 配置；历史验证记录 `work/codex_terminal_validation/` 未动。
- 该配置文件的摘要参与场景快照与持物方案的绑定，修改后旧快照和旧方案自然失效；修改时 8089 处于停止状态，无夹持、无活动任务，无需额外处理。
- 两机其余配置、物品模型、夹法数据未改动；MSI 未跟踪目录 `config/grasp/observation_pose_backups/` 保留。本轮未发送任何机械臂或夹爪动作。

## 7. 已验证与未验证

- 已验证：七关节驱动；D435i 采集与手眼标定（用户确认实机抓取可靠）；FoundationPose GPU 环境；本地 Qwen 常驻与旧管线共存（整卡峰值约 6059 MiB，两个双物品场景约 17 秒完成且质量通过）；试管与试管架本地找框（小样本）；场景快照、通用持物运动、放置、调姿、固定抓放的规划与执行逻辑（模拟设备及离线 SDK 回放，2026-10-08 MSI 全量后端 307 项、执行器 131 项、MCP 32 项、预览 14 项通过）；任务执行器的重试、纠错、证据核验与停止链路。
- 未验证：试管入架的实机成功；夹指与架体、环境的碰撞；力控插入；释放后物品的独立位姿核验；新视角、遮挡、多管场景下的找框泛化。规划通过不等于实物入架通过。

## 8. 阻塞问题与下一步

1. 试管入架：2026-10-08 已首次规划出完整方案（见第 6 节），但尚未实机执行；此前历史目标找不到连续解的问题在试管架完整入镜、位姿估计改善后未再出现，是否稳定可复现未确认。执行前先解决夹指与顶板约 5 mm 余量的风险。
2. 夹指/架体碰撞模型缺失；试管架 180° 对称下的孔位编号需物理方向基准。
3. 全局相机支架可能移动：不同时刻的全局图片视角不一定相同。目前只改了称呼，没有在提示词里额外写“不要假设前后视角一致”之类的约束，如实测发现模型误判再补。
4. 已打开的命令行 Codex/MCP 会话仍持有旧说法和旧角色标识，需重新启动才刷新；第 5 页刷新后新任务即使用新说法。
5. `gpt-5.6-luna` 在服务商侧返回 503（上游负载），未恢复。

## 9. 注意事项

- 文档、网络、登录排查不授权机械臂运动；验证时不触发使能、点动、复位、抓取或夹爪动作。重启 8089/8091 前确认无活动任务、无夹持记录。
- 与物品绑定的数据只放 `work/object_models/<物品ID>/`；不新增跨物品集中配置。`config/` 与 `.env`（权限 600）只在各机本地维护，不提交、不覆盖。
- 重启 8089 必须使用上面的 `--agent-token-file` 新路径；旧进程命令行里的旧路径已失效。已有 MCP/Codex 会话在后端更新后需重新连接才能刷新工具说明。
- 软件停止不能替代现场急停；未知动作结果会阻断新任务，人工核对入口见 `tools/task_runner/README.md`。
- 本机与 MSI 之间不经 GitHub 同步时，可向 MSI 推送临时引用后在 MSI 上快进合并；`work/*.bundle` 是此前用 Git bundle 同步留下的文件。

## 10. 历史工作索引（每项一行，详情见上述旧版报告与对应 `work/*_validation/` 证据）

- 10-08 删除 D405 标定脚本、接口与数据；换装精度标记改为读取配置并按用户确认设为已验证；恢复 8089/8088；D405 以可变动支架重新接入并验证，说法统一为“不固定的全局相机”、角色标识改为 `movable_global`；删除第 6 页网页 Codex 终端与 8092 服务；`codex/task-runner` 合并回 `main` 并推送 GitHub。
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
- 09-29 新增 `observe_global` 全局相机观察工具（MCP 与执行器），随后全局相机由 D435i 换为 D405。
- 09-28 多构型联合路径搜索（`BEAM_IK_V1`）；保持姿态的搬运候选与多初值逆解；linear 采样 10° 关节跳变上限。
- 09-28 持物方案严格“先调姿、再保持朝向搬运”；泛化放置流程（上方 100 mm 预放置 → 直线下降 → 松爪 → 可选退出），取代孔位专用检查；特征统一为 `features.json`。
- 09-28 执行器：场景来源纠正、只读规划参数纠错、共享 5 次规划/3 次执行预算、可恢复失败返回 `MOTION_REPLAN_REQUIRED`、`get_task_context` 批量只读与上下文精简。
- 09-28 模型请求：连接未发出、响应超时、响应协议/读取错误三类有限重试（每轮最多 5 次，退避 1/2/4/8 秒）；新增请求阶段诊断。
- 09-28 动作前多目标场景快照（`capture_scene_snapshot`），同一冻结帧定位全部目标，持物后不再重跟踪参照物。
- 09-28 调姿分段统一为默认/最大 15°；模型升级为 6 系列，默认 `gpt-6-sol`。
- 09-24 通用持物运动工具（`plan_held_object_motion`、`plan_release`、`execute_plan` 等）；全流程统一运动速度 25%；闭合接触诊断记录。
- 09-24 工具原点改为 222.5 mm，每套夹法独立 TCP 偏移与最小离桌距离；横躺 5D 物品强制竖直抓取；桌面高度确认为 Base Z −0.02609 m。
- 09-24 40 孔试管架参数模型（272×112×85 mm，孔径 20 mm，孔距 25 mm）；LLM 找框与每物品提示词；切换物品弹窗重排；越界提示区分工作区与桌面。
- 09-24 第 6 页网页 Codex 终端上线（真实 PTY、只读沙箱、独立 CODEX_HOME）；已于 10-08 删除。
- 09-23 自然语言任务执行器上线（第 5 页），默认 execute 模式，含时间线、模型与强度选择、调姿工具；按模型启用 5D 位姿。
- 09-23 Codex 清理重装后恢复，MCP 独立环境与后端令牌迁至 `~/.local/share/saydo-mcp/`。
- 09-22 机械臂 MCP 工具与 `/api/agent/` 认证接口；夹爪力度与张开位置绑定到每套夹法。
- 09-21 三维夹法编辑器与多夹法库；模型绑定数据迁入各物品目录；桌面自适应放置及其后续调整（关节转移、取消边缘余量、松爪后沿工具轴后撤）；两种复位；调姿失败后保留夹持以便重试；三栏工作台与各类观察/旋转圆环控件。
- 09-20 同型号同安装相机复用手眼关系；抓取/调姿的正解回验与到位稳定窗口；指定姿态改用 movej_p；取消抓取后自动返回；手动复位目标更新。
