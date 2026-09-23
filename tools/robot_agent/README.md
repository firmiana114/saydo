# Saydo 机械臂 MCP 工具

通过标准输入/输出 MCP 将客户端连接到现有 Saydo 后端。MCP 进程只调用固定的 `/api/agent/` 接口；实际动作仍由 `GraspController`、原设备锁和运动检查执行，不接受任意机器人指令、文件路径或配置修改。

项目另提供直接调用 LLM API 的 [任务执行器](../task_runner/README.md)，复用本目录的认证 HTTP 客户端。原 MCP 服务及18个工具保持可用，执行器无需通过 MCP 或 Codex 启动。

## 工具

| 工具 | 行为 |
| --- | --- |
| `get_status` | 跟踪准备状态、执行模式、夹持记录/版本、放置参数、活动任务编号 |
| `get_camera_frame` | 最新原始彩色画面，附采图时间、序号和尺寸；拒绝超过一秒的画面；返回原生 MCP 图片块 |
| `get_object_pose` | 稳定且新鲜的物品位姿；**相机光学坐标系、米制**，不是基座坐标；不满足原抓取观测条件时明确失败 |
| `get_robot_state` | 实际七关节角度（度）、TCP 位置（米）/RPY（弧度）、工具/工作坐标、错误与夹爪反馈；不使能、不清错 |
| `list_objects` / `get_grasp_profiles` | 模型目录及已保存夹法；模型 ID 不等于场景中独立实例 ID |
| `select_object` / `start_tracking` / `choose_tracking_target` | 选择模型、自动或框选初始化、处理多个候选；无机器人动作，忙时拒绝 |
| `preview_grasp` / `preview_place` | 使用原检查生成方案，不运动；返回异步任务编号 |
| `grasp` | 按当前已保存夹法抓取并抬升；请求目标必须与执行时新鲜观测中的物品一致 |
| `place` | 使用已保存的桌面矩形和实际持物朝向，执行转移、下降、释放、后撤、退出；校验夹持/朝向/放置配置版本 |
| `preview_reorient` / `reorient` | 已夹持物品的绝对朝向预览/执行；固定TCP终点或自适应移位，继续夹持；成功执行后返回新的朝向版本 |
| `reset_arm` | `observation` 返回记录的观测姿态；`zero` 返回七关节零位（不是编码器标定）；保持夹爪开合 |
| `get_task` | 按请求编号查询持久化任务，直到 `terminal=true` |
| `stop_robot` | 取消任务，并发送机械臂和夹爪软件停止，即使当前没有 Saydo 任务；不松爪；分别确认指令应答和采样停稳 |

停止工具是软件停止，不替代现场急停。停止后至少三次关节样本、覆盖 100 毫秒、每轴变化不超过 0.03°，且夹爪速度为零、当前动作任务退出，才报告 `stationary_observed=true`。未确认时锁住新的动作入口，返回 `STOP_UNCONFIRMED`，需现场核对；停止工具本身可以再次调用。停止不会自动恢复旧的夹持记录。

`start_tracking` 的框为当前 640×360 图像的 `[x0,y0,x1,y1]`。先用 `select_object` 后等待 `get_status.tracking.model_ready`；自动检测只识别类别，必须结合图像确认实例，不能将另一只杯子当作目标模型。当前未实现多实例长期身份跟踪。

## 姿态调整参数

`preview_reorient` 和 `reorient` 均需要6项参数：`object_id`、`hold_id`、`orientation_version`、`target_rpy_deg`、`position_mode`、`request_id`。前三项从最新 `get_status.grasp.held_object` 读取；`target_rpy_deg=[X,Y,Z]` 为物品模型相对零偏移 Base 基座的绝对朝向，单位度，每项在−180到180之间，转换顺序为 `Rz(Z) @ Ry(Y) @ Rx(X)`。不是相对增量，也不是TCP朝向，不能默认零角度等于物品直立。

`position_mode` 必须为 `fixed` 或 `adaptive`：前者只固定TCP最终位置，不保证中途位置恒定；后者允许现有后端搜索移位。后端从匹配的夹持记录取得模型摘要，继续核对实际模型、工具、夹持、工作区/桌面、运动学与到位反馈，不接受速度、力度或开度覆盖。

工具返回异步任务编号，使用 `get_task` 等待。预览终态为 `reorient_preview`，不修改夹持朝向版本；成功执行为 `completed`，保持同一 `hold_id`，`orientation_version` 加1。结果包含 `reorientation_plan` 与最新 `held_object.current_object_rotation`，后者是基座坐标系中的物品旋转矩阵。后续放置/调整使用新版本；旧请求重复提交仍按原 `request_id` 返回原任务，不会再次运动。实际动作失败后不得自动重复尝试。

## 夹法参数输出

`get_grasp_profiles` 返回每条夹法的 `effective_parameters`：`force_target_raw`、`force_limit_raw`、`open_position`、`pregrasp_distance_m`、`lift_height_m`、`arm_speed_percent` 及单位。力度和开度来自该夹法的保存值，不从全局夹爪设置继承；它们不是当前硬件寄存器读回值。缺少绑定值时返回 `parameter_error`，执行拒绝该候选。

`get_status` 的 `grasp.grasp_configuration` 和 `get_robot_state` 的 `controller.grasp_configuration` 提供当前物品的选择方式及各夹法参数摘要。Agent 输出不包含旧全局力度/开度字段。抓取任务的 `result.plan.effective_parameters` 记录实际选用的参数；历史任务保留当时的值。

当前初始化规则：纸杯力度 10，其他物品 75；马克杯抓取张开位置 200，其他物品 1000。前端“记录夹法”表单可填写新夹法参数，保存不发送夹爪动作。MCP 抓取入参仍是物品编号和请求编号，不允许单次请求暗中覆盖夹法参数。

## 任务和失败语义

每次预览或动作必须提供唯一 `request_id`（8–96 位字母、数字、下划线或连字符）。同编号同参数返回原任务，不重复执行；同编号不同参数拒绝。HTTP 断连、超时或损坏响应不自动重试；先以该编号 `get_task`，需要重发也必须使用同一个编号及原参数。

请求先存入 `work/agent/tasks.sqlite`，再进入原控制器。状态回调在控制器锁内保存结果，避免后来的网页任务覆盖旧结果。服务重启把未结束记录标为 `interrupted` / `outcome_unknown=true`，不恢复或重放动作。状态记录故障明确报 `TASK_STATE_UNCERTAIN`，禁止以新编号绕过。

`terminal=true` 可能表示完成、预览完成、拒绝、失败或取消，必须查看 `state` 和 `result/error`。控制器动作完成不等于已经独立证明物品放稳、接触桌面或未发生滑动；Agent 应再次观察并区分已验证事实与未知结果。端点运动学检查仍不等于完整环境碰撞检查。

## 安装与启动

MCP 使用独立 Python 3.11+ 环境，不修改 FoundationPose 环境：

```sh
python3 -m venv ~/.local/share/saydo-mcp/venv
~/.local/share/saydo-mcp/venv/bin/python -m pip install -r tools/robot_agent/requirements.txt
python3 tools/robot_agent/install.py --base-url http://127.0.0.1:8089
```

安装器默认在 `~/.codex/config.toml` 中登记 `mcp_servers.saydo_robot`；不存在配置时创建仅包含 MCP 的文件，不设置模型、模型服务商、推理强度、全局审批或 Skills。已有的其他设置保持原样。后端令牌独立于模型 API Key，保存为权限 600 的 `~/.local/share/saydo-mcp/robot-backend-token`，与客户端配置分开维护。重复安装保留令牌；MCP 不跟随 HTTP 重定向，不通过环境代理发送后端凭据。可用 `--agent-home`、`--python`、`--token-file` 指定路径。

使用其他 MCP 客户端时，启动命令为 `~/.local/share/saydo-mcp/venv/bin/python`，参数为项目内 `tools/robot_agent/server.py` 的绝对路径、`--base-url`、实际后端 URL、`--token-file` 和上述令牌的绝对路径。工具本身不依赖 Codex 的模型配置或登录方式。

后端需显式启用 Agent 路由（按实际监听地址调整，安装器的 URL 必须一致）：

```sh
source tools/foundationpose/runtime_env.sh
python tools/foundationpose/live_tracker.py --host 127.0.0.1 --port 8089 --auto-start \
  --mesh work/object_models/ob_yibao_350ml/model/model.obj \
  --grasp-mode execute \
  --agent-token-file ~/.local/share/saydo-mcp/robot-backend-token
```

启动、自动识别和连接 MCP 不发送机械臂动作。`--grasp-mode preview` 可关闭抓取/姿态调整/放置/复位执行，预览和观察仍可用；停止工具仍可发停止命令。不要在活动任务或需保留夹持记录时重启后端。

启动 `codex` 后载入工具；客户端登录需另行完成。旧的 `saydo-agent` 包装入口已移除。安装器仅为该 MCP 设置 `writes` 工具审批策略；停止工具可直接调用，避免在停止请求上等待额外审批。未开放改标定、改运动参数、任意关节运动、清除错误或重启服务的 Agent 工具。

认证只针对新增的 `/api/agent/` 路由；原网页接口沿用既有部署方式和访问边界，本次没有将其改造成新的认证体系。

## 验证

```sh
SAYDO_IK_CHECKER="$PWD/work/grasp/rm75_ik_check" work/foundationpose_env/bin/python -m unittest discover -s tools/foundationpose -p 'test_*.py' -q
~/.local/share/saydo-mcp/venv/bin/python -m unittest discover -s tools/robot_agent -p 'test_*.py' -v
```

动作测试使用模拟控制器；覆盖实际 HTTP 认证、任务去重/冲突/重启、物品和夹持绑定、停止失败锁定、最新画面和过期位姿。MCP SDK 测试覆盖原生图片块、18 个工具的模式和参数、任务查询及错误语义。实机联调仅调用观察/状态工具，不以真实抓取、放置、复位或停止作为自动验收。

2026-09-22 验证：175 项后端测试、11 项 MCP/客户端/安装器测试和 7 组前端检查通过。真实相机和机械臂读取成功；GPT-6 Astra/high 通过实际 MCP 工具读取状态并识别相机朝向天花板。当前无目标物品，位姿接口明确返回 `POSE_NOT_READY`，不把不存在的位姿交给 Agent。有效目标位姿和真实动作效果需在适当场景下单独验收。完整结果见根目录 `HANDOFF_REPORT.md`。
