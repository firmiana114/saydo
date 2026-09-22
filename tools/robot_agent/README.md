# Saydo 机械臂 MCP 工具

通过标准输入/输出 MCP 将隔离的 `saydo-agent` 连接到现有 Saydo 后端。MCP 进程只调用固定的 `/api/agent/` 接口；实际动作仍由 `GraspController`、原设备锁和运动检查执行，不接受任意机器人指令、文件路径或配置修改。

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
| `reset_arm` | `observation` 返回记录的观测姿态；`zero` 返回七关节零位（不是编码器标定）；保持夹爪开合 |
| `get_task` | 按请求编号查询持久化任务，直到 `terminal=true` |
| `stop_robot` | 取消任务，并发送机械臂和夹爪软件停止，即使当前没有 Saydo 任务；不松爪；分别确认指令应答和采样停稳 |

停止工具是软件停止，不替代现场急停。停止后至少三次关节样本、覆盖 100 毫秒、每轴变化不超过 0.03°，且夹爪速度为零、当前动作任务退出，才报告 `stationary_observed=true`。未确认时锁住新的动作入口，返回 `STOP_UNCONFIRMED`，需现场核对；停止工具本身可以再次调用。停止不会自动恢复旧的夹持记录。

`start_tracking` 的框为当前 640×360 图像的 `[x0,y0,x1,y1]`。先用 `select_object` 后等待 `get_status.tracking.model_ready`；自动检测只识别类别，必须结合图像确认实例，不能将另一只杯子当作目标模型。当前未实现多实例长期身份跟踪。

## 任务和失败语义

每次预览或动作必须提供唯一 `request_id`（8–96 位字母、数字、下划线或连字符）。同编号同参数返回原任务，不重复执行；同编号不同参数拒绝。HTTP 断连、超时或损坏响应不自动重试；先以该编号 `get_task`，需要重发也必须使用同一个编号及原参数。

请求先存入 `work/agent/tasks.sqlite`，再进入原控制器。状态回调在控制器锁内保存结果，避免后来的网页任务覆盖旧结果。服务重启把未结束记录标为 `interrupted` / `outcome_unknown=true`，不恢复或重放动作。状态记录故障明确报 `TASK_STATE_UNCERTAIN`，禁止以新编号绕过。

`terminal=true` 可能表示完成、预览完成、拒绝、失败或取消，必须查看 `state` 和 `result/error`。控制器动作完成不等于已经独立证明物品放稳、接触桌面或未发生滑动；Agent 应再次观察并区分已验证事实与未知结果。端点运动学检查仍不等于完整环境碰撞检查。

## 安装与启动

MCP 使用独立 Python 3.11+ 环境，不修改 FoundationPose 环境：

```sh
python3 -m venv ~/.local/share/saydo-agent/mcp-venv
~/.local/share/saydo-agent/mcp-venv/bin/python -m pip install -r tools/robot_agent/requirements.txt
python3 tools/robot_agent/install.py --base-url http://127.0.0.1:8089
```

安装器只更新独立 Agent 配置的 `mcp_servers.saydo_robot`，保留 GPT-6 Astra/high、只读沙箱、禁用 Skills/旧指令等设置。后端令牌独立于模型 API Key，保存为权限 600 的 `credentials/robot-backend-token`。重复安装保留令牌；MCP 不跟随 HTTP 重定向，不通过环境代理发送后端凭据。

后端需显式启用 Agent 路由（按实际监听地址调整，安装器的 URL 必须一致）：

```sh
source tools/foundationpose/runtime_env.sh
python tools/foundationpose/live_tracker.py --host 127.0.0.1 --port 8089 --auto-start \
  --mesh work/object_models/ob_yibao_350ml/model/model.obj \
  --grasp-mode execute \
  --agent-token-file ~/.local/share/saydo-agent/codex-home/credentials/robot-backend-token
```

启动、自动识别和连接 MCP 不发送机械臂动作。`--grasp-mode preview` 可关闭抓取/放置/复位执行，预览和观察仍可用；停止工具仍可发停止命令。不要在活动任务或需保留夹持记录时重启后端。

重新启动 `saydo-agent` 后载入工具。读工具沿用原生读取权限，写工具按 Codex `writes` 审批策略处理；停止工具可直接调用，避免在停止请求上等待额外审批。未开放改标定、改运动参数、任意关节运动、清除错误或重启服务的 Agent 工具。

认证只针对新增的 `/api/agent/` 路由；原网页接口沿用既有部署方式和访问边界，本次没有将其改造成新的认证体系。

## 验证

```sh
SAYDO_IK_CHECKER="$PWD/work/grasp/rm75_ik_check" work/foundationpose_env/bin/python -m unittest discover -s tools/foundationpose -p 'test_*.py' -q
~/.local/share/saydo-agent/mcp-venv/bin/python -m unittest discover -s tools/robot_agent -p 'test_*.py' -v
```

动作测试使用模拟控制器；覆盖实际 HTTP 认证、任务去重/冲突/重启、物品和夹持绑定、停止失败锁定、最新画面和过期位姿。MCP SDK 测试覆盖原生图片块、16 个工具的模式和参数、任务查询及错误语义。实机联调仅调用观察/状态工具，不以真实抓取、放置、复位或停止作为自动验收。

2026-09-22 验证：175 项后端测试、11 项 MCP/客户端/安装器测试和 7 组前端检查通过。真实相机和机械臂读取成功；GPT-6 Astra/high 通过实际 MCP 工具读取状态并识别相机朝向天花板。当前无目标物品，位姿接口明确返回 `POSE_NOT_READY`，不把不存在的位姿交给 Agent。有效目标位姿和真实动作效果需在适当场景下单独验收。完整结果见根目录 `HANDOFF_REPORT.md`。
