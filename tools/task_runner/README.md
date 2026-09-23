# Saydo 自然语言任务执行器

一个 Python 编排进程通过 Responses API 完成“观察 → 选择工具 → 等待结果 → 再判断”的任务循环。已有 MCP、视觉算法、夹法参数和机械臂后端保持独立；执行器复用 `tools/robot_agent/client.py` 的认证 HTTP 客户端，不通过 Codex，不执行模型生成的代码。

## 运行

需要 Python 3.11+。从项目根目录执行：

```sh
python3 -m venv work/task_runner_env
work/task_runner_env/bin/python -m pip install -r tools/task_runner/requirements.txt
cp .env.example .env
chmod 600 .env
# 在本机编辑 .env，填写模型密钥、后端地址及后端令牌路径。
work/task_runner_env/bin/python -m tools.task_runner.api
```

已有 `.env` 时保留并修改，不要用示例覆盖。`.env` 由 Git 忽略，安装及同步必须单独提供；密钥不进入网页或普通日志。模型密钥与机械臂 MCP 后端令牌分开。`.env` 支持 `KEY=value`、JSON 双引号或单引号，不执行 shell 展开。可用 `--env` 指定另一文件。

默认模型为 `gpt-6-luna`；仅在第一次模型决策收到明确“模型不可用”响应时回退到 `gpt-5.6-luna`。实际模型和回退事件写入运行记录。认证失败、限流、网络错误不改变模型；任务已开始调用工具后不切换模型。默认推理强度为 `medium`。服务商是否真正采用该强度无法仅凭请求字段证明。

`SAYDO_LLM_PROXY` 可选，只影响模型 HTTPS 请求。MSI 当前使用已有的本机代理 `http://127.0.0.1:7897`；模型代理与机械臂 HTTP 连接分开，后者不读取系统代理。该端口是当前机器配置，其他机器需按实际情况填写。

`SAYDO_RUNNER_MODE` 支持：

- `read_only`：只向模型提供观察、查询和结束工具。
- `preview`：另允许目标选择、初始化及方案预览；不把真实动作替换成伪造成功的预览。
- `execute`：允许抓取、放置、复位和模型主动停止；仍要求用户任务包含对应动作意图，且后端自身也允许执行。

默认 `preview`。切换模式须编辑本机 `.env` 并在任务结束后重启执行器；网页和模型不能更改模式。服务启动不会自动提交任务或发送动作。独立任务页监听 `http://127.0.0.1:8091`，不重启现有 8089 服务。Mac 上访问 MSI：

```sh
ssh -N -L 18091:127.0.0.1:8091 msi
```

浏览器打开 `http://127.0.0.1:18091`。首次版本是独立任务页面，现有视觉工作台仍在原入口。

## 模块与接口

- `llm_client.py`：仅发送原生函数定义和本次任务历史，保留模型可回传的 output；图片作为 `input_image`，不塞入普通 Base64 文本。禁用服务端存储、流式和并行工具调用。
- `tool_registry.py`：15 个现有业务工具加本地 `finish_task`，严格校验类型和多余字段。原 `get_task` 留在程序内部；MCP 的16个工具不变。
- `robot_adapter.py`：复用原认证客户端，等待模型加载、跟踪和后端任务。发生提交超时时按原编号查询，不自动重发，不替换放置的夹持或配置版本。
- `runner.py`：单任务、串行工具，最多20轮、整任务600秒；动作等待120秒、跟踪等待45秒、轮询间隔0.5秒。限制本次上下文为2 MiB，不加入长期记忆或自动续跑。相同动作已成功时禁止换调用编号重复执行。
- `store.py`：`work/task_runner/runs.sqlite` 保存任务、调用映射、确定参数、阶段、结果、耗时和回退事件；发起请求前写入请求编号。完整原始运动报告仍保存在已有后端。
- `api.py`：任务页和同源 API；只绑定回环地址，拒绝异常 Host、跨站 POST 和非 JSON 写入。状态目录设备锁阻止重复启动。

```text
POST /api/task-runs                    {"instruction":"..."}
GET  /api/task-runs                    最近20次任务与执行模式
GET  /api/task-runs/{run_id}            阶段、工具调用与最终结果
POST /api/task-runs/{run_id}/cancel     {}
```

取消按钮明确表示“停止机械臂并取消”：绕过 LLM，禁止后续调用，立即发软件停止；若有已发出的写请求尚未返回，等待其结束后再次确认停止，防止迟到的提交越过第一次停止。**即使只读/预览任务，用户点击此按钮也会发软件停止**，不会松爪。正常只读任务超时或进程退出不会自动向空闲机械臂发停止。

`finish_task` 的 `succeeded` 必须引用本轮全部真实成功动作，并在结束前核对后端状态；`answered` 只表示已回答问题或解释预览，不表示实际操作完成。页面显示“已核验的动作”，与模型给出的说明区分。控制器完成仍不能独立证明接触桌面、无滑动或长期放置稳定。

## 故障与恢复

后端未确认的动作、未确认停止会阻断新任务。重启不续跑；尚在提交/执行中的真实动作记录会标为结果未知。只读任务中断不形成机械臂动作未知记录。

操作员实际核对现场及原任务记录后，先停止执行器，再显式运行只读核对工具：

```sh
work/task_runner_env/bin/python -m tools.task_runner.reconcile \
  --run-id <运行编号> --confirm-scene-inspected
```

该工具要求后端空闲且停止已确认；查询已记录的任务编号，不发送动作、不恢复旧任务。仅当操作员确定原请求未提交且后端查无该编号时，额外使用 `--confirm-request-not-submitted`。核对后重新启动执行器，再提交新指令。不要直接删除数据库来绕过未知结果。

## 验证

```sh
# 不连接设备的单元/集成测试
work/task_runner_env/bin/python -m unittest discover -s tools/task_runner -t . -p 'test_*.py' -v
# 使用 .env 密钥；仅发送合成测试图片和无害的假工具，不连接机械臂
work/task_runner_env/bin/python -m tools.task_runner.probe
```

`probe` 检查严格函数调用、回传完整模型输出、工具结果续答、图片输入及初始模型回退，摘要保存到 `work/task_runner_validation/provider_probe.json`。先完成这些检查，再做真实查询和方案预览；真实抓放需另外进行现场验收。
