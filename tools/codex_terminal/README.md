# Codex 执行器（工作台第 6 页）

8089 工作台的「06 Codex执行器」提供真实交互终端。输入文本直接交给 MSI 已安装的任务专用 Codex，模型输出、工具调用过程和交互提示按终端原样显示。第五页 SayDo 任务执行器继续独立使用。

## MSI 启动

将本模块及 `tools/foundationpose/` 的配套修改部署至同一份项目后，在 MSI 项目根目录使用 Python 3.11 或更新版本运行：

```sh
work/task_runner_env/bin/python -m tools.codex_terminal.api
```

这是常驻前台服务，默认只监听 `127.0.0.1:8092`。8089 的固定代理转发 `/codex-terminal/` 页面及 `/api/codex-terminal/` 接口。若修改 `--port`，需同时调整代理端口；正常部署使用默认端口即可。8089 后端需加载此次修改，新服务可单独停止或启动，不需要为每次连接重启8089。

首次启动会生成 `work/codex_terminal/access_token`（权限600）。在 MSI 自己的终端读取该文件，将访问码填入第六页，点击「启动 / 连接」。不要把它放入 URL、源码、日志或共享截图。它与 LLM API key、MCP 后端令牌各自独立；API key 仍由原启动器从 `.env` 加载。

访问码只存在浏览器当前页面内存；刷新后需重新输入，断线后可重连已有会话。页面不将它保存到浏览器存储。8089 若使用普通 HTTP，应通过可信网络或 SSH 转发访问，避免访问码被网络窃听；本服务不配置公网入口。

## 已部署服务与维护

MSI当前部署使用项目内的 `work/task_runner_env/bin/python`，8092仅监听回环地址；8089加载了第六页代理和现有MCP认证路由。进程编号记录在 `work/codex_terminal/server.pid`，本次启动日志和验证记录在 `work/codex_terminal_validation/`。现有服务已运行，不要重复启动。

需要在结束所有终端会话后手动重新启动时，在MSI项目根目录运行：

```sh
nohup work/task_runner_env/bin/python -m tools.codex_terminal.api > work/codex_terminal/server.log 2>&1 &
echo $! > work/codex_terminal/server.pid
```

停止前先确认没有该会话发起的机器人任务，再核对PID对应 `tools.codex_terminal.api`，发送SIGTERM；服务会结束其CLI会话。服务不会自动恢复会话或重放任务，也未配置系统开机自启。不要为重连终端重启8089或终止视觉跟踪。

访问码可在自己终端中通过 `ssh msi 'cat /home/pc/saydo/work/codex_terminal/access_token'` 读取，填入第六页后点击“启动 / 连接”。不要把访问码粘贴到任务对话或截图中。

## 使用及运行前提

- 仅点击「启动 / 连接」会启动 Codex；打开网页或切换页签不会启动程序或任务。
- 在黑色终端内直接输入任务，Enter提交；支持中文、方向键、滚动和终端尺寸同步。
- 同一服务同时保留一个会话，重连回到该会话；多个已授权页面共享输入输出，不是独立用户会话。
- 「中断 Ctrl+C」向终端发送中断键；「清屏」仅清理本地终端显示；「结束会话」结束该 CLI 进程组。切页、刷新及断网不会自动结束 CLI。服务退出则会清理它启动的 CLI。
- 终端关闭或中断不等于机械臂停止。需要停止运动时使用工作台右侧「停止运动」。不要同时在两个执行器提交机械臂任务。
- 输出在服务内存保留最近1MiB；超过后重连显示保留部分，页面提示截断。服务重启不恢复终端会话，长期对话历史由原 Codex 管理。

服务固定调用 `tools/robot_agent/codex_launcher.py`，不接受网页传来的命令、程序参数或工作目录。沿用现有官方 CLI 安装、专用 `CODEX_HOME`、隔离工作目录、模型配置、技能及指令隔离；不改写它们。首次启动器检查失败时，错误会出现在终端中。

要执行实际机器人任务，现有 Codex MCP 配置、后端令牌、8089 的 `/api/agent/` 认证路由及硬件运行前提必须已就绪。该页面不会替用户打开这些权限或启用机械臂。

## 验证

```sh
python3 -m unittest tools.codex_terminal.test_terminal -v
node tools/foundationpose/test_workspace_tabs.cjs
node --check tools/codex_terminal/app.js
```

后端8项测试使用无硬件Python程序，验证控制终端及前台进程组、真实伪终端输入、中文、输入重试去重、缩放、重连、退出与子进程清理、输出截断、访问码权限和HTTP代理白名单。前端终端依赖已固定版本并随仓库提供，无须在 MSI 安装 Node/npm，也不从 CDN 加载脚本。
