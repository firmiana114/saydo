# SayDo 当前维护交接

## 项目与入口

- 自然语言机械臂项目；当前 Python 实时视觉及抓取入口在 `tools/foundationpose/`。
- `live_tracker.py` 采集 D435I 并调用 FoundationPose，8089 提供网页与状态；`grasp_control.py` 执行抓取检查和动作，`grasp_robot.py` 与控制器通信。
- 启动环境：`source tools/foundationpose/runtime_env.sh`；启动 `python tools/foundationpose/live_tracker.py --host 192.168.101.65 --port 8089 --auto-start`。启动不会自动抓取。
- 几何配置在 `config/calibration/`，抓取参数在 `config/grasp/trial.json`；模型、原始标定和日志在忽略版本管理的 `work/`，需另行保留。
- 更多硬件和历史验证见 `ros2_rm_ws/HANDOFF_REPORT.md`。

## 2026-09-20 同型号同安装相机复用

- 原标定相机 `135122077750`，当前连接相机 `405622073572`，均为 D435I。旧服务写死序列号导致无设备错误；指定当前相机后恢复预览，但抓取被序列号及旧内参检查阻塞。
- 用户确认型号和安装位置不变，希望替换相机无需重新标定。新增 `camera_reuse.yaml` 绑定型号、安装编号及旧标定来源，明确允许复用机械安装变换。
- 实时入口默认自动选择唯一 D435I；多台必须指定。采集元数据随帧和位姿传递；抓取与夹法记录检查当前相机身份及其自身内参。
- 原 `handeye.yaml`、原始采集结果及工具配置保持不变；任务记录 `camera_calibration`，相机连接事件记录实际设备参数。
- 策略仅适用于实时/抓取入口，离线采集和手眼求解规则不变。

## 验证与边界

- `work/foundationpose_env/bin/python -m unittest discover -s tools/foundationpose -p 'test_*.py' -q`：58 项通过，包含新增 4 项替换相机检查测试。
- 覆盖同型号允许复用、未授权/型号不符/来源变化拒绝、真实内参匹配、旧内参及错配观测拒绝、唯一相机选择与多台歧义。
- 已重启本机 8089 服务，不带 `--serial` 自动识别当前 D435I；本轮运行记录在 `work/foundationpose_live/20260920_094335_112155/`。
- 实际跟踪观测通过相机兼容与内参检查；调用 `/api/grasp/preview` 返回 `state=preview`、`error=null`、`execution_blockers=[]`，生成包含同型号同安装复用依据的方案，未发送运动。预览不代表完整轨迹或真实抓取精度通过。
- 软件无法测量换装偏差；新相机抓取精度未实测。复用前提为用户确认的安装位置及朝向保持不变。
- 源数据哈希、机器人/工具状态、视觉质量和已有动作检查保留。本轮不发送机械臂或夹爪运动。

## 后续与注意事项

- 安装关系实际变化时重新核对手眼关系；仅序列号变化不再阻塞。
- `camera_reuse.yaml` 设置 `enabled: false` 或移除后恢复跨序列号拒绝。
- 保留原有未提交的 `config/grasp/trial.json`、`tools/object_scan/capture_view.py` 和未跟踪 `tools/camera_monitor/`，不纳入本轮提交。
