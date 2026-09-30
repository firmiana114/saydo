# 双相机只读监控面板

并排显示 D435i 眼在手相机和 D405 固定机位的实时彩色画面。支持同时开始、单路/全部释放、独立帧率与状态、分别保存当前 JPEG 截图。不访问机械臂或夹爪写入接口，不录制视频。

## 访问与启动

- MSI 本机：`http://127.0.0.1:8088`
- 当前局域网：`http://192.168.101.65:8088`（地址变化时调整）

默认监听 `0.0.0.0:8088`，沿用可信局域网部署，无登录认证。单独访问页面和状态不会打开相机；点击“开始双路预览”或单路“开始预览”后采集。

已有 `.venv` 需包含 `pyrealsense2`、`opencv-python-headless`、`numpy`，系统需提供 `fuser`。在项目根目录执行：

```sh
./tools/camera_monitor/run.sh --port 8088 \
  --tracker-url http://192.168.101.65:8089
```

`--tracker-url` 必须是当前 8089 实际监听的地址。服务通过本机 owner-only 的 `~/.local/share/saydo-mcp/robot-backend-token` 认证，只读取 `/api/agent/frame`；令牌不进入网页。可用 `--token-file` 指定已有令牌文件。后端无法连接时才尝试直接打开空闲 D435i；认证、身份、时效或其他后端拒绝不会触发设备抢占。

不使用 8089 时可省略 `--tracker-url`，直接采集两台空闲设备。序列号默认读取机器本地 `config/cameras/global.json` 的 `eye_in_hand_serial` 和 D405 的 `serial`；可用 `--eye-in-hand-serial`、`--fixed-serial` 覆盖。没有配置时仅选择唯一匹配型号的设备，多个同型号设备会拒绝猜测。

默认直接采集 1280×720、15 fps；共享 8089 时显示其实际原始图像尺寸（当前 640×360），不冒充原生全分辨率。状态栏显示序列号、机位和画面来源。`--width`、`--height`、`--fps` 控制本服务直接采集的流。

## 相机共用与释放

- 每台相机独立管理采集线程，最多四个观看连接共用一路采集。单路异常不关闭另一台。
- D435i 已由 8089 使用时读取其最新原始画面，不开启第二条 SDK pipeline，也不暂停或重启 8089。释放共享预览只关闭本页连接，8089 继续采集。
- D405 预览与现有全局观察共用同一设备锁。预览运行时，`observe_global` 的短进程从监控服务获取请求之后的新帧，保留身份、时间和“仅观察、未经基座标定”的约束；没有活动预览时沿用原独立采集流程。已有执行器无需重启。
- 最后一个观看者关闭后自动释放本服务持有的设备；单路/全部释放会结束对应观看连接。独立采集、标定前必须释放需要的相机。
- 本服务不抢占其他程序、不复位设备。两路图像独立采集，不具备同步标定样本或深度采集能力。

## HTTP 接口

| 接口 | 行为 |
| --- | --- |
| `GET /api/status` | 聚合状态及 `cameras.d435i`、`cameras.d405`；保留旧顶层字段 |
| `GET /stream/d435i`、`GET /stream/d405` | 开始/加入对应 MJPEG 预览 |
| `GET /snapshot/d435i.jpg`、`GET /snapshot/d405.jpg` | 当前新鲜 JPEG，无活动预览或图像过期时 409 |
| `GET /api/cameras/<id>/status` | 单路状态 |
| `POST /api/cameras/<id>/pause` | 释放单路预览 |
| `GET /api/cameras/<id>/frame` | 等待请求之后的新帧并返回带身份/时间的 JPEG JSON；不会启动设备 |
| `GET /api/cameras/<id>/calibration-frame` | 同样等待新帧；直接采集返回编码前PNG及实际内参，共享8089时如实返回JPEG与来源标记 |
| `POST /api/pause` | 释放两路，兼容原标定及 8089 启动协调 |

旧 `GET /stream`、`GET /snapshot.jpg` 仍指向 D435i。前台 Ctrl-C 或服务 SIGTERM 只结束本服务采集与连接，不停止机器人、不关闭 8089。

双相机标定的单组脚本要求两台均为原生1280×720 PNG：采集阶段关闭8089，D435i与D405均由本服务直接采集，保存实际SDK内参/畸变及相机时间信息。参见 [单组采集说明](../handeye/DUAL_CAMERA_CAPTURE.md)。这不是硬件同步或标定求解。

后台实例的 PID 和日志沿用 `work/camera_monitor/server.pid`、`work/camera_monitor/server.log`；不配置开机启动。

## 验证

```sh
python3 -m unittest tools.camera_monitor.test_monitor -v
PYTHONPATH=tools/robot_agent python3 -m unittest discover -s tools/robot_agent -p test_global_camera.py -v
node tools/camera_monitor/test_panel.cjs
```

测试覆盖双路独立生命周期、旧接口、相机身份/时效、共享原始图像、全局观察新帧、最后观看者释放、单路故障隔离与跨站请求拒绝。实机验证结果见项目 `HANDOFF_REPORT.md`。
