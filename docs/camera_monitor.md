# 相机只读监控面板

并排显示 1 臂手上的 D435i、D405 不固定的全局相机，以及配置了序列号时 2 臂手上的 D435i 的实时彩色画面。支持同时开始、单路/全部释放、独立帧率与状态、分别保存当前 JPEG 截图。不访问机械臂或夹爪写入接口，不录制视频。

## 访问与启动

- MSI 本机：`http://127.0.0.1:8088`
- 当前局域网：`http://192.168.101.65:8088`（地址变化时调整）

默认监听 `0.0.0.0:8088`，沿用可信局域网部署，无登录认证。单独访问页面和状态不会打开相机；点击“开始全部预览”或单路“开始预览”后采集。

已有 `.venv` 需包含 `pyrealsense2`、`opencv-python-headless`、`numpy`，系统需提供 `fuser`。在项目根目录执行：

```sh
./scripts/run_camera_monitor.sh --port 8088 \
  --tracker-url http://192.168.101.65:8089
```

`--tracker-url` 必须是当前 8089 实际监听的地址。服务通过本机 owner-only 的 `~/.local/share/saydo-mcp/robot-backend-token` 认证，只读取 `/api/agent/frame`；令牌不进入网页。可用 `--token-file` 指定已有令牌文件。后端无法连接时才尝试直接打开空闲 D435i；认证、身份、时效或其他后端拒绝不会触发设备抢占。

不使用 8089 时可省略 `--tracker-url`，直接采集两台空闲设备。序列号默认读取机器本地 `config/cameras/global.json` 的 `eye_in_hand_serial` 和 D405 的 `serial`；可用 `--eye-in-hand-serial`、`--fixed-serial` 覆盖。没有配置时仅选择唯一匹配型号的设备，多个同型号设备会拒绝猜测。

2 臂手上的相机（2026-10-09 起）：在 `config/cameras/global.json` 里写 `arm2_eye_in_hand_serial`，或启动时加 `--arm2-serial`，页面才出现“2 臂 · D435i 眼在手”这一路（键为 `d435i_arm2`）。8089 接管了 2 臂相机时（2 臂的抓取配置里写了 `camera_serial`）共享 8089 的画面，否则由本服务直接采集。提供这一路时必须同时写明 1 臂相机的序列号，因为两台 D435i 同时连着，不写就无法区分。

默认直接采集 1280×720、15 fps；共享 8089 时显示其实际原始图像尺寸（当前 640×360），不冒充原生全分辨率。状态栏显示序列号、机位和画面来源。`--width`、`--height`、`--fps` 控制本服务直接采集的流。

## 相机共用与释放

- 每台相机独立管理采集线程，最多四个观看连接共用一路采集。单路异常不关闭另一台。
- D435i 已由 8089 使用时读取其最新原始画面，不开启第二条 SDK pipeline，也不暂停或重启 8089。释放共享预览只关闭本页连接，8089 继续采集。
- D405 预览与现有全局观察共用同一设备锁。预览运行时，`observe_global` 的短进程从监控服务获取请求之后的新帧，保留身份、时间和“仅观察、未经基座标定”的约束；没有活动预览时沿用原独立采集流程。已有执行器无需重启。
- 最后一个观看者关闭后自动释放本服务持有的设备；单路/全部释放会结束对应观看连接。
- **借用与归还**：手眼采集程序需要某台相机时，向本服务“借用”：本服务停止这一路并在一段时间内（最长 60 秒）拒绝新的预览连接，采集结束后采集程序归还。页面看到相机被借走时收起画面并说明原因，归还后自动重连，不用手动释放或重新开始。为什么要拒绝连接而不只是暂停：页面会自动重连，只暂停的话重连会在采集开始前把相机又占回去。只有“借走再归还”才自动重连；出错或被别处释放后仍由用户决定是否重新开始。
- **程序向本服务要画面（2026-10-10 起）**：一台相机同一时刻只能被一个进程打开。程序如果自己去打开相机，预览就得让出来，操作者摆放物品时看不到画面。所以需要全局相机画面的程序改为向本服务取图：`GET /api/cameras/<id>/frame?keep_s=20&format=png`。带 `keep_s` 时，相机没开就由本服务打开，并至少再保持采集这么多秒（上限 60）；程序每取一次续一次，退出后不用归还，期限一到、又没有人看预览，相机自行释放。`format=png` 给出无损图像（每个像素与相机给出的相同），应答里的 `intrinsics` 是这路彩色画面的内参与畸变参数。预览和程序共用同一次打开的相机，互不影响。全局相机的快速标定（`saydo/calibration/global_camera.py`）走的就是这条路。与“借用”的区别：借用是把相机整台让给另一个进程，期间没有预览；取图是相机留在本服务手里，预览照常。手上相机转接的是 8089 的画面，没有无损图像和内参，这类程序应直接向 8089 取。
- 本服务不抢占其他程序、不复位设备。两路图像独立采集，不具备同步标定样本或深度采集能力。

## HTTP 接口

| 接口 | 行为 |
| --- | --- |
| `GET /api/status` | 聚合状态及 `cameras.d435i`、`cameras.d405`（配置了时还有 `cameras.d435i_arm2`）；每路含 `arm` 和 `held_s`（还要借出多少秒）；保留旧顶层字段 |
| `GET /stream/d435i`、`GET /stream/d405` | 开始/加入对应 MJPEG 预览 |
| `GET /snapshot/d435i.jpg`、`GET /snapshot/d405.jpg` | 当前新鲜 JPEG，无活动预览或图像过期时 409 |
| `GET /api/cameras/<id>/status` | 单路状态；`kept_s` 是程序要求再保持采集多少秒，`intrinsics` 是本服务直接采集时的内参与畸变 |
| `POST /api/cameras/<id>/pause` | 释放单路预览；请求体带 `{"hold_s": 秒数}` 时同时借出这台相机 |
| `POST /api/cameras/<id>/resume` | 提前归还借出的相机 |
| `GET /api/cameras/<id>/frame` | 等待请求之后的新帧并返回带身份/时间的 JPEG JSON；不带参数时不会启动设备 |
| `GET /api/cameras/<id>/frame?keep_s=秒数` | 同上，但相机没开时先打开，并至少再保持采集这么多秒；可再加 `format=png` 取无损图像 |
| `POST /api/pause` | 释放全部预览（不借出），兼容原标定及 8089 启动协调 |

旧 `GET /stream`、`GET /snapshot.jpg` 仍指向 D435i。前台 Ctrl-C 或服务 SIGTERM 只结束本服务采集与连接，不停止机器人、不关闭 8089。

后台实例的 PID 和日志沿用 `work/camera_monitor/server.pid`、`work/camera_monitor/server.log`；不配置开机启动。

## 验证

```sh
python3 -m unittest tests.camera_monitor.test_monitor -v
python3 -m unittest tests.agent.test_global_camera -v
node tests/camera_monitor/test_panel.cjs
node tests/camera_monitor/test_panel_hold.cjs
```

测试覆盖双路独立生命周期、旧接口、相机身份/时效、共享原始图像、全局观察新帧、最后观看者释放、单路故障隔离、跨站请求拒绝、2 臂相机一路、借用期间拒绝连接与归还后自动恢复、程序取图（启动相机、无损图像与内参、与预览共用、期限到后释放、借出期间拒绝、转接画面没有无损图像）。程序取图的测试用假的相机驱动（`tests/camera_monitor/fake_realsense.py`）驱动真实的采集代码。实机验证结果见项目 `HANDOFF_REPORT.md`。
