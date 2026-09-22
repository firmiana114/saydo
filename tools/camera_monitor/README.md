# D435i 只读监控面板

面向 SayDo 的独立相机预览工具，不访问机械臂或夹爪控制接口。默认使用 1280×720、15 fps 彩色流，提供 MJPEG 预览、帧率与连接状态、当前 JPEG 截图下载。

## 访问

- MSI 本机：`http://127.0.0.1:8088`
- 当前局域网：`http://192.168.101.65:8088`（Wi-Fi 地址变化时需更新）

默认监听 `0.0.0.0:8088`，没有登录认证，仅供可信局域网或受控私网使用。未配置公网转发。

点击“开始预览”才会打开相机。最多四个观看连接共享一个相机采集进程。最后一个连接关闭后会自动释放相机；“释放相机”按钮会结束全部观看连接并释放设备。其他采集程序占用 D435i 时，面板报告占用，不执行设备复位或抢占。

**手眼标定前先点击“释放相机”。** 也可调用 `POST /api/pause`，等待 `/api/status` 的 `state` 为 `idle` 后，再启动标定程序。

## 启动与停止

已有虚拟环境需包含 `pyrealsense2`、`opencv-python-headless`、`numpy`，系统需提供 `fuser`。当前 MSI 环境已具备。

```bash
cd /home/pc/saydo
./tools/camera_monitor/run.sh --port 8088
```

本次后台实例的 PID 保存在 `work/camera_monitor/server.pid`，日志在 `work/camera_monitor/server.log`。未配置开机自动启动。前台运行时按 Ctrl+C 停止服务；后台实例可发送 SIGTERM，服务会释放自己持有的相机。

HTTP 接口：

- `GET /`：监控面板；单独访问网页和状态不会打开相机。
- `GET /api/status`：连接状态、实时帧率、帧序号、帧时间及错误。
- `GET /stream`：开始/加入 MJPEG 彩色预览。
- `GET /snapshot.jpg`：仅在预览运行时下载当前 JPEG，否则返回 409。
- `POST /api/pause`：停止预览并释放相机。

## 验证

已在实际 D435i 上验证连续帧解码、双客户端共用、截图下载、显式释放、最后观看者离开后的自动释放，以及释放后由独立 RealSense SDK 程序重新打开相机。

文件独立放在 `tools/camera_monitor/`，没有修改标定脚本、机械臂配置或既有文档。本次侧会话未提交或推送 Git，避免改动主任务的版本管理状态。
