# 眼在手上：只读静态配对采集

当前工具仅采集一个静止姿态的候选样本，不发送运动、夹爪或负载写入命令，也不输出最终手眼标定结果。

## 使用

依赖使用项目 `.venv`。当前验证版本记录在 `requirements.txt`。

如果相机监控页面正在预览，先点“释放相机”，或对 `http://127.0.0.1:8088/api/pause` 发 POST 并等待状态为 `idle`。采集程序不会抢占相机、复位设备或停止其他进程。

```bash
cd /home/pc/saydo
.venv/bin/python tools/handeye/capture.py --session calibration_01
```

相机保持静止并完整看到棋盘；本程序不会移动机械臂。采集结束后相机设备会释放；如需恢复网页预览，请重新点击“开始预览”。

## 数据与判定边界

数据保存在 `work/handeye/<session>/<unique_sample_id>/`，每次创建独立目录：

- `color.png`：从 RealSense SDK 取得的无损原始彩色图。
- `corners.jpg`：角点预览。
- `sample.json`：设备与坐标系配置、原始机械臂回复、时间信息、图像内参/畸变、88 个角点、`base_T_flange`、棋盘位姿候选及质量指标。

当前使用 8×11 内角点、用户提供的 0.03 m 格长、1280×720/15 fps 彩色流。格长仍需要实测确认。

限定支持零偏移 `Arm_Tip` 或 `he_ref` 工具和零变换 `Base` 工作坐标系，其他配置明确拒收，不自动切换。RealMan JSON 的位置从微米转换为米，RPY 从毫弧度转换为弧度，旋转为 `Rz(yaw) @ Ry(pitch) @ Rx(roll)`。

原始图像畸变参数全部为零时才采用针孔模型；不将非零的 RealSense 畸变模型直接当作 OpenCV 畸变参数。PnP 使用 IPPE，保存正深度候选及重投影误差，保留平面歧义供审阅。

时间配对采用图像前后多次状态读取和静止检查，保存相机时间戳及其时钟域、主机 monotonic/unix 时间。它不是硬件同步方案，也不适用于运动中采集。

单样本筛选阈值：关节变化 ≤0.03°、法兰平移变化 ≤0.2 mm、旋转变化 ≤0.15°、采集前后时间窗口 ≤0.5 s、最后一帧等待 ≤0.2 s、重投影 RMS ≤1 px。这些是暂定筛选阈值，不代表最终手眼标定精度。

`status=candidate` 只说明本次静止、图像和坐标检查通过。还需审阅棋盘物理角点方向、实测格长、平面位姿歧义，以及多姿态独立验收。`handeye_calibrated` 始终为 false。

总负载/质心未确认不妨碍当前不动状态下的读取，但不构成多姿态运动已经准备好的证明。程序记录工具配置，不根据夹爪标称重量猜测或写入负载。

## 验证

```bash
.venv/bin/python -m unittest discover -s tools/handeye -v
```

覆盖 JSON 单位、坐标旋转、角度跨 ±π、运动后回到原点的拒收、错误状态、非零工具偏移、已知投影几何恢复以及运动指令的网络发送前拒绝。

2026-09-14 首组实机候选样本：88 个角点，重投影 RMS 0.5935 px，关节跨度 0.006°，法兰位置跨度 0.03454 mm。单组数据不能求手眼变换。

依据：[RealMan 状态查询单位](https://develop.realman-robotics.com/robot/json/armState/)、[RPY 定义](https://develop.realman-robotics.com/robot/FQA/sdk/)、[OpenCV PnP 与手眼标定](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)。

2026-09-14 已读回用户创建的 `he_ref`：工具位姿六项为零，负载约 1.032676 kg、质心 (-19, 6, 110) mm。这些是控制器保存的辨识值，不是实称校核结果。负载和质心非零不代表工具位姿存在偏移。采集程序仅核对当前工具，不创建、不选择或修改工具；使用前应在示教器选中 `he_ref` 并读回确认。
