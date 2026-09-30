# 相机标定采集

## D435i与D405单组原生采集

新增`capture_pair.py`，每运行一次保存一组双路原生1280×720 PNG、内参和实际法兰位姿。组间允许改变板/机械臂姿态，组内必须停稳；通过8088采集，采集阶段关闭8089，不发送机器人写入命令。命令、静态质量门槛及角点顺序审阅见 [双相机采集说明](DUAL_CAMERA_CAPTURE.md)。

## 眼在手上：只读静态配对采集

当前工具仅采集一个静止姿态的候选样本，不发送运动、夹爪或负载写入命令，也不输出最终手眼标定结果。

## 推荐入口：单次自动检查与采集

当前已经冻结前 20 组求解候选，后续目标是采集 5 组独立验证候选。每次手动调整机械臂、保持棋盘固定并停稳后，执行一次：

```bash
cd /home/pc/saydo
.venv/bin/python tools/handeye/collect.py --session he_ref_calibration_20260914
```

脚本只运行一次，没有循环、交互等待或自动重试。无论保存、重复或拒收，本次处理完成即退出，不移动机械臂，不恢复相机预览。

流程：

1. 检查 `dataset_split.json` 与已有样本，满 5 组验证候选时直接退出。
2. 读取当前工具、工作坐标系和实际位姿，与所有已有候选比较。
3. 若相对任一候选的位置差 **小于 10 mm 且朝向差小于 5°**，判为近重复，跳过相机采集。
4. 必要时通过本机监控服务暂停预览，等待释放设备；等待 2 秒后调用一次 `capture.py`，保留既有静止和图像质量门槛。
5. 对实际采得的位姿再次去重，核对相机、棋盘、工具和固件配置与求解集一致。
6. 合格数据标为独立验证候选，原子更新分组与 `latest_progress.json`，不修改已冻结的求解样本名单。

输出中的 `state`：`captured` 为新增候选；`duplicate` 为近重复跳过；`rejected` 为质量、采集后去重或参数一致性不合格；`complete` 为验证候选已满额；`error` 为需处理的运行错误。退出码：0 为已保存或满额，2 为本次跳过/拒收，1 为运行错误。

采集数据和拒收记录均留在被 Git 忽略的 `work/handeye/`。同一会话使用排他锁，防止两个单次采集脚本同时修改分组。分组文件缺失或异常时明确报错，不自动重新划分现有数据。

该入口需要已存在的冻结分组计划。若要在新会话中进行未分组的基础采集，可使用下方 `capture.py`；它本身不包含自动去重或验证集登记。

## 基础采集入口

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

2026-09-14 单次自动采集验证：19 项测试通过；实际只采集一次并退出，新增验证候选 `171206_e8bee9d1`，RMS 0.5103 px。数据分组为求解候选 20 组、独立验证候选 1/5 组，尚未求解手眼变换。

## 离线求解与独立验证

完成分组计划中的独立验证采集后，可执行一次离线求解。脚本不连接相机或机器人，不修改原始样本/分组，不自动发布 TF，也不写入控制器。

当前采集环境的 OpenCV 5.0.0 Python wheel 未暴露 `calibrateHandEye`，因此使用独立的求解环境：

```bash
cd /home/pc/saydo
python3 -m venv work/handeye_solver_venv
work/handeye_solver_venv/bin/python -m pip install -r tools/handeye/requirements-solve.txt
work/handeye_solver_venv/bin/python -m unittest discover -s tools/handeye -p test_solve.py -v
work/handeye_solver_venv/bin/python tools/handeye/solve.py --session he_ref_calibration_20260914
```

已配置好环境时，只需运行最后一条命令。采集仍使用原有 `.venv`。

求解规则：

- 读取冻结的 `dataset_split.json`，只使用 `training_candidate_ids` 求解；验证集不参与拟合、算法选择或调参。
- 当前预先固定使用 PARK 方法，输出 `flange_T_color_optical_camera`（米制）。反向变换单独保存。
- 检查原始 PNG 的 SHA-256、图像尺寸、角点与相机参数、机器人配置，以及实际状态到法兰变换的对应关系。
- 使用求解集确定固定棋盘的基座参考位姿；验证集只计算相对该参考的误差，不重新估计自己的参考中心。
- 计算逐组位置/朝向一致性、全局重投影误差，以及仅使用求解集的留一法敏感性。
- 输出为 `work/handeye/<session>/solutions/<unique_run>/result.json`，不覆盖旧结果。原始数据哈希保存在结果中。

单帧 PnP 的重投影误差与手眼变换的空间误差不是同一个指标。固定棋盘的一致性也不代表经过独立测量的绝对定位精度。求解器新生成的结果默认保持 `acceptance_status=not_decided_no_task_tolerance_or_absolute_reference`、`deployed=false`，接受状态按每次具体结果单独记录。

2026-09-14 初次离线结果：20 组求解、5 组独立验证。验证位置 RMS 6.859 mm、最大 8.911 mm；朝向 RMS 0.319°、最大 0.494°。求解集留一法引起的结果变化最大为 1.835 mm / 0.094°。这些是诊断结果，未据此启用机器人抓取。

2026-09-14 用户明确将 `solutions/172825_bd2eabc9/result.json` 对应的这组标定标记为可用。该结果已更新为 `acceptance_status=user_accepted_usable`，并记录用户确认来源和时间；矩阵和误差未改动，`deployed=false`。这一接受决定仅适用于该组结果，不自动接受后续重新求解的结果。
