# 胶带多视角：单次 RGB-D 采集

每次手动拖动机械臂换好视角、松手并停稳后执行一次。程序只保存一个视角，然后退出，不控制机械臂或夹爪，不自动恢复相机预览。

```bash
cd /home/pc/saydo
.venv/bin/python tools/object_scan/capture_view.py --session tape_20260915
```

首次安装新增依赖：`.venv/bin/python -m pip install -r tools/object_scan/requirements.txt`。

## 现场操作

1. 胶带和机械臂底座在整组采集期间保持固定。只改变相机视角；不要翻转或移动胶带来拍背面，否则应新建会话并另做配准。
2. 保持胶带完整入镜，尽量占画面中间较大区域。从正面和各侧斜视采集，覆盖外壁与内孔；不为拍不可见底面而碰撞桌面或改变安装。
3. 停稳、松开拖动按钮，再运行命令。相机需要约数秒预热。
4. `candidate` 表示本视角保存；`duplicate` 表示与已有视角太接近，换角度再运行；`rejected` 表示检查未通过，查看 error。每次仅尝试一次，没有重试循环。
5. 建议先采 12～20 个有明显差别的视角，最终以可见表面覆盖和数据质量为准。脚本没有固定配额。没有看到的表面不能宣称已重建。

判重条件：相机位姿相对任一已有有效视角平移小于 10 mm 且转角小于 5°。只绕同一条视线旋转不能代替观察外壁与内孔的视角覆盖。

## 数据

目录：`work/object_scan/<session>/views/<unique_id>/`，不覆盖旧数据。

- `color.png`：原始彩色图，PNG 标准颜色；OpenCV 读取后为 BGR，传给 FoundationPose 前须转换 RGB。
- `depth_raw.png`：对齐彩色后的设备原始 uint16 深度；比例记录在 sample.json。
- `depth_mm.png`：uint16 毫米深度，零表示无效；供后续官方数据格式转换。
- `depth_m.npy`：float32 米制深度，保留实际设备比例。
- `K.txt`：本次视频流真实内参。
- `base_T_camera.txt`：彩色光学相机到机械臂基座的 4×4 变换，平移单位米。
- `sample.json`：原始机器人只读回复、TCP/法兰/相机位姿、时间戳、质量数据和文件 SHA-256。
- `preview.jpg`：人工检查用画面。

会话冻结两份标定配置的 SHA-256 和相机参数。每次校验原始标定记录哈希、当前完整工具配置、零偏移 Base、相机序列号和内参。当前 TCP 反馈通过完整逆矩阵还原法兰，不直接减基座 Z。

```text
base_T_camera = base_T_grip_tcp @ inverse(flange_T_grip_tcp) @ flange_T_color_optical_camera
```

用采图前后的多次状态查询验证静止。它不是硬件同步；不适合运动中拍摄。深度有效率为整幅图统计，不能证明胶带区域的深度完整。物体是否移动、是否完整入镜、是否反光和遮挡仍需人工检查。

## 与 FoundationPose 重建的边界

这些是重建输入候选，还不是物体模型或跟踪结果。尚需逐视角分割胶带（内孔应保留为背景）、检查深度和跨视角配准，再运行神经重建。

FoundationPose 自带 `bundlesdf/run_nerf.py` 的 `run_neural_object_field` 接受 RGB、米制深度、mask 和相机位姿。此处保存的是基座世界坐标，后续应选择物体参考系并转换成该函数所需的 `cam_in_obs`；不能把文件直接冒充已确定的物体中心坐标系。现有约 6.86 mm 手眼独立一致性误差可能影响精细重建，需检查配准并考虑姿态优化。

胶带的反光/透明材质和近似旋转对称性可能影响深度、重建及转角可辨识性，实际效果以运行结果为准。

官方依据：
- https://github.com/NVlabs/FoundationPose/blob/main/bundlesdf/run_nerf.py
- https://github.com/NVlabs/BundleSDF#run-on-your-custom-data

## 验证

```bash
.venv/bin/python -m unittest discover -s tools/object_scan -p 'test_*.py' -v
```

2026-09-15 MSI 实机验证：4 项几何/深度测试通过；首个视角 `101121_b43524bc` 保存成功，7 个数据文件哈希及回读检查通过，整幅图深度有效率 95.57%。重复运行在打开相机前跳过同一姿态，没有新增候选。当前为 1 个候选视角；逐视角分割、重建环境及 FoundationPose 推理尚未完成。
