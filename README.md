# SayDo（言动）

**说一句话，让机械臂替你完成想做的事。**

项目面向自然语言驱动的机械臂操作；位姿跟踪、LLM 与 Agent 是实现这一体验的技术基础。该体验是项目目标，目前已完成基础机械臂驱动验证。

## 项目目标

1. 驱动睿尔曼机械臂，验证关节控制与状态反馈。
2. 接入 D435i RGB-D 相机与 [FoundationPose](https://github.com/NVlabs/FoundationPose)，持续跟踪被抓取物体的位置和朝向。
3. 后续接入 LLM、Agent 和语音交互，实现语音指令到机械臂操作的完整流程。

## 当前目录与状态

- `ros2_rm_ws/`：睿尔曼 ROS 2 工作区，包括源码、原有编译产物、MSI 的 Jazzy 编译产物和验证记录。
- 已验证硬件：RM75-BI，7 自由度，三代控制器，IP `192.168.1.19`。
- 已完成：MSI 上的 ROS 2 Jazzy 环境、驱动连接，以及全部七关节的小幅往返验证。
- 已连接：RealSense D435i 相机接入 MSI，知行 CTAG2F90D 夹爪接到机械臂末端；相机刚性固定在末端、随夹爪运动。
- 已完成基础接入：D435i RGB-D 采集、夹爪小幅开合、用户接受的手眼标定、名义 grip_tcp 配置与负载辨识，以及 FoundationPose GPU 环境和离线重建/推理流程。
- 待完成：TCP 几何验证、胶带模型和定位质量改善、独立新视角验收、连续实时跟踪、实际抓取、LLM/Agent 与语音控制。

详细环境与测试结果见 `ros2_rm_ws/HANDOFF_REPORT.md`。

## FoundationPose 源码与物体采集

官方源码已按固定版本接入 `third_party/FoundationPose/`，含多视角神经重建代码。使用 [单视角 RGB-D 采集](tools/object_scan/README.md) 的胶带数据，已完成 GPU 部署、分割、草模重建和离线 register/同帧 track_one 检查；模型与定位质量尚未验收。版本、命令、权重来源和实际验证范围见 [FoundationPose 接入说明](tools/foundationpose/README.md)。

```bash
python3 tools/foundationpose/check_source.py
```

## 相机、夹爪与官方资料

以下安装情况由用户于 2026-09-14 确认；连接到设备不代表已完成采集、运动或标定验证。

| 设备 | 型号 | 安装与验证状态 |
| --- | --- | --- |
| 相机 | Intel RealSense D435i | 刚性固定在末端夹爪旁；彩色与对齐深度采集已验证，胶带侧壁仍有明显深度缺失。 |
| 夹爪 | 知行机器人科技（苏州）有限公司 CTAG2F90D，官网写作 CTAG2F90-D | RM Plus 通信与小幅开合已验证；grip_tcp 为名义偏移，实际抓取中心尚未几何验证。 |

该安装方式为眼在手上（Eye-in-Hand）：相机随机械臂末端运动。已接受的手眼结果和名义工具中心点（TCP）配置见 `config/calibration/`；手眼求解采用 8×11 内角点和用户提供的 30 mm 格长，原始实测边界保留在该目录说明中。

| 设备 | 官方资料 | 用途 |
| --- | --- | --- |
| D435i | [产品页面](https://www.realsenseai.com/products/depth-camera-d435i/)；[D435i 开发说明](https://github.com/realsenseai/librealsense/blob/master/doc/d435i.md) | 查询设备信息和开发注意事项。 |
| D435i | [机器人操作系统第二代（ROS 2）驱动说明](https://github.com/realsenseai/realsense-ros) | 将相机接入项目使用的软件通信框架，查阅采集、深度对齐及坐标定义。 |
| D435i | [成像参数与坐标换算说明](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0) | 查询镜头参数、距离单位及图像点到空间点的换算。 |
| CTAG2F90-D | [产品与尺寸图、三维模型入口](https://www.changingtek.com/diandong/147) | 核对夹爪型号、规格和安装几何尺寸。 |
| CTAG2F90-D | [官方资料中心](https://www.changingtek.com/service) | 在“产品使用手册”中查找《CRT 睿尔曼+知行协作手操作说明》；尺寸图与三维模型选择 CTAG2F90-D。 |

## 启动机械臂驱动

先确认没有重复运行的驱动进程，再执行：

```bash
cd /home/pc/saydo/ros2_rm_ws
./start_rm75_driver.sh
```

脚本只启动驱动，不自动发送运动指令。

## 迁移兼容

原路径 `/home/pc/ros2_rm_ws` 保留为指向本项目工作区的符号链接，以兼容正在运行的驱动、历史编译缓存及辅助脚本中的绝对路径。实际文件均位于 `/home/pc/saydo/ros2_rm_ws`，未复制第二份。

后续若重新组织编译目录，可在重新构建并排除旧路径依赖后移除该兼容链接；目前请保留。

## GitHub 版本管理范围

私有仓库：https://github.com/firmiana114/saydo

版本库包含项目说明、启动脚本与睿尔曼源码快照。项目根目录 `doc/`、ROS 编译和安装产物、日志、`work/` 临时数据、模型权重及传感器记录保留在本地，不上传。首次克隆后需安装 ROS 2 Jazzy 并按 `ros2_rm_ws/HANDOFF_REPORT.md` 的构建命令生成 `install_jazzy/`，随后才能运行启动脚本。

第三方源码版本与 SDK 库说明见 `VENDORED_SOURCES.md`。
