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
- 待完成：D435i 在 MSI 上的采集验证、FoundationPose 集成、LLM/Agent 与语音控制。

详细环境与测试结果见 `ros2_rm_ws/HANDOFF_REPORT.md`。

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
