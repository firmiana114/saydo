# MSI 睿尔曼机械臂驱动验证

验证日期：2026-09-11。工作区：`/home/pc/saydo/ros2_rm_ws`，SSH 别名：`msi`。

## 相机与夹爪接入状态（2026-09-14）

- 用户确认：Intel RealSense D435i 已连接本机，刚性固定在机械臂末端夹爪旁、随末端运动；夹爪为知行机器人 CTAG2F90D，官网写作 CTAG2F90-D，已连接到机械臂。
- 本次会话的只读设备枚举已识别 D435i；未启动图像采集，未发送夹爪或机械臂运动指令。相机采集、夹爪控制和标定结果均未验证。
- 现场已有普通黑白棋盘格标定板；横纵格数、单格实际边长、夹指及转接件尺寸、夹爪实际通信配置未确认。
- 型号、安装关系及官方资料入口已纳入根目录 [README.md](../README.md#相机夹爪与官方资料)，包括 D435i 开发与驱动说明，以及知行《CRT 睿尔曼+知行协作手操作说明》的资料中心入口。
- 资料核对：RealSense 开发与驱动页面可读取；知行产品与资料中心信息已通过官方网页的检索索引核对，直接访问暂未成功，未验证其中各下载文件。
- 本次仅修改项目说明与本报告，`git diff --check` 格式检查通过；未修改驱动、配置或本地辅助脚本。
- 下一步：确认棋盘格尺寸，验证相机采集，核实机械臂反馈的坐标参照后采集标定数据；夹爪实际抓取中心另行确认。以下机械臂运行与运动结果仍为 2026-09-11 的历史记录。

## 结果与证据

- 设备实际为 **RM75-BI，7 自由度，第三代控制器，固件 V1.7.0**，不是最初猜测的 RM65。
- 控制器 TCP：`192.168.1.19:8080`；MSI 有线网口 `enp3s0`：`192.168.1.10/24`。
- ROS 驱动成功连接、查询设备身份并发布真实的 7 关节反馈。
- 实机先完成第 7 关节 2° 往返，再完成更便于观察的 **10° 往返，速度 1%，到位停留 3 秒**；10° 转出、返回各约 5.1 秒。
- 10° 测试采集 96 组 ROS/控制器对照样本，最大角度差约 0.0342°；其余六关节相对起点最大变化 0.004°。
- 最后一次控制器查询中，第 7 关节为 0.000°，系统错误为 0，全部关节已使能，关节错误码均为 0。
- 现场观察确认：用户已确认第 7 关节 10° 往返“看清了，运动正常”；随后明确要求验证全部关节。
- 追加验证：第 1～7 关节逐一以 1% 速度转动 +3°，停留 2 秒，再返回起点；7/7 关节、14/14 运动段全部通过。每次只移动一个关节。
- 全关节验证采集 260 组 ROS/控制器对照样本，最大角度差 0.0478°，非目标关节最大变化 0.0070°。
- 验证范围是基本驱动通信、全部七关节的小幅指令执行及反馈闭环；未验证全行程、MoveIt、笛卡尔轨迹、负载抓取或全部驱动功能。

## 环境与构建

系统为 Ubuntu 24.04 x86_64，安装了官方 ROS 2 Jazzy ROS Base、ros-dev-tools、build-essential、CMake、arping 及依赖。`dpkg --audit` 未报告问题。

保留复制过来的 Humble `build/`、`install/`、`log/`。重新编译使用独立目录：

```bash
cd /home/pc/saydo/ros2_rm_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base log_jazzy build --base-paths src/ros2_rm_robot \
  --build-base build_jazzy --install-base install_jazzy \
  --packages-select rm_ros_interfaces rm_driver --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
```

两个包编译通过。已比较官方 Jazzy 分支：驱动核心源码与现有 Humble 分支一致，差异集中在配置和接口说明，因而直接沿用现有源码编译。未修改源码仓库；原有 `rm_moveit2/CMakeLists.txt` 修改和安装日志未动。

## 启动与当前状态

当前驱动在后台运行，启动 PID 记录在 `work/rm75-launch.pid`。未配置开机自动启动，不会自动重复运动测试。

下次启动前先确认没有已有驱动进程；已停止时运行：

```bash
cd /home/pc/saydo/ros2_rm_ws
./start_rm75_driver.sh
```

脚本加载 Jazzy 与 `install_jazzy/local_setup.bash`，设置 `ROS_DOMAIN_ID=75` 和 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，启动 `rm_75_driver.launch.py`。需要使用 ROS CLI 时，同样先加载这两个环境文件并设置这两个变量。

驱动沿用 `src/ros2_rm_robot/rm_driver/config/rm_75_config.yaml` 中正确的 RM75、7 关节、TCP `.19`、UDP `.10:8089` 配置。启动时将控制器状态反馈目标从原先 `.234:8090` 改为 MSI 的 `.10:8089`；API 日志报告反馈周期 5 ms，JSON 协议查询中的 `cycle=1` 表示一个 5 ms 单位。

网络新增 NetworkManager 配置 `realman-direct`，有线静态地址 `.10/24`、不设置默认路由、关闭该配置的 IPv6。原 Wi-Fi 和原有有线配置均保留。到机械臂的路由已经直接走 `enp3s0`，不再经过 Clash 的 Meta 虚拟网卡。

## 故障处理记录

最初系统错误 `4116 / 0x1014` 为关节超出软限位：第 6 关节约 -130.462°，下限 -128°。用户按照设备说明现场恢复后，重新读取确认错误清零、关节回到零位附近。未放宽限位、未关闭碰撞保护。

第一次 2° 检查已使关节到位，但独立 TCP 观察脚本收到控制器广播的异步到位事件，误按状态查询响应解析，触发停止。已修复观察脚本，使其按响应类型匹配查询，保留异步事件记录；随后返回原点及完整 10° 往返均通过。此修复不涉及厂商驱动源码。

## 日志与辅助文件

- `work/rm75-driver.log`：驱动运行日志。
- `work/build-jazzy.log`、`work/apt-install.log`：构建与安装记录。
- `work/all-joints-validation.json`、`work/all-joints-validation.log`、`work/all-joints-summary.json`：最终七关节逐一验证的全部样本、结果与汇总。
- `work/motion-validation.json`、`work/motion-validation.log`：之前第 7 关节 10° 测试完整样本与结果。
- `work/verification-summary.json`、`work/final-controller-state.json`：最终汇总与直接状态查询。
- `work/motion-first-attempt.json`、`work/motion-2deg-validation.json`：最初停止及后续 2° 恢复记录。
- `work/msi_motion_smoke.py`：单独第 7 关节有界检查；`work/msi_all_joints_smoke.py`：七关节逐一检查。无 `--execute` 时均不运动。脚本会真实驱动机械臂，后续运行须重新确认现场条件。

官方依据：[驱动说明](https://develop.realman-robotics.com/robot/ros2/driver/)、[错误代码](https://develop.realman-robotics.com/en/robot/json/errorList/)、[Jazzy 分支](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy)。

## 项目目录迁移

工作区已迁入 SayDo（言动） 项目。原路径 `/home/pc/ros2_rm_ws` 保留为兼容符号链接，指向当前工作区，以保留运行中驱动及历史绝对路径的可用性。迁移过程中未重启驱动、未发送运动指令。

## GitHub 版本管理

项目根目录纳入 `firmiana114/saydo` 私有仓库，默认分支 `main`。根目录 `doc/`、编译与安装目录、日志、运行记录、模型权重和凭据文件由 `.gitignore` 排除。GitHub 保存第三方源码实际文件及必要 SDK 动态库，来源记录见项目根目录 `VENDORED_SOURCES.md`。嵌套的上游 Git 元数据和既有 Boost 修改在 MSI 原地保留。

首次推送通过已认证的本机 GitHub CLI 完成，没有把本机 GitHub 凭据复制到 MSI。以后从 MSI 推送需使用该主机自己的 GitHub 认证，或继续通过已认证的本机同步。

## 2026-09-14：手眼标定静态采集准备

新增 `../tools/handeye/` 只读配对采集工具，使用项目 `.venv`，详见该目录 README。7 项测试通过；第一组真实样本已保存为候选，88 角点齐全、重投影 RMS 0.5935 px、法兰位置跨度 0.03454 mm。数据位于项目根目录 `work/handeye/initial_static_check/134310_cc77b0ad/`，不纳入 Git。没有发送机械臂运动、夹爪或负载写入命令。

本次为采集原始 PNG 而调用相机监控的 pause 接口释放相机；采集完毕设备已释放，恢复网页预览需要点击“开始预览”。现有未提交的 `tools/camera_monitor/` 保留，不纳入本次提交。

尚未完成手眼标定：只有一个姿态，棋盘实测尺寸/角点方向待复核，仍缺多姿态采集和独立验收。工具当前负载为 0；厂家公开参数可确认 CTAG2F90D 夹爪 0.8 kg 和 D435i 相机 75 g，但不能据此推定整套附件质量及质心。已查到睿尔曼官方负载辨识流程，需结合实际安装和安全空间实施，不会自动执行默认大范围运动。检索补充见本地 `../doc/标定准备补充检索.md`。
