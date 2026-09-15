# 第三方源码来源

`ros2_rm_ws/src/ros2_rm_robot/` 保存睿尔曼驱动的完整源码快照，供本项目独立克隆与构建。

- 上游仓库：https://github.com/RealManRobot/ros2_rm_robot
- 来源分支：`humble`
- 来源提交：`c941b565e4f9174afa36561f143ef5fbbb744750`
- 本地既有差异：`rm_moveit2/CMakeLists.txt` 增加 `find_package(Boost REQUIRED COMPONENTS random)`；首次上传保留该修改。
- 保留上游文件中的版权与许可声明，不额外声明第三方代码许可。

GitHub 中收录实际源码文件，而非 Git 子模块。MSI 上已有的嵌套 `.git` 元数据仍原地保留，但不会上传至 SayDo 仓库。`rm_driver/lib/` 中的厂商 SDK 动态库是构建依赖，明确纳入版本控制；本机重新编译产生的动态库不纳入。

## FoundationPose

- 路径：`third_party/FoundationPose/`
- 上游：https://github.com/NVlabs/FoundationPose
- 来源分支：`main`
- 固定提交：`a1b694b83e633c2cb6115b9063d940a687759392`
- 接入日期：2026-09-15
- 方式：完整导出上游 Git 跟踪树，54 个文件、实际源码快照；不包含嵌套 Git 元数据。
- 本地修改：无。所有原文件与逐文件 SHA-256 清单一致，清单为 `third_party/FoundationPose.upstream.json`。
- 许可证：原样保留 `third_party/FoundationPose/LICENSE` 及每文件版权声明。该许可证第 3.3 节限定研究/评估的非商业用途，不替第三方代码声明其他许可证。
- 权重、demo 数据、环境和编译产物不随此次源码接入生成或上传。
- SayDo 使用入口与运行状态：[FoundationPose 接入说明](tools/foundationpose/README.md)。
