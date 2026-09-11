# 第三方源码来源

`ros2_rm_ws/src/ros2_rm_robot/` 保存睿尔曼驱动的完整源码快照，供本项目独立克隆与构建。

- 上游仓库：https://github.com/RealManRobot/ros2_rm_robot
- 来源分支：`humble`
- 来源提交：`c941b565e4f9174afa36561f143ef5fbbb744750`
- 本地既有差异：`rm_moveit2/CMakeLists.txt` 增加 `find_package(Boost REQUIRED COMPONENTS random)`；首次上传保留该修改。
- 保留上游文件中的版权与许可声明，不额外声明第三方代码许可。

GitHub 中收录实际源码文件，而非 Git 子模块。MSI 上已有的嵌套 `.git` 元数据仍原地保留，但不会上传至 SayDo 仓库。`rm_driver/lib/` 中的厂商 SDK 动态库是构建依赖，明确纳入版本控制；本机重新编译产生的动态库不纳入。
