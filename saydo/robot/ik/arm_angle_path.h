// 按臂角枚举逆解，搜索一条连续且不碰到桌面、立杆的关节角路径。
//
// 臂角（arm angle）：夹爪位姿固定不动时，手肘还可以绕“肩到腕”的连线
// 转动，转过的角度就是臂角。7 关节机械臂比确定夹爪位姿所需多一个关节，
// 多出来的自由度正好由它表示。厂家算法库能按指定的臂角直接算出关节角，
// 所以同一个夹爪位姿下“手肘可以怎么摆”能完整地列出来，再从中挑出与
// 前后衔接、又避开障碍物的摆法。
#ifndef SAYDO_ROBOT_IK_ARM_ANGLE_PATH_H_
#define SAYDO_ROBOT_IK_ARM_ANGLE_PATH_H_

namespace saydo_ik {

// 从标准输入读取路点和外形，搜索关节角路径，结果按 IK_FK_V1 协议写到
// 标准输出。只调用厂家算法库的计算接口，不连接机械臂。
//
// 调用前须已初始化算法库并设置好工具坐标系。
// 参数 start_joints_deg：起点（机械臂当前）的 7 个关节角，单位度。
// 标准输入的格式：路点个数、时间预算（毫秒），每个路点 6 个位姿数
// （米、弧度）加相邻关节角变化上限（度），最后是 ReadArmModel 的格式。
// 返回进程退出码：0 找到路径；1 没有可行路径；2 输入无效；4 超出预算。
int RunArmAnglePath(const float* start_joints_deg);

}  // namespace saydo_ik

#endif  // SAYDO_ROBOT_IK_ARM_ANGLE_PATH_H_
