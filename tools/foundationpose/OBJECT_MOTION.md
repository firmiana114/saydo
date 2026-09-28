# 通用持物运动与释放

支持“抓取物品 → 调整物品朝向 → 保持夹持移动 → 到目标后松爪并退出”。抓取仍使用已保存的夹法。新工具直接指定物品目标，不受旧 `place` 的桌面矩形区域限制；仍检查全局工作区、桌面、夹持、工具配置与运动学。MCP 与 SayDo 执行器使用同一后端。

## 工具链

| 工具 | 输入与结果 |
| --- | --- |
| `get_object_geometry` | 物品ID；返回模型坐标范围、模式、模型摘要和原点/Z轴两端等参考点 |
| `plan_held_object_motion` | 当前物品ID、hold_id、orientation_version、goal、request_id；只读采样并生成持物方案，不运动 |
| `plan_release` | 同上，另有基座坐标退出向量 retreat_vector_base_m；goal=null 以当前位置作为最终放置位姿，仍先上移再下降 |
| `execute_plan` | plan_id、request_id；执行服务器保存的方案，不接受客户端重新传入路径或临时修改参数 |
| `get_task_diagnostics` | task_id；返回持久化任务及对应的新运动报告；旧任务没有此类报告时明确标记不可用 |

规划和执行都返回异步任务。MCP 用 `get_task` 等待，SayDo 自动等待。规划成功状态为 `object_motion_planned`，读取 `result.object_motion_plan.plan_id` 后才能执行。执行成功状态为 `completed`。持物移动后 `hold_id` 保留、`orientation_version` 加1（包括只平移）；下一段必须使用新版本。释放后清除夹持记录。

`get_robot_state.held_object_pose` 是根据实际TCP反馈与抓取时的物品/TCP关系推算的位姿，不是独立视觉测量；滑动会使该推算失准。

## 目标格式

下例仅说明参数，不代表现场已验证的可达目标。`target_position_m` 是物品参考点的位置，不是夹爪TCP的位置。单位为米和度。

```json
{
  "reference_frame": "base",
  "reference_object_id": null,
  "reference_point_object_m": [0, 0, 0],
  "target_position_m": [0.30, 0.00, 0.30],
  "target_rpy_deg": [0, 0, 0],
  "path_mode": "joint",
  "position_mode": "fixed",
  "max_rotation_step_deg": 15,
  "waypoints": []
}
```

- `reference_point_object_m` 是模型局部参考点，可选模型原点、底端等；`get_object_geometry` 的包围盒特征不等于已标定接触点。5D物品只能选自身Z轴上的参考点（X/Y=0）。每套夹法TCP偏移已经包含在抓取后的夹持关系中，不重复叠加。
- `reference_frame` 可为 `base`（基座）、`tcp`（规划起点TCP）、`camera`（当前相机光学坐标）、`held_object`（当前6D持物坐标）或 `tracked_object`（当前稳定跟踪的另一6D物品，需要 reference_object_id）。参考系在规划时冻结；不支持把5D物品不确定的横向轴用作完整参考坐标系。
- `target_rpy_deg` 为物品相对参考系的绝对朝向，采用 `Rz(Z) @ Ry(Y) @ Rx(X)`。5D只约束物品Z轴方向，并搜索绕该轴的等价旋转。模型Z轴与物品直立方向的关系应先查模型定义。
- `target_position_m=null` 保留位置；`target_rpy_deg=null` 保留朝向。两者都为空时须给出路点（原地释放除外）。`waypoints` 最多8项，每项必须同时包含 `position_m` 和 `rpy_deg`，其中一个可为null；fixed模式下继承上一段未指定的分量。所有路点使用同一冻结参考系。
- `position_mode=fixed` 约束物品参考点的指令路点位置；不保证关节运动过程中参考点连续不动。`adaptive` 仅允许没有显式位置目标的持物调姿，允许搜索移位，最终TCP须高于桌面10厘米；方案明确返回实际选定目标。释放须用fixed。
- `path_mode=joint` 使用离线验证得到的关节角执行movej；`linear` 使用movel。调姿默认拆为最多15°的旋转段，可设1–15°，后端上限不可绕过；分段预留0.2°控制器量化余量。拆段不能保证任意姿态都可达。无解时可以只读重规划、更换中间路点；已经开始执行的持物动作失败后，仅在后端recovery.can_replan=true时返回模型重新规划，使用新的夹持版本；旧方案不可重放。释放任务失败不自动恢复。

例如先调直再平移：第一次仅指定目标朝向，规划/执行后读取新夹持版本；第二次仅指定目标位置，保留调整后的朝向；最后调用 `plan_release(goal=null, retreat_vector_base_m=[0,0,0.05])`，确认松爪后沿基座+Z退出50毫米。是否有支撑以及退出方向是否避开物品，仍需依据现场确定。也可以在释放方案中直接指定位置和朝向，后端先到位再松爪。

## 执行、记录与限制

方案有效期5分钟，绑定当前后端进程、夹持身份/版本、起点、模型、位姿模式及配置摘要。重启、过期、配置变化或起点偏离（关节0.5°、TCP位置2毫米/朝向2°）后须重规划；不自动使用旧方案。每个plan_id只能进入执行一次，即使失败也不能换request_id重放。同request_id同参数查询/提交仍返回原任务。

速度读取前端统一速度设置（当前上限25%），不能通过工具临时覆盖。单方案最多64个运动路点、480个采样点，规划搜索有20秒预算，执行运动有90秒预算。执行逐段重新检查运动学，核验夹持接触和实际到位，目标误差阈值为参考点2毫米、朝向2°。松爪须连续确认完全张开才退出；松爪已尝试但反馈不明时不恢复旧夹持记录。

当前检查包括正逆解、采样点机械臂自碰撞、TCP工作区及物品与桌面的包围范围。直线路径每10毫米/5°采样；关节段未做完整连续轨迹碰撞检查。尚未实现夹爪/环境完整碰撞、支撑面接触、力控插入或松爪后独立位姿验证。`validation_scope` 与 `object_settled_pose_verified` 明确返回这些边界，不把“松爪到位”描述成“已放稳”或“已插入孔”。

方案存于 `work/grasp/object_motion_plans.sqlite`；规划/执行报告存于 `work/grasp/object_motion_jobs/`，通过任务内的精确引用读取。诊断含候选失败原因、关节目标、到位误差、松爪反馈和异常阶段，不新增跨物品的模型配置。服务重启不会恢复夹持记录或自动重放计划。SayDo重启遇到未确认的execute_plan会锁住新动作，并按原task_id只读核对。

更新部署后，第5页的新任务使用新工具；已经运行的第6页Codex会话需重新启动会话以刷新MCP工具清单。不要为刷新清单中断仍在执行的物理任务。

统一放置由plan_release生成：最终目标沿Base +Z上方100 mm预放置点→保持最终朝向直线下降→确认到位后松爪→可选退出。高度由config/grasp/trial.json的release_preplacement_height_m配置，缺省0.1 m（允许0.01～0.3 m）。同一个物品参考点定义两端位置，不按模型名或特征类型分支。旧区域放置保留区域选点，approach_height_m同样配置为0.1 m。
