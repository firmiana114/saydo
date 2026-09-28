# 先观察全部目标，再执行静态场景任务

MCP和SayDo均提供 `capture_scene_snapshot`、`get_scene_snapshot`。模型或相机位姿不由LLM计算；后端在机械臂静止时配对RGB-D和实际工具位姿，使用现有手眼标定转换到Base基座坐标。每个任务首先声明所需全部实例，定位全部通过后才允许引用快照抓取。

## 入口与顺序

```json
{
  "targets": [
    {"instance_id":"tube1","object_id":"transparent_test_tube_150mm","role":"source","box":null},
    {"instance_id":"rack1","object_id":"test_tube_rack_40_272x112x85","role":"reference","box":null}
  ],
  "environment_static": true,
  "request_id": "capture-scene-example"
}
```

这只是调用格式示例。MCP需要request_id并用get_task等待；SayDo自动生成请求ID并等待。capture只读设备、暂停原实时跟踪，在GPU所属线程用同一份冻结RGB-D依次运行检测、分割、FoundationPose定位与深度/掩膜质量检查；不移动机械臂或夹爪，也不要求各模型同时常驻GPU。自动检测尚未退出时等待它释放请求资源，不重复触发“上一轮请求尚未退出”。

最多8个实例，instance_id在本次快照中唯一；object_id是模型名，多个实例可以共用模型。box为640×360图像整数框；null要求唯一自动候选。多个候选、同模型重叠框、无目标、深度不可靠、采集期间机械臂移动都会使整份快照失败；结合相机画面明确框选后可重新采集。角色source表示本任务要抓取的物品，reference表示留在原地的参照物。

capture成功任务状态为scene_ready，结果含scene_id、各实例base_T_object、质量及有效性。此时才用 `grasp(object_id, scene_id, instance_id, request_id)` 或preview_grasp；新的MCP/SayDo接口必须带场景引用。SayDo拒绝使用未在本次任务中采集的scene_id。旧HTTP只含object_id的接口仅在没有活动快照时保留兼容；快照捕获失败或未全部准备齐全时不能用旧接口绕过。

任务作者/模型必须声明自然语言任务涉及的全部目标；后端核验的是声明的集合，不能自动判断语言中是否还遗漏了物品。不能先只声明试管、抓起后才补充试管架。持物后禁止重采场景及重新初始化环境目标跟踪；仍可读取实时图片检查意外变化。

## 引用固定目标和命名特征

持物运动/释放沿用plan_held_object_motion、plan_release和execute_plan。goal新增：

```json
{
  "reference_frame":"scene_object",
  "scene_id":"<capture返回的scene_id>",
  "reference_instance_id":"rack1",
  "reference_feature_id":"A01.entry",
  "reference_object_id":null,
  "reference_point_object_m":[0,0,-0.075],
  "target_position_m":[0,0,0.03],
  "target_rpy_deg":[0,0,0],
  "path_mode":"linear",
  "position_mode":"fixed",
  "max_rotation_step_deg":15,
  "waypoints":[]
}
```

例如此目标表示试管模型底端位于本次估计孔口坐标系上方30毫米，朝向相对该坐标系为零；并非已验证可执行动作。reference_feature_id=null时使用参照物模型坐标；命名特征可表示孔口、托盘中心或放置面。其他reference_frame须把三项快照字段设为null。5D物品的横向轴不可观测，不能作为完整场景参照系；5D被夹物品仍支持沿自身Z轴选参考点。

get_object_geometry新增features（每项id/kind/object_T_feature）、feature_revision与说明。自定义特征存于对应物品目录 `features.json`：

```json
{"units":"metres","features":[{"id":"place_center","kind":"surface","object_T_feature":[[1,0,0,0],[0,1,0,0],[0,0,1,0.05],[0,0,0,1]]}]}
```

现有试管架hole_positions.json自动适配为40孔×3特征（A01.entry、A01.middle、A01.support等），保留原几何文件，不复制成跨物品配置。未标记试管架有绕Z轴180度歧义，孔位ID只属于此次估计坐标，不能保证实物标签身份。特征不代表已验证孔位空闲、实物尺寸/接触或插入成功。

## 生命周期与验证边界

- 快照固定的是环境目标基座位姿，不是整条关节轨迹。抓取和每段运动仍按实际关节、TCP、夹持关系重新规划和到位核验；相机随臂移动不会重新计算静态物品坐标。
- 单次采集预算180秒（正在进行的GPU调用须返回后才能完成取消），期间持续检查机械臂未移动。冻结帧定位不做实时追帧；质量为单帧模型/深度/掩膜拟合及同帧细化稳定性，不等同于外部精密测量。
- 快照只在当前后端进程和当前活动场景中执行，30分钟过期；模型、位姿模式、特征或手眼/TCP标定变化后拒绝沿用。新采集取代旧快照。未使用的旧快照可以查询证据，但不能恢复执行。
- 实例从static进入moving/held，原始位姿不再可作目标；抓取失败且已发动作记unknown。释放尝试记release_unknown，成功释放及退出后记placed_unverified并保存预测物品位姿，不能称为独立观察到放稳。全部source放置完成后结束活动场景。
- get_scene_snapshot返回最新实例状态；capture任务中的scene_snapshot只是采集完成时的历史证据。get_status不重复输出旧的实例状态，active_scene_id提供活动场景ID。
- 尚未实现完整环境/夹指碰撞、孔位占用、插入力控和释放后独立定位。冻结场景不能替代这些检查；环境实际改变时应停止并重新安排任务。

场景元数据及更新写入 `work/grasp/scenes/<scene_id>.json`，冻结RGB-D证据为同目录 `<scene_id>/frame.npz`。重启后历史场景明确标记不可执行，不自动恢复旧抓取。第5页的新任务采用新流程；第6页已有Codex会话需重启会话刷新MCP参数。
