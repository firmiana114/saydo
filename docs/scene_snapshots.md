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

## 出画检查：物品被画面裁掉时拒绝定位

**为什么需要**：位姿估计是拿模型去对物品在画面里露出的那部分。物品有一截在画面外时，估计照样会给出各项质量指标都通过的结果，但出画的那一段对结果没有约束。试管这类两头不一样的细长物品，分辨口和底靠的正是两端，一端出画就可能认反。2026-10-09 15:35 的 1 臂抓放就是这样：试管下端约 5% 的表面在画面外，口和底被认反，规划选了手肘甩向立杆的翻转方式，把立杆碰倒了。

**怎么判**：位姿估计一出结果，就把模型表面按这个位姿投影到画面上，按面积统计落在画面外的比例；超过 **1%** 就中止本次采集。实现在 `saydo/perception/frame_coverage.py`，场景采集和工作台页面上的首帧定位都经过它。逐帧跟踪不检查：机械臂靠近物品时物品本来就会滑出画面，方向由跟踪沿用。

**报错的样子**：`tube 有 5.3% 的表面在画面之外（下边 5.3%），上限 1%。…请把物品完整放进画面后重新识别`。写明是哪个目标、出画多少、从哪条边出去的。处理办法是把物品往画面中间挪，或换一个能看全的拍摄姿态。

**1% 的来历**：统计 MSI 上 2026-09-28 至 10-09 的全部场景记录（139 个定位结果），这个比例要么不超过 0.9%（物品完整入画，只是一个角被切到），要么不低于 3.2%（确实被裁掉一截），中间没有数据。按 1% 回放，有 18 个结果会被拒绝，其中 3 个所在的任务当时执行成功了（10-08 13:04 试管 3.7%、16:13 试管架 7.8%、16:50 试管 12.3%）——被裁掉不一定出错，但出错时现有的质量指标发现不了。

**通过时的记录**：场景里每个目标多一项 `frame_coverage`，含 `outside_percent`（画面外占比）、`outside_percent_by_edge`（各边分别多少）、`limit_percent`（上限）。

**局限**：检查用的是估计出的位姿。如果估计为了贴合露出的部分把模型整体挪进了画面，本检查发现不了；历史记录里没有见到这种情况。门限是全部物品共用的一个值，没有按物品分别设置。

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

所有命名特征统一读取各物品目录的features.json；原专用孔位数据已一次性迁移，运行时不再识别专用文件格式。特征只是模型坐标数据，不代表已验证占用、接触或物理编号。

## 生命周期与验证边界

- 快照固定的是环境目标基座位姿，不是整条关节轨迹。抓取和每段运动仍按实际关节、TCP、夹持关系重新规划和到位核验；相机随臂移动不会重新计算静态物品坐标。
- 单次采集预算180秒（正在进行的GPU调用须返回后才能完成取消），期间持续检查机械臂未移动。冻结帧定位不做实时追帧；质量为单帧模型/深度/掩膜拟合及同帧细化稳定性，不等同于外部精密测量。
- 快照只在当前后端进程和当前活动场景中执行，30分钟过期；模型、位姿模式、特征或手眼/TCP标定变化后拒绝沿用。新采集取代旧快照。未使用的旧快照可以查询证据，但不能恢复执行。
- 场景上下文从采集开始时建立，要到其中的物品全部放置完才结束；只规划不执行的固定抓放、失败的采集都会把它留着。留着的时候，智能体不引用场景直接抓取会被拒绝（“已有场景采集上下文”）。操作者在 8089 页面上点“抓取当前被跟踪物品”或预览时不受这个限制：这台臂遗留的场景上下文会被放弃（标为 `superseded`），依赖它的旧方案此后不能再执行（2026-10-10 起；此前页面上的点击同样被拒绝，只在消息栏显示一行提示）。
- 实例从static进入moving/held，原始位姿不再可作目标；抓取失败且已发动作记unknown。释放尝试记release_unknown，成功释放及退出后记placed_unverified并保存预测物品位姿，不能称为独立观察到放稳。全部source放置完成后结束活动场景。
- get_scene_snapshot返回最新实例状态；capture任务中的scene_snapshot只是采集完成时的历史证据。get_status不重复输出旧的实例状态，active_scene_id提供活动场景ID。
- 尚未实现完整环境/夹指碰撞、孔位占用、插入力控和释放后独立定位。冻结场景不能替代这些检查；环境实际改变时应停止并重新安排任务。

场景元数据及更新写入 `work/grasp/scenes/<scene_id>.json`，冻结RGB-D证据为同目录 `<scene_id>/frame.npz`。重启后历史场景明确标记不可执行，不自动恢复旧抓取。第5页的新任务采用新流程；已有的独立Codex/MCP会话需重启会话刷新MCP参数。
