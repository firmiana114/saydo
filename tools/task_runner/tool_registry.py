"""Fixed function allowlist, separate from low-level device control."""
import json
from jsonschema import Draft202012Validator
from tools.robot_agent.motion_contract import GOAL_SCHEMA, VECTOR

STRING = {'type': 'string', 'minLength': 1, 'maxLength': 200}
INT = {'type': 'integer', 'minimum': 0}
READS = {'get_status': 'status', 'get_camera_frame': 'frame', 'get_robot_state': 'robot',
         'list_objects': 'objects', 'get_object_pose': 'pose', 'get_grasp_profiles': 'profiles', 'get_object_geometry':'object-geometry', 'get_task_diagnostics':'task-diagnostics'}
ACTIONS = {'preview_grasp': 'preview_grasp', 'grasp': 'grasp', 'preview_place': 'preview_place', 'place': 'place', 'reset_arm': 'reset',
           'preview_reorient': 'preview_reorient', 'reorient': 'reorient', 'plan_held_object_motion':'plan_held_object_motion', 'plan_release':'plan_release', 'execute_plan':'execute_plan'}
MOTIONS = {'grasp', 'place', 'reset_arm', 'reorient', 'execute_plan'}
REORIENT_ARGS = {'object_id': STRING, 'hold_id': STRING, 'orientation_version': INT,
    'target_rpy_deg': {'type': 'array', 'items': {'type': 'number', 'minimum': -180, 'maximum': 180}, 'minItems': 3, 'maxItems': 3},
    'position_mode': {'type': 'string', 'enum': ['fixed', 'adaptive']}}
VISION = {'select_object': 'select_object', 'start_tracking': 'start_tracking', 'choose_tracking_target': 'choose_target'}
DEFINITIONS = {
    'get_status': ('查询跟踪状态、物品、候选选择、夹持记录及放置配置版本。', {}),
    'get_camera_frame': ('获取一秒内的真实相机图片与时间戳；框选坐标为 640×360 像素。', {}),
    'get_robot_state': ('只读查询真实关节、TCP、错误和夹爪反馈。位置米，关节度，RPY弧度。', {}),
    'list_objects': ('列出已登记模型；模型编号不是场景实例编号。', {}),
    'get_object_pose': ('读取稳定且新鲜的物品位姿；相机光学坐标系，米，不是基座坐标。', {}),
    'get_grasp_profiles': ('读取物品夹法与各自有效力度、开度、预抓距离、抬升高度、速度及单位；只读。', {'object_id': STRING}),
    'select_object': ('选择已登记模型并等待加载完成，不运动。', {'object_id': STRING}),
    'start_tracking': ('初始化选定模型的跟踪，等待可靠位姿或候选；box 为最新图片中的像素框，null 表示自动检测。',
        {'object_id': STRING, 'box': {'type': ['array', 'null'], 'items': {'type': 'integer', 'minimum': 0, 'maximum': 640}, 'minItems': 4, 'maxItems': 4}}),
    'choose_tracking_target': ('结合候选及图片选定目标，并等待跟踪；使用当前 selection_id 和从0开始的 index。', {'selection_id': INT, 'index': INT}),
    'preview_grasp': ('等待抓取预览结束，不运动，不建立夹持关系。', {'object_id': STRING}),
    'grasp': ('实际抓取并抬升，等待结束。使用已保存夹法的力度和开度，不接受覆盖。执行后查看图片。', {'object_id': STRING}),
    'preview_place': ('等待放置预览结束；使用真实查询的夹持和配置版本，不运动。', {'hold_id': STRING, 'orientation_version': INT, 'placement_revision': STRING}),
    'place': ('实际放置、释放、后撤并等待结束；保留当前朝向，使用已配置区域。不能替换过期身份或版本。', {'hold_id': STRING, 'orientation_version': INT, 'placement_revision': STRING}),
    'preview_reorient': ('预览已夹持物品的目标朝向，不运动不松爪。target_rpy_deg=[X,Y,Z] 是物品相对Base基座的绝对角度（度），Rz·Ry·Rx；不是相对增量或TCP朝向。5D物品仅约束该旋转对应的Z轴指向，自身绕轴转角自由，由后端搜索。fixed仅固定TCP终点位置，adaptive允许移位。引用当前物品、夹持编号和朝向版本。模型坐标决定直立方向，不假设零角度就是直立。', REORIENT_ARGS),
    'reorient': ('调整已夹持物品至相对Base的绝对目标朝向并继续夹持，等待结束。target_rpy_deg=[X,Y,Z]，单位度，Rz·Ry·Rx，不是旋转增量或TCP朝向。5D物品仅约束目标Z轴指向，由后端自由搜索绕轴方位。fixed固定TCP终点位置，adaptive允许移位。请求的物品、夹持及版本必须匹配；之后使用返回的新版本放置或继续调整。失败不自动重试。', REORIENT_ARGS),
    'reset_arm': ('实际返回观测姿态或七关节零位并等待；保持夹爪开合。zero不是编码器标定。', {'target': {'type': 'string', 'enum': ['observation', 'zero']}}),
    'stop_robot': ('软件停止机械臂和夹爪，不松爪；需要明确停止意图，不用于普通状态检查。', {}),
    'get_object_geometry': ('读取物品模型坐标下包围盒、参考点和5D/6D模式；不是场景定位或接触标定。', {'object_id': STRING}),
    'get_task_diagnostics': ('读取本次后端任务关联的规划/执行诊断；旧任务缺少日志关联时返回已有结果，不猜测。', {'task_id': STRING}),
    'plan_held_object_motion': ('规划已夹持物品的位置/朝向，无运动。goal位置指物品参考点，坐标系在规划时冻结；朝向为该坐标系下物品绝对RPY度数，不是TCP。null分量保持前一路点值；fixed保持参考点端点，adaptive仅允许未指定位置的姿态目标移位。最多8中间点，默认15度旋转分步；linear为TCP直线子段，joint中间路径及环境/夹指碰撞未完整检查。5D仅约束有方向Z轴且参考点X/Y为0。返回方案编号后用execute_plan，5分钟有效，配置/起点/夹持改变需重规划。',
        {'object_id':STRING,'hold_id':STRING,'orientation_version':INT,'goal':GOAL_SCHEMA}),
    'plan_release': ('规划在当前物品位姿(goal=null)或目标位姿释放并退出，不运动。retreat_vector_base_m为松爪后TCP在Base的位移向量，长度最多0.3m，零向量不退出。只能fixed位置策略。execute_plan将松开夹爪；目标是释放前物品位姿，不保证释放后稳定停留，不提供完整支撑/夹指/环境碰撞验证。',
        {'object_id':STRING,'hold_id':STRING,'orientation_version':INT,'goal':{**GOAL_SCHEMA,'type':['object','null']},
         'retreat_vector_base_m':{**VECTOR,'items':{'type':'number','minimum':-.3,'maximum':.3}}}),
    'execute_plan': ('执行已规划且不可修改的方案。release类型会松爪，须符合用户释放意图；只接受plan_id，不接受替换坐标。方案一次使用，绑定当前夹持/模型/标定/配置及起点。成功持物移动后版本加1；失败不得自动续跑或换编号重放。', {'plan_id':STRING}),
    'finish_task': ('结束本次任务。从工具结果的 evidence_call_id 字段复制证据编号，不要使用后端 task_id 或编造编号。succeeded 必须引用全部真实成功动作证据；answered 仅说明已提供信息；blocked 表示无法完成。summary 区分控制器事实与视觉判断。',
        {'outcome': {'type': 'string', 'enum': ['succeeded', 'answered', 'blocked']},
         'summary': {'type': 'string', 'minLength': 1, 'maxLength': 2000},
         'evidence_call_ids': {'type': 'array', 'items': STRING, 'maxItems': 32, 'uniqueItems': True}}),
}


def schema(name):
    description, properties = DEFINITIONS[name]
    properties = dict(properties)
    if name != 'finish_task':
        properties['progress_message'] = {'type': 'string', 'minLength': 1, 'maxLength': 600,
            'description': '面向用户的一两句过程说明：已确认的进展，以及本次准备做什么。只说明行动，不展开内部推理；未执行的动作不能说已完成。'}
    return {'type': 'function', 'name': name, 'description': description, 'strict': True,
        'parameters': {'type': 'object', 'properties': properties, 'required': list(properties), 'additionalProperties': False}}


def schemas(mode):
    excluded = set()
    if mode != 'execute':
        excluded |= MOTIONS | {'stop_robot'}
    if mode == 'read_only':
        excluded |= set(ACTIONS) | set(VISION)
    def supported(value):
        # Keep length/uniqueness checks in our local validator. Providers only
        # implement a subset of JSON Schema for constrained generation.
        if isinstance(value, dict):
            return {k: supported(v) for k, v in value.items() if k not in ('minLength', 'maxLength', 'uniqueItems')}
        if isinstance(value, list):
            return [supported(v) for v in value]
        return value
    return [supported(schema(n)) for n in DEFINITIONS if n not in excluded]


def validate(call, mode):
    if call.get('type') != 'function_call' or not isinstance(call.get('call_id'), str) or not 1 <= len(call['call_id']) <= 200:
        raise ValueError('无效的模型工具调用编号')
    name = call.get('name')
    if name not in {s['name'] for s in schemas(mode)}:
        raise ValueError('未知或当前模式不允许的工具')
    raw = call.get('arguments')
    if not isinstance(raw, str) or len(raw) > 10000:
        raise ValueError('工具参数格式或大小无效')
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError('工具参数含重复字段')
            result[key] = value
        return result
    args = json.loads(raw, object_pairs_hook=pairs, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('非有限数值')))
    if not Draft202012Validator(schema(name)['parameters']).is_valid(args):
        raise ValueError('工具参数不符合严格定义')
    if name == 'start_tracking' and args['box'] is not None:
        x0, y0, x1, y1 = args['box']
        if not (0 <= x0 < x1 <= 640 and 0 <= y0 < y1 <= 360):
            raise ValueError('框选坐标超出640×360画面')
    return name, args
