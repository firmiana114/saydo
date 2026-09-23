"""Fixed function allowlist, separate from low-level device control."""
import json
from jsonschema import Draft202012Validator

STRING = {'type': 'string', 'minLength': 1, 'maxLength': 200}
INT = {'type': 'integer', 'minimum': 0}
READS = {'get_status': 'status', 'get_camera_frame': 'frame', 'get_robot_state': 'robot',
         'list_objects': 'objects', 'get_object_pose': 'pose', 'get_grasp_profiles': 'profiles'}
ACTIONS = {'preview_grasp': 'preview_grasp', 'grasp': 'grasp', 'preview_place': 'preview_place', 'place': 'place', 'reset_arm': 'reset'}
MOTIONS = {'grasp', 'place', 'reset_arm'}
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
    'reset_arm': ('实际返回观测姿态或七关节零位并等待；保持夹爪开合。zero不是编码器标定。', {'target': {'type': 'string', 'enum': ['observation', 'zero']}}),
    'stop_robot': ('软件停止机械臂和夹爪，不松爪；需要明确停止意图，不用于普通状态检查。', {}),
    'finish_task': ('结束本次任务。succeeded 必须有本轮真实成功动作且引用全部动作证据；answered 仅说明已提供信息，不能声称实物任务完成；blocked 表示无法完成。summary 区分控制器事实与视觉判断。',
        {'outcome': {'type': 'string', 'enum': ['succeeded', 'answered', 'blocked']},
         'summary': {'type': 'string', 'minLength': 1, 'maxLength': 2000},
         'evidence_call_ids': {'type': 'array', 'items': STRING, 'maxItems': 32, 'uniqueItems': True}}),
}


def schema(name):
    description, properties = DEFINITIONS[name]
    return {'type': 'function', 'name': name, 'description': description, 'strict': True,
        'parameters': {'type': 'object', 'properties': properties, 'required': list(properties), 'additionalProperties': False}}


def schemas(mode):
    excluded = set()
    if mode != 'execute':
        excluded |= MOTIONS | {'stop_robot'}
    if mode == 'read_only':
        excluded |= set(ACTIONS) | set(VISION)
    return [schema(n) for n in DEFINITIONS if n not in excluded]


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
