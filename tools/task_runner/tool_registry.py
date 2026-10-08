"""Fixed function allowlist, separate from low-level device control."""

import json
from jsonschema import Draft202012Validator
from tools.robot_agent.motion_contract import GOAL_SCHEMA, VECTOR
from tools.robot_agent.scene_contract import TARGETS
from .planning_validation import READONLY_PLANNING, PlanningArgumentError

STRING = {'type': 'string', 'minLength': 1, 'maxLength': 200}
INT = {'type': 'integer', 'minimum': 0}
READS = {
    'get_status': 'status',
    'get_camera_frame': 'frame',
    'observe_global': 'global-frame',
    'get_robot_state': 'robot',
    'list_objects': 'objects',
    'get_object_pose': 'pose',
    'get_grasp_profiles': 'profiles',
    'get_object_geometry': 'object-geometry',
    'get_task_diagnostics': 'task-diagnostics',
    'get_scene_snapshot': 'scene-snapshot',
}
ACTIONS = {
    'capture_scene_snapshot': 'capture_scene_snapshot',
    'preview_grasp': 'preview_grasp',
    'grasp': 'grasp',
    'preview_place': 'preview_place',
    'place': 'place',
    'reset_arm': 'reset',
    'preview_reorient': 'preview_reorient',
    'reorient': 'reorient',
    'plan_held_object_motion': 'plan_held_object_motion',
    'plan_release': 'plan_release',
    'execute_plan': 'execute_plan',
}
MOTIONS = {'grasp', 'place', 'reset_arm', 'reorient', 'execute_plan'}
REORIENT_ARGS = {
    'object_id': STRING,
    'hold_id': STRING,
    'orientation_version': INT,
    'target_rpy_deg': {
        'type': 'array',
        'items': {'type': 'number', 'minimum': -180, 'maximum': 180},
        'minItems': 3,
        'maxItems': 3,
    },
    'position_mode': {'type': 'string', 'enum': ['fixed', 'adaptive']},
    'max_rotation_step_deg': {'type': 'number', 'minimum': 1, 'maximum': 15},
}
VISION = {
    'select_object': 'select_object',
    'start_tracking': 'start_tracking',
    'choose_tracking_target': 'choose_target',
}
DEFINITIONS = {
    'get_task_context': (
        (
            '批量只读查询：一次取得多个物品的几何及夹法，'
            '并可包含当前机器人状态、眼在手图片、场景快照。'
            'include_frame只返回眼在手画面，不是全局相机画面；'
            '全局观察使用observe_global。各读数非原子同步；运动前仍由后端核验。'
        ),
        {
            'object_ids': {
                'type': 'array',
                'items': STRING,
                'maxItems': 8,
                'uniqueItems': True,
            },
            'include_profiles': {'type': 'boolean'},
            'include_robot': {'type': 'boolean'},
            'include_frame': {'type': 'boolean'},
            'scene_id': {
                'type': ['string', 'null'], 'minLength': 1, 'maxLength': 200
            },
        },
    ),
    'get_status': (
        '查询跟踪状态、物品、候选选择、夹持记录及放置配置版本。', {}
    ),
    'get_camera_frame': (
        (
            '获取一秒内的眼在手相机图片与时间戳；相机随机械臂移动，'
            '不是全局相机。框选坐标为640×360像素。'
            '独立全局观察用observe_global。'
        ),
        {},
    ),
    'observe_global': (
        (
            '通过独立的不固定的全局相机观察全局，按需拍摄最新RGB图片，'
            '返回相机身份、时间和分辨率。用于理解场景、观察可见任务进展；'
            '未标定到基座，不能生成运动坐标、毫米距离、'
            '眼在手框选或确认精确入孔/接触/无碰撞。不替代capture_scene_snapshot'
            '，不切换跟踪相机；失败不返回旧图或另一相机图片。'
        ),
        {},
    ),
    'get_robot_state': (
        '只读查询真实关节、TCP、错误和夹爪反馈。位置米，关节度，RPY弧度。', {}
    ),
    'list_objects': ('列出已登记模型；模型编号不是场景实例编号。', {}),
    'get_object_pose': (
        '读取稳定且新鲜的物品位姿；相机光学坐标系，米，不是基座坐标。', {}
    ),
    'get_grasp_profiles': (
        (
            '读取物品夹法与各自有效力度、开度、预抓距离、抬升高度、速度及单位；'
            '只读。'
        ),
        {'object_id': STRING},
    ),
    'select_object': (
        '选择已登记模型并等待加载完成，不运动。', {'object_id': STRING}
    ),
    'start_tracking': (
        (
            '初始化选定模型的跟踪，等待可靠位姿或候选；box '
            '为最新图片中的像素框，null 表示自动检测。'
        ),
        {
            'object_id': STRING,
            'box': {
                'type': ['array', 'null'],
                'items': {'type': 'integer', 'minimum': 0, 'maximum': 640},
                'minItems': 4,
                'maxItems': 4,
            },
        },
    ),
    'choose_tracking_target': (
        (
            '结合候选及图片选定目标，并等待跟踪；使用当前 selection_id '
            '和从0开始的 index。'
        ),
        {'selection_id': INT, 'index': INT},
    ),
    'preview_grasp': (
        '等待抓取预览结束，不运动，不建立夹持关系。',
        {'object_id': STRING, 'scene_id': STRING, 'instance_id': STRING},
    ),
    'grasp': (
        (
            '引用本任务采集完成的scene_id/instance_id实际抓取并抬升，等待结束。'
            '使用已保存夹法的力度和开度，不接受覆盖。执行后查看图片。'
        ),
        {'object_id': STRING, 'scene_id': STRING, 'instance_id': STRING},
    ),
    'preview_place': (
        '等待放置预览结束；使用真实查询的夹持和配置版本，不运动。',
        {
            'hold_id': STRING,
            'orientation_version': INT,
            'placement_revision': STRING,
        },
    ),
    'place': (
        (
            '实际放置、释放、后撤并等待结束；保留当前朝向，使用已配置区域。'
            '不能替换过期身份或版本。'
        ),
        {
            'hold_id': STRING,
            'orientation_version': INT,
            'placement_revision': STRING,
        },
    ),
    'preview_reorient': (
        (
            '预览已夹持物品的目标朝向，不运动不松爪。target_rpy_deg=[X,Y,Z] '
            '是物品相对Base基座的绝对角度（度），Rz·Ry·Rx；'
            '不是相对增量或TCP朝向。5D物品仅约束该旋转对应的Z轴指向，'
            '自身绕轴转角自由，由后端搜索。fixed仅固定TCP终点位置，'
            'adaptive允许移位。引用当前物品、夹持编号和朝向版本。'
            '模型坐标决定直立方向，不假设零角度就是直立。'
        ),
        REORIENT_ARGS,
    ),
    'reorient': (
        (
            '调整已夹持物品至相对Base的绝对目标朝向并继续夹持，等待结束。'
            'target_rpy_deg=[X,Y,Z]，单位度，Rz·Ry·Rx，不是旋转增量或TCP朝向。'
            '5D物品仅约束目标Z轴指向，由后端自由搜索绕轴方位。'
            'fixed固定TCP终点位置，adaptive允许移位。请求的物品、'
            '夹持及版本必须匹配；之后使用返回的新版本放置或继续调整。'
            '后端分段最大15度，可将max_rotation_step_deg改为10或5；'
            '失败仅在recovery.can_replan=true时按新版本重新规划原目标，'
            '不重复抓取。'
        ),
        REORIENT_ARGS,
    ),
    'reset_arm': (
        (
            '实际返回观测姿态或七关节零位并等待；保持夹爪开合。'
            'zero不是编码器标定。'
        ),
        {'target': {'type': 'string', 'enum': ['observation', 'zero']}},
    ),
    'stop_robot': (
        '软件停止机械臂和夹爪，不松爪；需要明确停止意图，不用于普通状态检查。',
        {},
    ),
    'capture_scene_snapshot': (
        (
            '所有抓取前先声明本任务全部source及reference目标（不同实例用不同ins'
            'tance_id）。静止采集一份RGB-D并依次定位所有模型，固定为Base坐标，'
            '无运动。environment_static必须true；box=null使用唯一自动候选，'
            '多候选/质量差时整个快照失败，可先查看画面再指定box重采。'
            '成功后用scene_id与instance_id抓取，'
            '后续相机移动不改变静态目标位置。抓取后禁止再采；'
            '后端重启/标定改变/30分钟过期失效。'
        ),
        {
            'targets': TARGETS,
            'environment_static': {'type': 'boolean', 'const': True},
        },
    ),
    'get_scene_snapshot': (
        (
            '查询快照中的Base位姿、实例状态和有效性。backend_valid_for_executio'
            'n表示后端有效，usable_for_grasp_in_this_run才表示本任务可用；'
            '读旧快照不能替代capture_scene_snapshot。'
            '已抓起/释放的实例原位姿不可再作静态参考；'
            '特征编号仅针对估计模型坐标，不代表已校准物理编号。'
        ),
        {'scene_id': STRING},
    ),
    'get_object_geometry': (
        '读取物品模型坐标下包围盒、参考点和5D/6D模式；不是场景定位或接触标定。',
        {'object_id': STRING},
    ),
    'get_task_diagnostics': (
        (
            '读取本次后端任务关联的规划/执行诊断；'
            '旧任务缺少日志关联时返回已有结果，不猜测。'
        ),
        {'task_id': STRING},
    ),
    'plan_held_object_motion': (
        (
            '规划已夹持物品的位置/朝向，无运动。goal位置指物品参考点，'
            '坐标系在规划时冻结；朝向为该坐标系下物品绝对RPY度数，不是TCP。'
            'null分量保持前一路点值；fixed保持参考点端点，'
            'adaptive仅允许未指定位置的姿态目标移位。最多8中间点，'
            '默认15度旋转分步；scene_object坐标系须提供scene_id/reference_insta'
            'nce_id，可选reference_feature_id引用命名特征，无需实时跟踪；'
            '同时请求位置/朝向时强制先在当前物品参考点调姿，再平移；'
            'path_mode仅选择调姿方式，搬运强制linear且保持朝向，'
            '中间点不能改变最终朝向。joint调姿段中间路径及环境/夹指碰撞未完整检'
            '查。5D仅约束有方向Z轴且参考点X/Y为0。'
            '返回方案编号后用execute_plan，5分钟有效，'
            '配置/起点/夹持改变需重规划。'
        ),
        {
            'object_id': STRING,
            'hold_id': STRING,
            'orientation_version': INT,
            'goal': GOAL_SCHEMA,
        },
    ),
    'plan_release': (
        (
            '规划统一放置：先在当前物品参考点调到最终姿态，'
            '再保持姿态直线到最终目标沿Base +Z正上方100毫米（配置值），'
            '再保持姿态直线下降到最终目标并松爪。goal=null以当前物品位姿作最终'
            '目标，仍先上移再下降。规划不运动。retreat_vector_base_m为松爪后TCP'
            '在Base的位移向量，长度最多0.3m，零向量不退出。只能fixed位置策略。'
            'execute_plan将松开夹爪；目标是释放前物品位姿，'
            '不保证释放后稳定停留，不提供完整支撑/夹指/环境碰撞验证。'
        ),
        {
            'object_id': STRING,
            'hold_id': STRING,
            'orientation_version': INT,
            'goal': {**GOAL_SCHEMA, 'type': ['object', 'null']},
            'retreat_vector_base_m': {
                **VECTOR,
                'items': {'type': 'number', 'minimum': -.3, 'maximum': .3},
            },
        },
    ),
    'execute_plan': (
        (
            '执行已规划且不可修改的方案。release类型会松爪，'
            '须符合用户释放意图；只接受plan_id，不接受替换坐标。方案一次使用，'
            '绑定当前夹持/模型/标定/配置及起点。成功持物移动后版本加1；'
            '失败只有recovery.can_replan=true时可生成新方案继续，'
            '旧plan_id不能重放。'
        ),
        {'plan_id': STRING},
    ),
    'finish_task': (
        (
            '结束本次任务。从工具结果的 evidence_call_id 字段复制证据编号，'
            '不要使用后端 task_id 或编造编号。succeeded '
            '必须引用全部真实成功动作证据；answered 仅说明已提供信息；blocked '
            '表示无法完成。summary 区分控制器事实与视觉判断。'
        ),
        {
            'outcome': {
                'type': 'string', 'enum': ['succeeded', 'answered', 'blocked']
            },
            'summary': {'type': 'string', 'minLength': 1, 'maxLength': 2000},
            'evidence_call_ids': {
                'type': 'array',
                'items': STRING,
                'maxItems': 32,
                'uniqueItems': True,
            },
        },
    ),
}


def schema(name):
    description, properties = DEFINITIONS[name]
    properties = dict(properties)
    if name != 'finish_task':
        properties['progress_message'] = {
            'type': 'string',
            'minLength': 1,
            'maxLength': 600,
            'description': (
                '面向用户的一两句过程说明：已确认的进展，以及本次准备做什么。'
                '只说明行动，不展开内部推理；未执行的动作不能说已完成。'
            ),
        }
    return {
        'type': 'function',
        'name': name,
        'description': description,
        'strict': True,
        'parameters': {
            'type': 'object',
            'properties': properties,
            'required': list(properties),
            'additionalProperties': False,
        },
    }


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
            return {
                k: supported(v)
                for k, v in value.items()
                if k not in ('minLength', 'maxLength', 'uniqueItems')
            }
        if isinstance(value, list):
            return [supported(v) for v in value]
        return value

    return [supported(schema(n)) for n in DEFINITIONS if n not in excluded]


def validate(call, mode):
    if (
        call.get('type') != 'function_call'
        or not isinstance(call.get('call_id'), str)
        or not 1 <= len(call['call_id']) <= 200
    ):
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

    args = json.loads(
        raw,
        object_pairs_hook=pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(
            ValueError('非有限数值')
        ),
    )
    validator = Draft202012Validator(schema(name)['parameters'])
    errors = list(validator.iter_errors(args))
    if errors:
        if name in READONLY_PLANNING and isinstance(args, dict):
            fields = [
                {
                    'field': (
                        '.'.join(str(p)[:80] for p in e.absolute_path)
                        or 'arguments'
                    ),
                    'rule': e.validator,
                    'expected': e.validator_value,
                }
                for e in errors[:5]
            ]
            raise PlanningArgumentError(
                name,
                args,
                {
                    'code': 'INVALID_ARGUMENTS',
                    'message': (
                        '规划参数不符合工具Schema；请按fields中的字段、'
                        '类型或范围修正后重新规划。'
                    ),
                    'fields': fields,
                },
            )
        raise ValueError('工具参数不符合严格定义')
    if name == 'start_tracking' and args['box'] is not None:
        x0, y0, x1, y1 = args['box']
        if not (0 <= x0 < x1 <= 640 and 0 <= y0 < y1 <= 360):
            raise ValueError('框选坐标超出640×360画面')
    return name, args
