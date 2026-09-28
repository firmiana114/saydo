"""Shared, dependency-free schemas for object motion tools."""
import copy
import math

VECTOR = {'type':'array','items':{'type':'number','minimum':-2,'maximum':2},'minItems':3,'maxItems':3}
RPY = {'type':'array','items':{'type':'number','minimum':-180,'maximum':180},'minItems':3,'maxItems':3}
WAYPOINT = {'type':'object','properties':{'position_m':{**VECTOR,'type':['array','null']},
             'rpy_deg':{**RPY,'type':['array','null']}},'required':['position_m','rpy_deg'],'additionalProperties':False}
GOAL_PROPERTIES = {
    'reference_frame':{'type':'string','enum':['base','camera','tcp','held_object','tracked_object','scene_object']},
    'scene_id':{'type':['string','null'],'description':'仅scene_object使用：填写capture_scene_snapshot返回的scene_id；其他坐标系必须为null。'},
    'reference_instance_id':{'type':['string','null'],'description':'仅scene_object使用：填写该快照中的实例ID，不是模型ID；其他坐标系必须为null。'},
    'reference_feature_id':{'type':['string','null'],'description':'仅scene_object可填写命名特征ID；null表示实例原点。其他坐标系必须为null。'},
    'reference_object_id':{'type':['string','null'],'description':'仅tracked_object使用模型ID。scene_object必须为null，改用scene_id和reference_instance_id；base/camera/tcp/held_object也必须为null。'},
    'reference_point_object_m':VECTOR,
    'target_position_m':{**VECTOR,'type':['array','null']},
    'target_rpy_deg':{**RPY,'type':['array','null']},
    'path_mode':{'type':'string','enum':['joint','linear']},
    'position_mode':{'type':'string','enum':['fixed','adaptive']},
    'max_rotation_step_deg':{'type':'number','minimum':1,'maximum':15},
    'waypoints':{'type':'array','items':WAYPOINT,'maxItems':8},
}
GOAL_SCHEMA = {'type':'object','properties':GOAL_PROPERTIES,'required':list(GOAL_PROPERTIES),'additionalProperties':False}
DEFAULT_GOAL = {'reference_frame':'base','scene_id':None,'reference_instance_id':None,'reference_feature_id':None,'reference_object_id':None,'reference_point_object_m':[0.,0.,0.],
                'target_position_m':None,'target_rpy_deg':None,'path_mode':'joint','position_mode':'fixed',
                'max_rotation_step_deg':15.,'waypoints':[]}


class GoalArgumentError(ValueError):
    def __init__(self,field,message,expected):
        self.field,self.expected=field,expected
        super().__init__(message)


def vector(value, name, bound=2.):
    if (not isinstance(value,list) or len(value)!=3
            or any(type(x) not in (int,float) or not math.isfinite(x) or abs(x)>bound for x in value)):
        raise ValueError(f'{name}须为三个有限数值，绝对值不超过{bound}')
    return [float(x) for x in value]


def identity(args):
    if any(not isinstance(args.get(k),str) or not args[k].strip() or len(args[k])>200 for k in ('object_id','hold_id')):
        raise ValueError('物品和夹持编号无效')
    if type(args.get('orientation_version')) is not int or args['orientation_version']<0:
        raise ValueError('夹持版本必须为非负整数')


def checked_goal(value, release=False):
    if value is None and release:value={}
    if not isinstance(value,dict) or set(value)-set(DEFAULT_GOAL):raise ValueError('物品运动目标字段无效')
    result={**copy.deepcopy(DEFAULT_GOAL),**copy.deepcopy(value)}
    for key in ('reference_frame','path_mode','position_mode'):
        if result[key] not in GOAL_PROPERTIES[key]['enum']:raise ValueError(f'{key}无效')
    oid=result['reference_object_id']
    if result['reference_frame']=='tracked_object':
        if not isinstance(oid,str) or not oid.strip() or len(oid)>200:raise GoalArgumentError('goal.reference_object_id','reference_frame=tracked_object时须提供非空reference_object_id（模型ID）','nonempty model ID')
    elif oid is not None:raise GoalArgumentError('goal.reference_object_id','只有tracked_object坐标系接受reference_object_id；当前坐标系必须将goal.reference_object_id设为null。scene_object通过scene_id和reference_instance_id定位，不填写模型ID。',None)
    scene_keys=('scene_id','reference_instance_id','reference_feature_id')
    if result['reference_frame']=='scene_object':
        for k in scene_keys[:2]:
            if not isinstance(result[k],str) or not result[k] or len(result[k])>100:raise GoalArgumentError('goal.'+k,'reference_frame=scene_object时须填写'+k+'，请从场景快照读取，不能用模型ID替代。','nonempty snapshot/instance ID')
        if result['reference_feature_id'] is not None and (not isinstance(result['reference_feature_id'],str) or not result['reference_feature_id'] or len(result['reference_feature_id'])>100):raise ValueError('特征编号无效')
    elif any(result[k] is not None for k in scene_keys):raise GoalArgumentError('goal','只有scene_object参考系接受快照字段；其他坐标系的scene_id、reference_instance_id、reference_feature_id须全部为null。',{k:None for k in scene_keys})
    result['reference_point_object_m']=vector(result['reference_point_object_m'],'reference_point_object_m')
    if result['target_position_m'] is not None:result['target_position_m']=vector(result['target_position_m'],'target_position_m')
    if result['target_rpy_deg'] is not None:result['target_rpy_deg']=vector(result['target_rpy_deg'],'target_rpy_deg',180.)
    step=result['max_rotation_step_deg']
    if type(step) not in (int,float) or not math.isfinite(step) or not 1<=step<=15:raise ValueError('旋转步长须为1–15度')
    rows=result['waypoints']
    if not isinstance(rows,list) or len(rows)>8:raise ValueError('中间路点最多8个')
    for row in rows:
        if not isinstance(row,dict) or set(row)!={'position_m','rpy_deg'}:raise ValueError('路点须包含position_m和rpy_deg（可为null）')
        if row['position_m'] is None and row['rpy_deg'] is None:raise ValueError('路点不能同时省略位置和朝向')
        if row['position_m'] is not None:row['position_m']=vector(row['position_m'],'waypoint position_m')
        if row['rpy_deg'] is not None:row['rpy_deg']=vector(row['rpy_deg'],'waypoint rpy_deg',180.)
    has_position=result['target_position_m'] is not None or any(r['position_m'] is not None for r in rows)
    if result['position_mode']=='adaptive' and (has_position or release):raise ValueError('自适应位置仅用于未指定位置的持物姿态调整；释放必须使用确定位置')
    if not release and not rows and result['target_position_m'] is None and result['target_rpy_deg'] is None:
        raise ValueError('至少指定物品目标位置、朝向或中间路点')
    return result
