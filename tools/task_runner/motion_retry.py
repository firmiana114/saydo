"""Per-task retry cycle shared by both motion APIs; changing tool/ID cannot reset it."""
import copy
import math
from .planning_validation import READONLY_PLANNING


class RetryLimit(ValueError):pass


def rotation(rpy):
    x,y,z=map(math.radians,rpy);cx,sx,cy,sy,cz,sz=math.cos(x),math.sin(x),math.cos(y),math.sin(y),math.cos(z),math.sin(z)
    return [[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]]


def same_orientation(a,b,mode):
    cosine=sum(a[i][2]*b[i][2] for i in range(3)) if mode=='5d' else (sum(a[i][j]*b[i][j] for i in range(3) for j in range(3))-1)/2
    return math.degrees(math.acos(max(-1,min(1,cosine))))<=2


class MotionRetry:
    def __init__(self):self.reset()
    def reset(self):self.hold_id=None;self.planning=0;self.execution=0;self.pending=None;self.plans={}
    def snapshot(self):return {'planning_attempts':self.planning,'planning_limit':5,'execution_attempts':self.execution,'execution_limit':3,'recovery_pending':self.pending is not None,'planning_remaining':max(0,5-self.planning),'execution_remaining':max(0,3-self.execution)}
    def before(self,name,args,held):
        planning=name in READONLY_PLANNING or name=='reorient'
        executing=name in ('reorient','execute_plan')
        if self.pending and self.execution>=3 and (planning or executing):raise RetryLimit('同一目标已尝试执行3次，达到上限')
        if self.pending and name in ('grasp','place','reset_arm','plan_release'):raise RetryLimit('调姿失败尚未恢复；只能观察或重新规划原目标，不能跳过后续核验')
        if name=='execute_plan' and self.pending:
            plan=self.plans.get(args['plan_id'])
            if not plan or plan.get('kind')!='held_object_motion':raise RetryLimit('失败后必须重新规划持物目标，不能重放旧方案或直接释放')
        if not planning and not executing:return
        ident=(held or {}).get('hold_id')
        if self.hold_id is not None and self.hold_id!=ident:raise RetryLimit('重试期间夹持身份变化')
        self.hold_id=ident
        if self.pending and name=='reorient':
            if self.pending.get('tcp_position_required') and math.dist(self.pending['actual_tcp_position_m'],self.pending['tcp_position_m'])>.002:raise RetryLimit('停止点已偏离原固定TCP目标，请用通用工具规划回原目标')
            if self.pending.get('position_required'):raise RetryLimit('原目标包含指定位置，请使用通用持物规划工具')
            if self.pending['position_mode']=='fixed' and args['position_mode']!='fixed':raise RetryLimit('不能放宽原目标的位置约束')
            if not same_orientation(rotation(args['target_rpy_deg']),self.pending['base_T_object_target'],self.pending['pose_mode']):raise RetryLimit('不能改变原最终朝向以绕过重试预算')
        if planning:
            if self.planning>=5:raise RetryLimit('同一目标已规划5次，达到上限')
            self.planning+=1
        if executing:
            if self.execution>=3:raise RetryLimit('同一目标已尝试执行3次，达到上限')
            self.execution+=1

    def plan(self,plan):
        if self.pending:
            goal=plan.get('goal',{});target=plan.get('base_T_object_target')
            if not target or not same_orientation(target,self.pending['base_T_object_target'],self.pending['pose_mode']):raise RetryLimit('新方案改变了原最终朝向')
            mode=goal.get('position_mode',plan.get('position_mode'))
            if self.pending['position_mode']=='fixed' and mode!='fixed':raise RetryLimit('新方案放宽了原位置约束')
            if self.pending.get('tcp_position_required'):
                attachment=self.pending['object_T_tcp'];tcp=[target[i][3]+sum(target[i][j]*attachment[j][3] for j in range(3)) for i in range(3)]
                if math.dist(tcp,self.pending['tcp_position_m'])>.002:raise RetryLimit('新方案改变了原固定TCP目标位置')
            if self.pending.get('position_required'):
                p=self.pending['reference_point_object_m'];old=self.pending['base_T_object_target']
                distance=math.sqrt(sum((target[i][3]+sum(target[i][j]*p[j] for j in range(3))-old[i][3]-sum(old[i][j]*p[j] for j in range(3)))**2 for i in range(3)))
                if distance>.002:raise RetryLimit('新方案改变了原目标位置')
        if plan.get('plan_id'):self.plans[plan['plan_id']]=copy.deepcopy(plan)

    def recover(self,recovery):
        goal=recovery.get('goal_constraint')
        if not isinstance(goal,dict) or not goal.get('base_T_object_target'):raise RetryLimit('缺少原目标约束，不能自动恢复')
        self.pending=copy.deepcopy(goal);self.plans.clear()
        if recovery.get('motion_sent') is False:self.execution=max(0,self.execution-1)
