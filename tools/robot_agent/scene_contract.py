"""Dependency-free scene capture inputs shared by both agent frontends."""
import re
BOX={'type':['array','null'],'items':{'type':'integer','minimum':0,'maximum':640},'minItems':4,'maxItems':4}
TARGETS={'type':'array','minItems':1,'maxItems':8,'items':{'type':'object','properties':{
    'instance_id':{'type':'string'},'object_id':{'type':'string'},'role':{'type':'string','enum':['source','reference']},'box':BOX},
    'required':['instance_id','object_id','role','box'],'additionalProperties':False}}


def checked_targets(value):
    if not isinstance(value,list) or not 1<=len(value)<=8:raise ValueError('场景须包含1至8个目标')
    result=[];ids=set()
    for row in value:
        if not isinstance(row,dict) or set(row)!={'instance_id','object_id','role','box'}:raise ValueError('场景目标字段无效')
        if any(not isinstance(row[k],str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,100}',row[k]) for k in ('instance_id','object_id')):raise ValueError('场景实例/模型编号无效')
        if row['instance_id'] in ids:raise ValueError('场景实例编号重复')
        if row['role'] not in ('source','reference'):raise ValueError('场景角色须为source或reference')
        box=row['box']
        if box is not None:
            if not isinstance(box,list) or len(box)!=4 or any(type(n) is not int for n in box):raise ValueError('框选坐标须为四个整数')
            x0,y0,x1,y1=box
            if not (0<=x0<x1<=640 and 0<=y0<y1<=360 and min(x1-x0,y1-y0)>=8):raise ValueError('框选范围无效')
        ids.add(row['instance_id']);result.append(dict(row))
    return result
