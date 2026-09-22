"""Visual grasp authoring and selection; editor mutations never connect to hardware."""
import copy
import fcntl
import json
import subprocess
import time
import uuid

import numpy as np
import yaml

from grasp_geometry import plan_grasp, workspace_blockers, rotation_error, controller_pose
from grasp_library import (GraspLibrary, MAX_GRASPS, revision, checked_profile, checked_overrides,
                           point_limits, binding_error, effective_config)
from reorientation_geometry import model_geometry
from reorientation_visual import model_appearance


class GraspProfilesMixin:
    def grasp_library(self):
        return GraspLibrary(self.root, getattr(self.tracker, 'models_root', None))

    def grasp_editor_model(self, object_id):
        geometry = model_geometry(self.tracker.models_root, object_id)
        low, high = point_limits(geometry['vertices'])
        geometry.update(initial_rotation=np.eye(3).tolist(), holding=False,
                        pivot_object_m=np.asarray(geometry['vertices']).mean(axis=0).tolist(),
                        point_min_m=low.tolist(), point_max_m=high.tolist())
        try:
            geometry['appearance'] = model_appearance(self.tracker.models_root, object_id)
        except (ValueError, OSError, TypeError) as error:
            geometry['appearance'] = {'status':'unavailable', 'error':str(error)}
        return geometry

    def grasp_profiles(self, object_id):
        geometry = model_geometry(self.tracker.models_root, object_id)
        config = json.loads(self.config_path.read_text())
        tool = yaml.safe_load((self.root/config['tool_file']).read_text())['controller_readback']['tool_response_raw']
        handeye = yaml.safe_load((self.root/config['handeye_file']).read_text())['flange_T_color_optical_camera']
        value = self.grasp_library().object(object_id)
        entries = []
        for entry in value['grasps']:
            row = copy.deepcopy(entry)
            row['binding_error'] = binding_error(entry, geometry['model_sha256'], tool, handeye)
            entries.append(row)
        return {**value, 'grasps':entries, 'revision':revision(value), 'object_id':object_id,
                'object_name':geometry['object_name'], 'model_sha256':geometry['model_sha256'], 'maximum':MAX_GRASPS,
                'changed_at':self.status.get('grasp_library_changed_at'),
                'defaults':{'pregrasp_distance_m':config['pregrasp_distance_m'],
                            'open_position':config['gripper']['open_position'], 'force_target_raw':self.force_config()['value']}}

    def edit_grasp_profiles(self, request):
        if not isinstance(request, dict):
            raise ValueError('夹法编辑请求无效')
        object_id = request.get('object_id');action = request.get('action')
        if action not in ('save', 'duplicate', 'delete', 'select'):
            raise ValueError('未知的夹法操作')
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('机械臂任务进行中，不能修改夹法库')
            directory = self.root/'work/grasp';directory.mkdir(parents=True, exist_ok=True)
            with (directory/'robot.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                geometry = model_geometry(self.tracker.models_root, object_id)
                library = self.grasp_library();value = library.object(object_id);before = revision(value)
                if request.get('revision') != before:
                    raise ValueError('夹法已变化，请重新载入后操作')
                selected = next((g for g in value['grasps'] if g['id'] == request.get('id')), None)
                if action in ('duplicate','delete') and selected is None:
                    raise ValueError('夹法不存在')
                if action == 'select':
                    mode = request.get('selection_mode')
                    if mode not in ('auto','manual') or (mode=='manual' and (selected is None or not selected['enabled'])):
                        raise ValueError('手动模式必须选择一套已启用的夹法')
                    value.update(selection_mode=mode, selected_id=selected['id'] if mode=='manual' else None)
                elif action == 'delete':
                    value['grasps'].remove(selected)
                    if value['selected_id'] == selected['id']:
                        value.update(selection_mode='auto', selected_id=None)
                elif action == 'duplicate':
                    if len(value['grasps']) >= MAX_GRASPS:
                        raise ValueError('夹法数量已达上限')
                    selected = copy.deepcopy(selected)
                    selected.update(id=uuid.uuid4().hex, name=(selected['name']+' 副本')[:60], tested_successfully=False)
                    value['grasps'].append(selected)
                else:
                    data = request.get('grasp')
                    if not isinstance(data, dict):
                        raise ValueError('缺少夹法内容')
                    name = data.get('name')
                    if not isinstance(name,str) or not name.strip() or len(name.strip()) > 60:
                        raise ValueError('夹法名称须为 1–60 个字符')
                    if type(data.get('enabled')) is not bool or type(data.get('priority')) is not int or not 0 <= data['priority'] <= 100:
                        raise ValueError('夹法启用状态或优先级无效（0–100）')
                    if request.get('id') is not None and selected is None:
                        raise ValueError('夹法不存在，请重新载入')
                    profile = checked_profile(data.get('profile'));overrides = checked_overrides(data.get('overrides',{}))
                    low,high = point_limits(geometry['vertices']);point = np.asarray(profile['grasp_point_object_m'])
                    if np.any(point<low) or np.any(point>high):
                        raise ValueError('抓取中心超出模型附近允许的编辑范围')
                    config = json.loads(self.config_path.read_text())
                    tool = yaml.safe_load((self.root/config['tool_file']).read_text())['controller_readback']['tool_response_raw']
                    if request.get('model_sha256') != geometry['model_sha256']:
                        raise ValueError('编辑器模型已变化，请重新载入')
                    if selected is not None and binding_error(selected, geometry['model_sha256'], tool):
                        raise ValueError('旧夹法的模型或工具绑定已变化，请新建夹法')
                    if selected is None and len(value['grasps']) >= MAX_GRASPS:
                        raise ValueError('夹法数量已达上限')
                    entry = {'id':selected['id'] if selected else uuid.uuid4().hex, 'name':name.strip(),
                             'enabled':data['enabled'], 'priority':data['priority'], 'profile':profile, 'overrides':overrides,
                             'model_sha256':geometry['model_sha256'], 'tool_response':tool, 'flange_T_camera':None,
                             'source':'visual', 'recorded_at':time.time(), 'tested_successfully':False}
                    if selected is not None:
                        # Metadata edits must not silently rebind a taught pose to new calibration.
                        if profile == checked_profile(selected['profile']):
                            entry.update(source=selected.get('source','taught'), flange_T_camera=selected.get('flange_T_camera'))
                            if overrides == selected.get('overrides',{}):
                                entry['tested_successfully'] = selected.get('tested_successfully',False)
                                if 'last_success_at' in selected:
                                    entry['last_success_at'] = selected['last_success_at']
                        value['grasps'][value['grasps'].index(selected)] = entry
                    else:
                        value['grasps'].append(entry)
                    selected = entry
                    if not entry['enabled'] and value['selected_id']==entry['id']:
                        value.update(selection_mode='auto',selected_id=None)
                library.write_object(object_id,value,before)
                records = library.object_directory(object_id)/'grasp_profile_commands';records.mkdir(parents=True,exist_ok=True)
                (records/f'{time.time_ns()}.json').write_text(json.dumps({'action':action,'object_id':object_id,
                    'id':selected['id'] if selected else None,'before_revision':before,'after_revision':revision(value),
                    'motion_sent':False},ensure_ascii=False,indent=2))
                self.status.update(plan=None,grasp_library_changed_at=time.time())
            return {**self.grasp_profiles(object_id), 'saved_id':selected['id'] if selected else None, 'motion_sent':False}

    def select_grasp_candidate(self, config, observation, tool, handeye, base_object, arm, report):
        library = self.grasp_library();value = library.object(observation['object_id'])
        entries = value['grasps']
        # Config-only installations keep their existing single-profile behavior.
        if not entries and not library.has_object(observation['object_id']):
            entries = [{'id':'config','name':'配置夹法','enabled':True,'priority':0,'profile':copy.deepcopy(config['grasp_profile']),
                        'model_sha256':observation.get('model_sha256'),'tool_response':tool['controller_readback']['tool_response_raw']}]
        if value['selection_mode']=='manual':
            entries = [g for g in entries if g['id']==value['selected_id']]
        entries = sorted((g for g in entries if g['enabled']),key=lambda g:(-g['priority'],g['id']))
        if not entries:
            raise ValueError('没有已启用的夹法，请在夹法编辑器中新增或启用')
        results = report['grasp_candidates'] = [];passed = [];deadline = time.monotonic()+12
        for entry in entries:
            self.check_cancel()
            row = {'id':entry['id'],'name':entry['name'],'priority':entry['priority'],'passed':False};results.append(row)
            try:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('候选检查时间预算已用完')
                error = binding_error(entry,observation.get('model_sha256'),tool['controller_readback']['tool_response_raw'],handeye['flange_T_color_optical_camera'])
                if error:
                    raise ValueError(error)
                effective = effective_config(config,entry)
                profile = {**copy.deepcopy(config['grasp_profile']),**entry['profile']}
                plan = plan_grasp(arm['pose'],base_object,profile,effective)
                blockers = workspace_blockers(plan,effective)
                if blockers:
                    raise ValueError('；'.join(blockers))
                grip = effective['gripper'];base = report['gripper_base']
                maximum = base.get('force_up')
                if not isinstance(maximum,list) or len(maximum)!=1 or type(maximum[0]) not in (int,float) or not np.isfinite(maximum[0]):
                    raise ValueError('夹爪未返回有效的力度上限')
                if not base.get('force') or not 0 < grip['force_target_raw'] <= grip['force_limit_raw'] <= maximum[0]:
                    raise ValueError('夹爪力度能力或参数不匹配')
                self.check_gripper_start(report['initial_gripper_state'],grip)
                plan['flange_T_grip_tcp'] = tool['flange_T_grip_tcp']
                report['active_stage'] = '检查候选夹法：'+entry['name']
                detail = self.verify_motion_targets(plan,effective,report,arm,plan['waypoints'],timeout=min(3.,remaining)) or {}
                travel = detail.get('joint_travel_deg',float(np.rad2deg(rotation_error(controller_pose(arm['pose'])[:3,:3],plan['target_rotation']))))
                row.update(passed=True,joint_travel_deg=travel)
                passed.append(((-entry['priority'],travel,entry['id']),plan,profile,effective,entry))
            except (ValueError, RuntimeError, TimeoutError, OSError, subprocess.TimeoutExpired) as error:
                self.check_cancel();row['reason'] = str(error)
            if time.monotonic() >= deadline:
                report['grasp_search_truncated'] = len(results)<len(entries);break
        if not passed:
            raise ValueError('没有可执行的候选夹法：\n'+'\n'.join(f"{r['name']}：{r.get('reason','未通过')}" for r in results))
        self.check_cancel()
        _,plan,profile,effective,entry = min(passed,key=lambda item:item[0])
        plan.update(grasp_id=entry['id'],grasp_name=entry['name'],grasp_selection_mode=value['selection_mode'],
                    grasp_candidates=results,grasp_library_revision=revision(value))
        return plan,profile,effective
