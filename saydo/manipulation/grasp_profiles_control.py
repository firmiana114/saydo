"""Visual grasp authoring and selection; editor mutations never connect to hardware."""

import copy
import json
import subprocess
import time
import uuid

import numpy as np
import yaml

from saydo.geometry import grasp_geometry
from saydo.objects import grasp_library as objects_grasp_library
from saydo.manipulation import reorientation_geometry
from saydo.manipulation import reorientation_visual
from saydo.geometry import pose_symmetry
from saydo.robot import arms


class GraspProfilesMixin:
    """夹法的查看、编辑和选用；这里的修改只写夹法库文件，不连接硬件。
    """
    def grasp_library(self):
        return objects_grasp_library.GraspLibrary(
            self.root, getattr(self.tracker, 'models_root', None)
        )

    def grasp_configuration(self, config):
        object_id = getattr(self.tracker, 'state', {}).get('object_id')
        if not object_id:
            return {'object_id': None, 'grasps': [], 'teaching_defaults': None}
        try:
            value = self.grasp_library().object(object_id)
            mode = getattr(self.tracker, 'state', {}).get('pose_mode', '6d')
            entries = []
            for entry in value['grasps']:
                row = {
                    key: entry.get(key)
                    for key in ('id', 'name', 'enabled', 'priority')
                }
                try:
                    row['profile'] = pose_symmetry.effective_profile(
                        entry, mode
                    )
                    row['effective_parameters'] = (
                        objects_grasp_library.effective_parameters(
                            config, entry
                        )
                    )
                    row['parameter_error'] = None
                except ValueError as error:
                    row.update(
                        effective_parameters=None, parameter_error=str(error)
                    )
                entries.append(row)
            return {
                'object_id': object_id,
                'selection_mode': value['selection_mode'],
                'pose_mode': mode,
                'selected_id': value['selected_id'],
                'revision': objects_grasp_library.revision(value),
                'grasps': entries,
                'teaching_defaults': (
                    objects_grasp_library.object_gripper_defaults(object_id)
                ),
                'error': None,
            }
        except (ValueError, OSError, KeyError, TypeError) as error:
            return {
                'object_id': object_id,
                'grasps': [],
                'teaching_defaults': (
                    objects_grasp_library.object_gripper_defaults(object_id)
                ),
                'error': str(error),
            }

    def grasp_editor_model(self, object_id):
        geometry = reorientation_geometry.model_geometry(
            self.tracker.models_root, object_id
        )
        low, high = objects_grasp_library.point_limits(geometry['vertices'])
        geometry.update(
            initial_rotation=np.eye(3).tolist(),
            holding=False,
            pivot_object_m=np.asarray(
                geometry['vertices']
            ).mean(axis=0).tolist(),
            point_min_m=low.tolist(),
            point_max_m=high.tolist(),
        )
        try:
            geometry['appearance'] = reorientation_visual.model_appearance(
                self.tracker.models_root, object_id
            )
        except (ValueError, OSError, TypeError) as error:
            geometry['appearance'] = {
                'status': 'unavailable', 'error': str(error)
            }
        return geometry

    def grasp_profiles(self, object_id):
        geometry = reorientation_geometry.model_geometry(
            self.tracker.models_root, object_id
        )
        config = json.loads(self.config_path.read_text())
        tool = yaml.safe_load(
            (self.root / config['tool_file']).read_text()
        )['controller_readback']['tool_response_raw']
        handeye = yaml.safe_load(
            (self.root / config['handeye_file']).read_text()
        )['flange_T_color_optical_camera']
        value = self.grasp_library().object(object_id)
        entries = []
        for entry in value['grasps']:
            row = copy.deepcopy(entry)
            row['profile'] = pose_symmetry.effective_profile(
                entry, geometry.get('pose_mode', '6d')
            )
            if geometry.get('pose_mode') == '5d':
                row['tested_successfully'] = entry.get(
                    'tested_successfully_5d', False
                )
            row['binding_error'] = objects_grasp_library.binding_error(
                entry, geometry['model_sha256'], tool, handeye
            )
            try:
                row['effective_parameters'] = (
                    objects_grasp_library.effective_parameters(config, entry)
                )
                row['parameter_error'] = None
            except ValueError as error:
                row.update(
                    effective_parameters=None, parameter_error=str(error)
                )
            entries.append(row)
        return {
            **value,
            'grasps': entries,
            'revision': objects_grasp_library.revision(value),
            'object_id': object_id,
            'pose_mode': geometry.get('pose_mode', '6d'),
            'pose_mode_revision': geometry.get('pose_mode_revision'),
            'object_name': geometry['object_name'],
            'model_sha256': geometry['model_sha256'],
            'maximum': objects_grasp_library.MAX_GRASPS,
            'changed_at': self.status.get('grasp_library_changed_at'),
            'parameter_scope': 'per_grasp',
            'defaults': {
                'pregrasp_distance_m': config['pregrasp_distance_m'],
                'tcp_offset_m': [0., 0., 0.],
                'min_table_clearance_m': grasp_geometry.minimum_table_clearance(
                    config
                ),
                **objects_grasp_library.object_gripper_defaults(object_id),
            },
        }

    def edit_grasp_profiles(self, request: dict) -> dict:
        """保存、复制、删除或选用一套夹法；只写夹法库文件，不连接硬件。

        夹法库是各臂共用的，所以修改期间要求所有臂都没有任务在执行。

        Args:
            request: 编辑请求，含物品编号 `object_id`、操作 `action`
                （`save`、`duplicate`、`delete`、`select`）以及该操作需要的
                夹法内容和版本号。

        Returns:
            更新后的夹法列表，以及本次保存或选用的夹法编号 `saved_id`。

        Raises:
            ValueError: 请求无效、这台臂有任务在进行，或夹法、模型设置
                在页面载入之后已被改动。
            BlockingIOError: 有一台臂正在执行任务。
        """
        if not isinstance(request, dict):
            raise ValueError('夹法编辑请求无效')
        object_id = request.get('object_id')
        action = request.get('action')
        if action not in ('save', 'duplicate', 'delete', 'select'):
            raise ValueError('未知的夹法操作')
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('机械臂任务进行中，不能修改夹法库')
            # 夹法库各臂共用：改它时任何一台臂都不能正在执行任务。
            with arms.all_locked(self.root):
                geometry = reorientation_geometry.model_geometry(
                    self.tracker.models_root, object_id
                )
                mode = geometry.get('pose_mode', '6d')
                if (
                    request.get('pose_mode', '6d') != mode
                    or (
                        mode == '5d'
                        and (
                            request.get('pose_mode_revision')
                            != geometry.get('pose_mode_revision')
                        )
                    )
                ):
                    raise ValueError('物品位姿模式已变化，请重新载入夹法编辑器')
                library = self.grasp_library()
                value = library.object(object_id)
                before = objects_grasp_library.revision(value)
                if request.get('revision') != before:
                    raise ValueError('夹法已变化，请重新载入后操作')
                selected = next(
                    (
                        g
                        for g in value['grasps']
                        if g['id'] == request.get('id')
                    ),
                    None,
                )
                if action in ('duplicate', 'delete') and selected is None:
                    raise ValueError('夹法不存在')
                if action == 'select':
                    mode = request.get('selection_mode')
                    if (
                        mode not in ('auto', 'manual')
                        or (
                            mode == 'manual'
                            and (selected is None or not selected['enabled'])
                        )
                    ):
                        raise ValueError('手动模式必须选择一套已启用的夹法')
                    value.update(
                        selection_mode=mode,
                        selected_id=(
                            selected['id'] if mode == 'manual' else None
                        ),
                    )
                elif action == 'delete':
                    value['grasps'].remove(selected)
                    if value['selected_id'] == selected['id']:
                        value.update(selection_mode='auto', selected_id=None)
                elif action == 'duplicate':
                    if len(value['grasps']) >= objects_grasp_library.MAX_GRASPS:
                        raise ValueError('夹法数量已达上限')
                    selected = copy.deepcopy(selected)
                    selected.update(
                        id=uuid.uuid4().hex,
                        name=(selected['name'] + ' 副本')[:60],
                        tested_successfully=False,
                    )
                    value['grasps'].append(selected)
                else:
                    data = request.get('grasp')
                    if not isinstance(data, dict):
                        raise ValueError('缺少夹法内容')
                    name = data.get('name')
                    if (
                        not isinstance(name, str)
                        or not name.strip()
                        or len(name.strip()) > 60
                    ):
                        raise ValueError('夹法名称须为 1–60 个字符')
                    if (
                        type(data.get('enabled')) is not bool
                        or type(data.get('priority')) is not int
                        or not 0 <= data['priority'] <= 100
                    ):
                        raise ValueError('夹法启用状态或优先级无效（0–100）')
                    if request.get('id') is not None and selected is None:
                        raise ValueError('夹法不存在，请重新载入')
                    profile = (
                        pose_symmetry.checked_axis_profile(data.get('profile'))
                        if mode == '5d'
                        else objects_grasp_library.checked_profile(
                            data.get('profile')
                        )
                    )
                    overrides = objects_grasp_library.checked_bound_overrides(
                        {
                            **objects_grasp_library.object_gripper_defaults(
                                object_id
                            ),
                            **objects_grasp_library.checked_overrides(
                                data.get('overrides', {})
                            ),
                        }
                    )
                    low, high = objects_grasp_library.point_limits(
                        geometry['vertices']
                    )
                    point = np.asarray(profile['grasp_point_object_m'])
                    if np.any(point < low) or np.any(point > high):
                        raise ValueError('抓取中心超出模型附近允许的编辑范围')
                    config = json.loads(self.config_path.read_text())
                    tool = yaml.safe_load(
                        (self.root / config['tool_file']).read_text()
                    )['controller_readback']['tool_response_raw']
                    if request.get('model_sha256') != geometry['model_sha256']:
                        raise ValueError('编辑器模型已变化，请重新载入')
                    if (
                        selected is not None
                        and objects_grasp_library.binding_error(
                            selected, geometry['model_sha256'], tool
                        )
                    ):
                        raise ValueError(
                            '旧夹法的模型或工具绑定已变化，请新建夹法'
                        )
                    if (
                        selected is None
                        and (
                            len(value['grasps'])
                            >= objects_grasp_library.MAX_GRASPS
                        )
                    ):
                        raise ValueError('夹法数量已达上限')
                    entry = {
                        'id': selected['id'] if selected else uuid.uuid4().hex,
                        'name': name.strip(),
                        'enabled': data['enabled'],
                        'priority': data['priority'],
                        'profile': profile,
                        'overrides': overrides,
                        'model_sha256': geometry['model_sha256'],
                        'tool_response': tool,
                        'flange_T_camera': None,
                        'source': 'visual',
                        'recorded_at': time.time(),
                        'tested_successfully': False,
                    }
                    if mode == '5d':
                        entry['profile_5d'] = profile
                        entry['profile'] = (
                            copy.deepcopy(selected['profile'])
                            if selected
                            else objects_grasp_library.checked_profile(profile)
                        )
                        entry['tested_successfully_5d'] = False
                        if selected:
                            entry['flange_T_camera'] = selected.get(
                                'flange_T_camera'
                            )
                            if overrides == selected.get('overrides', {}):
                                entry['tested_successfully'] = selected.get(
                                    'tested_successfully', False
                                )
                    elif selected and selected.get('profile_5d') is not None:
                        entry['profile_5d'] = copy.deepcopy(
                            selected['profile_5d']
                        )
                        entry['tested_successfully_5d'] = (
                            selected.get('tested_successfully_5d', False)
                            if overrides == selected.get('overrides', {})
                            else False
                        )
                    if selected is not None:
                        # Metadata edits must not silently rebind a taught pose
                        # to new calibration.
                        if (
                            profile
                            == (
                                pose_symmetry.effective_profile(selected, mode)
                                if mode == '5d'
                                else objects_grasp_library.checked_profile(
                                    selected['profile']
                                )
                            )
                        ):
                            entry.update(
                                source=selected.get('source', 'taught'),
                                flange_T_camera=selected.get('flange_T_camera'),
                            )
                            if overrides == selected.get('overrides', {}):
                                success_key = (
                                    'tested_successfully_5d'
                                    if mode == '5d'
                                    else 'tested_successfully'
                                )
                                entry[success_key] = selected.get(
                                    success_key, False
                                )
                                if 'last_success_at' in selected:
                                    entry['last_success_at'] = selected[
                                        'last_success_at'
                                    ]
                        value['grasps'][value['grasps'].index(selected)] = entry
                    else:
                        value['grasps'].append(entry)
                    selected = entry
                    if (
                        not entry['enabled']
                        and value['selected_id'] == entry['id']
                    ):
                        value.update(selection_mode='auto', selected_id=None)
                library.write_object(object_id, value, before)
                records = (
                    library.object_directory(object_id)
                    / 'grasp_profile_commands'
                )
                records.mkdir(parents=True, exist_ok=True)
                (records / f'{time.time_ns()}.json').write_text(
                    json.dumps(
                        {
                            'action': action,
                            'object_id': object_id,
                            'id': selected['id'] if selected else None,
                            'before_revision': before,
                            'after_revision': objects_grasp_library.revision(
                                value
                            ),
                            'motion_sent': False,
                        },
                        ensure_ascii=False,
                        indent=2,
                    )
                )
                self.status.update(
                    plan=None, grasp_library_changed_at=time.time()
                )
            return {
                **self.grasp_profiles(object_id),
                'saved_id': selected['id'] if selected else None,
                'motion_sent': False,
            }

    def select_grasp_candidate(
        self,
        config: dict,
        observation: dict,
        tool: dict,
        handeye: dict,
        base_object: np.ndarray,
        arm: dict,
        report: dict,
    ) -> tuple:
        """在物品的各套夹法里选出第一套通过全部检查的；只计算，不动作。

        依次检查每套已启用的夹法：绑定是否仍然有效、几何是否可达、
        是否越出工作范围或低于桌面、路径逆解是否连续。

        Args:
            config: 这台臂的抓取配置。
            observation: 视觉观测。
            tool: 这台臂的工具记录。
            handeye: `calibration` 返回的手眼记录。
            base_object: 物品在这台臂基座坐标里的位姿，4×4。
            arm: 控制器报告的机械臂当前状态。
            report: 本次任务的诊断记录，各候选的检查结果写入其中。

        Returns:
            `(方案, 夹法, 生效的配置)`。

        Raises:
            ValueError: 没有任何一套夹法通过检查，原因写在报错里。
        """
        mode = observation.get('pose_mode', '6d')
        library = self.grasp_library()
        value = library.object(observation['object_id'])
        entries = value['grasps']
        # Config-only installations keep their existing single-profile behavior.
        if not entries and not library.has_object(observation['object_id']):
            entries = [
                {
                    'id': 'config',
                    'name': '配置夹法',
                    'enabled': True,
                    'priority': 0,
                    'profile': copy.deepcopy(config['grasp_profile']),
                    'overrides': objects_grasp_library.object_gripper_defaults(
                        observation['object_id']
                    ),
                    'model_sha256': observation.get('model_sha256'),
                    'tool_response': tool[
                        'controller_readback'
                    ]['tool_response_raw'],
                }
            ]
        if value['selection_mode'] == 'manual':
            entries = [g for g in entries if g['id'] == value['selected_id']]
        entries = sorted(
            (g for g in entries if g['enabled']),
            key=lambda g: (-g['priority'], g['id']),
        )
        if not entries:
            raise ValueError('没有已启用的夹法，请在夹法编辑器中新增或启用')
        results = report['grasp_candidates'] = []
        passed = []
        deadline = time.monotonic() + 12
        for entry in entries:
            self.check_cancel()
            row = {
                'id': entry['id'],
                'name': entry['name'],
                'priority': entry['priority'],
                'passed': False,
            }
            results.append(row)
            try:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('候选检查时间预算已用完')
                error = objects_grasp_library.binding_error(
                    entry,
                    observation.get('model_sha256'),
                    tool['controller_readback']['tool_response_raw'],
                    objects_grasp_library.binding_handeye(handeye),
                )
                if error:
                    raise ValueError(error)
                effective = objects_grasp_library.effective_config(
                    config, entry
                )
                row['effective_parameters'] = (
                    objects_grasp_library.effective_parameters(config, entry)
                )
                profile = {
                    **copy.deepcopy(config['grasp_profile']),
                    **pose_symmetry.effective_profile(entry, mode),
                }
                grip = effective['gripper']
                base = report['gripper_base']
                maximum = base.get('force_up')
                if (
                    not isinstance(maximum, list)
                    or len(maximum) != 1
                    or type(maximum[0]) not in (int, float)
                    or not np.isfinite(maximum[0])
                ):
                    raise ValueError('夹爪未返回有效的力度上限')
                if (
                    not base.get('force')
                    or (
                        not 0
                        < grip['force_target_raw']
                        <= grip['force_limit_raw']
                        <= maximum[0]
                    )
                ):
                    raise ValueError('夹爪力度能力或参数不匹配')
                self.check_gripper_start(report['initial_gripper_state'], grip)
                variants = (
                    pose_symmetry.grasp_variants(
                        base_object,
                        profile,
                        grasp_geometry.controller_pose(arm['pose'])[:3, :3],
                    )
                    if mode == '5d'
                    else [(profile, {})]
                )
                attempts = row['azimuth_attempts'] = []
                plan = None
                for resolved, azimuth in variants:
                    self.check_cancel()
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('候选检查时间预算已用完')
                    trial = dict(azimuth)
                    attempts.append(trial)
                    try:
                        candidate = grasp_geometry.plan_grasp(
                            arm['pose'],
                            base_object,
                            resolved,
                            effective,
                            grasp_reference_offset_tcp_m=effective[
                                'tcp_offset_m'
                            ],
                        )
                        blockers = grasp_geometry.workspace_blockers(
                            candidate, effective
                        )
                        if blockers:
                            raise ValueError('；'.join(blockers))
                        candidate['flange_T_grip_tcp'] = tool[
                            'flange_T_grip_tcp'
                        ]
                        report['active_stage'] = (
                            '检查候选夹法：' + entry['name']
                        )
                        detail = (
                            self.verify_motion_targets(
                                candidate,
                                effective,
                                report,
                                arm,
                                candidate['waypoints'],
                                timeout=min(3., remaining),
                            )
                            or {}
                        )
                        trial['passed'] = True
                        plan = candidate
                        if mode == '5d':
                            vertical = (
                                azimuth.get('approach_policy')
                                == 'horizontal_5d_vertical_down'
                            )
                            plan.update(
                                orientation_policy='axisymmetric_z',
                                approach_angle_deg=resolved[
                                    'approach_angle_deg'
                                ],
                                approach_policy=azimuth.get(
                                    'approach_policy', 'free_azimuth'
                                ),
                                azimuth=azimuth,
                                azimuth_step_deg=None if vertical else 15,
                                azimuth_search_max_candidates=len(variants),
                            )
                        profile = resolved
                        break
                    except (
                        ValueError,
                        RuntimeError,
                        TimeoutError,
                        OSError,
                        subprocess.TimeoutExpired,
                    ) as error:
                        self.check_cancel()
                        trial.update(passed=False, reason=str(error))
                if plan is None:
                    label = (
                        '横躺5D物品仅允许竖直向下抓取，两种夹爪朝向均未通过'
                        if (
                            attempts[-1].get('approach_policy')
                            == 'horizontal_5d_vertical_down'
                        )
                        else '自由方位候选均未通过'
                    )
                    raise ValueError(
                        (
                            (
                                f'{label}（已检查{len(attempts)}个，'
                                f'以下为最后一个候选的原因）：'
                            )
                            + attempts[-1]['reason']
                        )
                        if mode == '5d'
                        else attempts[-1]['reason']
                    )
                plan['pose_mode'] = mode
                travel = detail.get(
                    'joint_travel_deg',
                    float(
                        np.rad2deg(
                            grasp_geometry.rotation_error(
                                grasp_geometry.controller_pose(
                                    arm['pose']
                                )[:3, :3],
                                plan['target_rotation'],
                            )
                        )
                    ),
                )
                row.update(passed=True, joint_travel_deg=travel)
                passed.append(
                    (
                        (-entry['priority'], travel, entry['id']),
                        plan,
                        profile,
                        effective,
                        entry,
                    )
                )
            except (
                ValueError,
                RuntimeError,
                TimeoutError,
                OSError,
                subprocess.TimeoutExpired,
            ) as error:
                self.check_cancel()
                row['reason'] = str(error)
            if time.monotonic() >= deadline:
                report['grasp_search_truncated'] = len(results) < len(entries)
                break
        if not passed:
            raise ValueError(
                '没有可执行的候选夹法：\n'
                + '\n'.join(
                    f"{r['name']}：{r.get('reason','未通过')}" for r in results
                )
            )
        self.check_cancel()
        _, plan, profile, effective, entry = min(
            passed, key=lambda item: item[0]
        )
        plan.update(
            grasp_id=entry['id'],
            grasp_name=entry['name'],
            grasp_selection_mode=value['selection_mode'],
            grasp_candidates=results,
            grasp_library_revision=objects_grasp_library.revision(value),
            effective_parameters=objects_grasp_library.effective_parameters(
                config, entry
            ),
        )
        return plan, profile, effective
