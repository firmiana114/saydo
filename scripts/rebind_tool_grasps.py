"""Explicit offline rebinding after a user-changed tool; never writes hardware.
The caller must hold the robot lock and stop the runtime before applying a change.
"""

import copy
import numpy as np
from saydo.geometry import grasp_geometry
from saydo.objects import grasp_library


def updated_tool_config(
    previous, tool_response, source_path, source_sha256, recorded_at
):
    old = previous['controller_readback']['tool_response_raw']
    if (
        tool_response.get('tool_name') != old.get('tool_name')
        or tool_response.get('state') != 'current_tool_frame'
    ):
        raise ValueError('工具名称或读回类型不匹配')
    pose = tool_response.get('pose')
    if (
        not isinstance(pose, list)
        or len(pose) != 6
        or any(type(v) not in (int, float) or not np.isfinite(v) for v in pose)
        or pose[:2] + pose[3:] != old['pose'][:2] + old['pose'][3:]
    ):
        raise ValueError('本次同步只允许工具Z平移及负载/质心变化')
    mass, center = tool_response.get('payload'), tool_response.get('position')
    if type(mass) not in (int, float) or not np.isfinite(mass) or mass < 0:
        raise ValueError('工具负载读回无效')
    if (
        not isinstance(center, list)
        or len(center) != 3
        or any(
            type(v) not in (int, float) or not np.isfinite(v) for v in center
        )
    ):
        raise ValueError('工具质心读回无效')
    result = copy.deepcopy(previous)
    result.update(
        recorded_at=recorded_at,
        translation_m=(np.asarray(pose[:3]) / 1e6).tolist(),
        rpy_rad=(np.asarray(pose[3:]) / 1000).tolist(),
        flange_T_grip_tcp=grasp_geometry.controller_pose(pose).tolist(),
    )
    result['load'].update(
        mass_kg=mass / 1000,
        center_of_mass_m=(np.asarray(center) / 1e6).tolist(),
        method='controller_readback_after_user_tool_edit',
    )
    result['controller_readback'].update(
        path=source_path,
        sha256=source_sha256,
        read_at=recorded_at,
        tool_response_raw=copy.deepcopy(tool_response),
        tool_was_active=True,
    )
    result['geometry'].update(
        status='configured_not_geometrically_validated',
        method='user_tool_origin_update_with_per_grasp_offsets',
        position_reference=(
            'User-set controller tool origin; per-grasp local offsets define '
            'the grasp reference.'
        ),
    )
    result['geometry']['latest_tool_update'] = {
        'previous_tool_response': copy.deepcopy(old),
        'new_translation_m': result['translation_m'],
        'source': 'user_edit_and_readonly_controller_snapshot',
        'recorded_at': recorded_at,
        'existing_grasp_offset_policy': (
            'reset_to_zero_by_explicit_user_request'
        ),
    }
    result['controller_written_by_this_change'] = False
    return result


def rebind_grasps(library, old_tool, new_tool, apply=False):
    changes = []
    count = 0
    for path in sorted(library.models_root.glob('*/grasp_profiles.json')):
        oid = path.parent.name
        before = library.object(oid)
        after = copy.deepcopy(before)
        for entry in after['grasps']:
            if entry.get('tool_response') == new_tool:
                grasp_library.checked_bound_overrides(
                    entry.get('overrides', {})
                )
                continue  # Already rebound: do not erase subsequent user edits.
            if entry.get('tool_response') != old_tool:
                raise ValueError(
                    f'{oid}/{entry["id"]}工具绑定与本次旧工具不一致，未迁移'
                )
            entry['overrides'] = grasp_library.checked_bound_overrides(
                {**entry.get('overrides', {}), 'tcp_offset_m': [0., 0., 0.]}
            )
            entry.update(
                tool_response=copy.deepcopy(new_tool),
                tested_successfully=False,
                tested_successfully_5d=False,
            )
            count += 1
        if before != after:
            changes.append((oid, grasp_library.revision(before), after))
    # All objects are prechecked before writing the first one.
    if apply:
        for oid, expected, value in changes:
            library.write_object(oid, value, expected)
    return {
        'files_to_write': len(changes),
        'grasps_to_rebind': count,
        'objects': [row[0] for row in changes],
        'offset_policy': 'zero',
        'applied': apply,
        'motion_sent': False,
    }
