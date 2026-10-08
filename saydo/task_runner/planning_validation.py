"""Correctable argument errors are restricted to planners that cannot move hardware."""

import math
from saydo.contracts.motion_contract import checked_goal, identity, vector

READONLY_PLANNING = frozenset(
    (
        'preview_grasp',
        'preview_place',
        'preview_reorient',
        'plan_held_object_motion',
        'plan_release',
    )
)


class PlanningArgumentError(ValueError):
    def __init__(self, name, arguments, detail):
        self.name, self.arguments, self.detail = name, arguments, detail
        super().__init__(detail['message'])


def goal_error_detail(error):
    result = {'code': 'INVALID_ARGUMENTS', 'message': str(error)[:1200]}
    if hasattr(error, 'field'):
        result.update(field=error.field, expected=error.expected)
    return result


def validate_planning_arguments(name, args):
    if name in ('plan_held_object_motion', 'plan_release'):
        identity(args)
        checked_goal(args['goal'], release=name == 'plan_release')
        if name == 'plan_release':
            retreat = vector(
                args['retreat_vector_base_m'], 'retreat_vector_base_m', .3
            )
            if math.sqrt(sum(v * v for v in retreat)) > .3:
                raise ValueError('retreat_vector_base_m长度不得超过0.3米')
    elif name == 'preview_reorient':
        identity(args)
        checked_goal(
            {
                'target_rpy_deg': args['target_rpy_deg'],
                'position_mode': args['position_mode'],
                'max_rotation_step_deg': args.get('max_rotation_step_deg', 15),
            }
        )
