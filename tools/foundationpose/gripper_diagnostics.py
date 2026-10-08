"""Bounded closure diagnostics; adds no hardware commands or acceptance rules."""
import copy
import time

from grasp_geometry import contact_result

MAX_SAMPLES = 128


def feedback_summary(state):
    values = {**state, 'sys_err':state.get('sys_err', state.get('sys_error'))}
    return '，'.join(f'{key}={values.get(key)}' for key in
                    ('pos', 'dof_state', 'force', 'current', 'speed', 'sys_state', 'sys_err', 'dof_err'))


def contact_failure_message(result, state, gripper):
    summary = feedback_summary(state)
    if result == 'empty_close':
        threshold = gripper['closed_position'] + gripper['contact_position_margin']
        return f'夹爪位置到位，但未确认接触（按现有规则判为空夹，位置阈值≤{threshold}，原始值非毫米）：{summary}'
    dof = state.get('dof_state')
    reason = ('夹爪电流保护停止' if dof == [5] else
              '夹爪报告故障停止' if dof == [6] else '夹爪系统/自由度反馈异常')
    return f'{reason}：{summary}'


class CloseDiagnostics:
    def __init__(self, report, gripper):
        self.started = time.monotonic()
        self.gripper = gripper
        self.data = {'schema_version': 1, 'started_at': time.time(),
                     'command_ack_is_not_contact': True,
                     'policy': {'closed_position': gripper['closed_position'],
                                'contact_position_margin': gripper['contact_position_margin'],
                                'force_limit_raw': gripper['force_limit_raw'],
                                'close_timeout_s': gripper['close_timeout_s'],
                                'stable_samples_required': 3, 'position_units': 'raw_not_mm'},
                     'commands': [], 'samples': [], 'sample_limit': MAX_SAMPLES,
                     'samples_total': 0, 'samples_dropped': 0, 'last_sample': None}
        report['gripper_close_diagnostics'] = self.data

    def stamp(self):
        return {'at': time.time(), 'elapsed_s': time.monotonic()-self.started}

    def command(self, name, parameters, call):
        record = {**self.stamp(), 'command': name, 'parameters': parameters, 'acknowledged': False}
        self.data['commands'].append(record)
        try:
            call()
            record['acknowledged'] = True
        except Exception as error:
            record.update(error=str(error), error_type=type(error).__name__)
            raise
        finally:
            record['finished'] = self.stamp()

    def sample(self, robot):
        sample = {**self.stamp(), 'sequence': self.data['samples_total']+1,
                  'state': None, 'classification': None}
        self.data['samples_total'] += 1
        if len(self.data['samples']) < MAX_SAMPLES:
            self.data['samples'].append(sample)
        else:
            self.data['samples_dropped'] += 1
        self.data['last_sample'] = sample
        try:
            sample['state'] = copy.deepcopy(robot.query('get_rm_plus_state_info')['state_info'])
            sample['received'] = self.stamp()
            sample['classification'] = contact_result(sample['state'], self.gripper)
        except Exception as error:
            sample.update(error=str(error), error_type=type(error).__name__)
            sample['failed'] = self.stamp()
            raise
        return sample

    def decision(self, outcome, sample=None):
        self.data['decision'] = {**self.stamp(), 'outcome': outcome, 'sample': copy.deepcopy(sample)}


def record_failure_before_stop(report, error):
    record = {'at': time.time(), 'stage': report.get('active_stage'),
              'error': str(error), 'error_type': type(error).__name__}
    diagnostics = report.get('gripper_close_diagnostics')
    if diagnostics is not None:
        record['last_closing_sample'] = copy.deepcopy(diagnostics['last_sample'])
        diagnostics['failure_before_stop'] = copy.deepcopy(record)
    report['failure_before_stop'] = record
