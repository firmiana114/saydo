"""Operator-triggered read-only reconciliation; never resumes or sends motion."""
import argparse
import fcntl
import json
from .config import Config, ROOT
from .store import Store
from tools.robot_agent.client import RobotBackend, BackendError


def reconcile(store, backend, run_id, confirmed_not_submitted=False):
    run = store.get(run_id)
    if run['state'] == 'running':
        raise ValueError('不能核对仍在运行的任务')
    status = backend.request('/api/agent/status')
    if status.get('active_task_id') or status.get('task_recording_error') or status['grasp'].get('busy') or status['grasp'].get('stopping'):
        raise ValueError('后端忙碌、停止未确认或记录故障，不能解除阻断')
    checks = []
    for call in run['calls']:
        if call['name'] not in ('grasp', 'place', 'reset_arm', 'reorient'):
            continue
        try:
            task = backend.request('/api/agent/tasks/'+call['request_id'])
        except BackendError as error:
            if error.payload.get('code') != 'TASK_NOT_FOUND' or not confirmed_not_submitted:
                raise ValueError('原动作请求未确认；不能解除阻断') from error
            checks.append({'task_id': call['request_id'], 'operator_confirmed_not_submitted': True})
            continue
        if not task.get('terminal'):
            raise ValueError('原动作仍未结束')
        checks.append({'task_id': call['request_id'], 'state': task['state'], 'outcome_unknown': task.get('outcome_unknown', False)})
    run.pop('calls')
    run.update(outcome_unknown=False, reconciliation={'operator_confirmed_scene': True, 'tasks': checks},
        phase='已人工核对，可提交新的任务；原任务未恢复')
    store.save(run)
    return {'run_id': run_id, 'motion_sent': False, 'resumed': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env', default=ROOT/'.env')
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--confirm-scene-inspected', action='store_true', required=True)
    parser.add_argument('--confirm-request-not-submitted', action='store_true')
    args = parser.parse_args()
    config = Config.load(args.env)
    with (config.state_dir/'runner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        store = Store(config.state_dir, secrets=(config.api_key,))
        backend = RobotBackend(config.robot_url, config.token_file)
        print(json.dumps(reconcile(store, backend, args.run_id, args.confirm_request_not_submitted)))


if __name__ == '__main__':
    main()
