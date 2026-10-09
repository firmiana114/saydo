"""No-LLM tube/rack workflow: capture -> complete plan -> optional fixed execution."""

import argparse
import json
import pathlib
import secrets
import time
import urllib.parse

from saydo.agent import client


def box(value):
    try:
        values = [int(x) for x in value.split(',')]
    except ValueError as error:
        raise argparse.ArgumentTypeError('框格式为x0,y0,x1,y1') from error
    if (
        len(values) != 4
        or not 0 <= values[0] < values[2] <= 640
        or not 0 <= values[1] < values[3] <= 360
    ):
        raise argparse.ArgumentTypeError('框须位于眼在手640×360画面内')
    return values


def wait_task(backend, operation, args, ident, timeout=240):
    try:
        task = backend.action(operation, ident, args)
    except client.BackendError as error:
        if error.payload.get('code') != 'REQUEST_UNCERTAIN':
            raise
        # Never submit the action again. Query the original durable request ID.
        task = backend.request(
            '/api/agent/tasks/' + urllib.parse.quote(ident, safe='')
        )
    deadline = (
        None
        if operation == 'plan_fixed_pick_place'
        else time.monotonic() + timeout
    )
    while not task.get('terminal'):
        if deadline is not None and time.monotonic() > deadline:
            raise TimeoutError('任务等待超时，原任务编号：' + ident)
        time.sleep(.3)
        task = backend.request(
            '/api/agent/tasks/' + urllib.parse.quote(ident, safe='')
        )
    expected = {
        'capture_scene_snapshot': 'scene_ready',
        'plan_fixed_pick_place': 'fixed_pick_place_planned',
        'execute_fixed_pick_place': 'completed',
    }[operation]
    if task.get('state') != expected:
        raise RuntimeError(
            json.dumps(
                {
                    'task_id': ident,
                    'state': task.get('state'),
                    'error': (
                        task.get('error') or task.get('result', {}).get('error')
                    ),
                    'diagnostics_ref': task.get('result', {}).get(
                        'diagnostics_ref'
                    ),
                },
                ensure_ascii=False,
            )
        )
    return task['result']


def workflow(backend, args):
    ident = 'fixed-' + secrets.token_hex(10)
    directory = pathlib.Path(args.output) / ident
    directory.mkdir(parents=True)

    def save(name, value):
        (directory / name).write_text(
            json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
        )

    status = backend.request('/api/agent/status')
    g = status.get('grasp', {})
    if (
        g.get('busy')
        or g.get('stopping')
        or g.get('held_object')
        or status.get('active_task_id')
    ):
        raise ValueError('须先结束活动任务，并从无夹持记录开始')
    if args.execute and g.get('mode') != 'execute':
        raise ValueError('后端未启用执行')
    if args.empty_rack:
        geometry = backend.request(
            '/api/agent/object-geometry?object_id='
            + urllib.parse.quote(args.rack, safe='')
        )
        slots = sorted(
            f['id'][:-8]
            for f in geometry.get('features', [])
            if f['id'].endswith('.support')
        )
    else:
        slots = args.free_slots.split(',')
    if not slots or any(not s for s in slots):
        raise ValueError('没有已确认空闲的孔位')
    targets = [
        {
            'instance_id': 'tube',
            'object_id': args.tube,
            'role': 'source',
            'box': args.tube_box,
        },
        {
            'instance_id': 'rack',
            'object_id': args.rack,
            'role': 'reference',
            'box': args.rack_box,
        },
    ]
    save(
        'request.json',
        {
            'request_prefix': ident,
            'targets': targets,
            'free_slots': slots,
            'empty_rack_declared': args.empty_rack,
            'release_gap_m': args.release_gap_mm / 1000,
            'execute_requested': args.execute,
        },
    )
    print('采集同一画面的试管与试管架位姿……', flush=True)
    scene = wait_task(
        backend,
        'capture_scene_snapshot',
        {'targets': targets, 'environment_static': True},
        ident + '-scene',
    )
    save('scene.json', scene)
    print('规划完整抓取、调姿、入架及退出路线；尚未运动……', flush=True)
    request = {
        'scene_id': scene['scene_id'],
        'source_instance_id': 'tube',
        'reference_instance_id': 'rack',
        'free_slots': slots,
        'release_gap_m': args.release_gap_mm / 1000,
    }
    try:
        result = wait_task(
            backend, 'plan_fixed_pick_place', request, ident + '-plan'
        )
    except KeyboardInterrupt:
        save(
            'cancel_planning.json',
            backend.request(
                '/api/agent/cancel-planning', {'task_id': ident + '-plan'}
            ),
        )
        raise
    save('plan.json', result)
    plan = result['fixed_pick_place_plan']
    print(
        json.dumps(
            {
                'selected_slot': plan['selected_slot'],
                'selection': '距试管最近且全程通过的孔位',
                'distance_mm': round(
                    plan['selected_slot_distance_m'] * 1000, 1
                ),
                'nearer_slots_rejected': plan['nearer_slots_rejected'],
                'plan_id': plan['plan_id'],
                'complete_path_prechecked': plan['complete_path_prechecked'],
                'files': str(directory),
                'validation_scope': plan['validation_scope'],
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    if not args.execute:
        print('只读规划完成。未发送机械臂或夹爪动作。', flush=True)
        return result
    print('执行已保存的固定方案；Ctrl-C停止，不自动重试。', flush=True)
    try:
        # Sampled joint moves make the whole pick/place take minutes; a client
        # timeout here would stop the arm.
        result = wait_task(
            backend,
            'execute_fixed_pick_place',
            {'plan_id': plan['plan_id']},
            ident + '-execute',
            timeout=900,
        )
    except BaseException:
        try:
            save('stop.json', backend.request('/api/agent/stop', {}))
        except Exception as error:
            print('软件停止未确认：' + str(error), flush=True)
        raise
    save('execution.json', result)
    print(
        '控制器确认到位、松爪和退出；实物是否稳定入架仍需现场核验。', flush=True
    )
    return result


def main() -> None:
    """解析命令行并运行一次固定抓放；默认只规划，`--execute` 才会运动。"""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base-url', default='http://192.168.101.65:8089')
    p.add_argument(
        '--token-file',
        type=pathlib.Path,
        default=(
            pathlib.Path.home() / '.local/share/saydo-mcp/robot-backend-token'
        ),
    )
    p.add_argument(
        '--arm',
        help='用哪台机械臂，例如 arm2；不写时用 1 臂',
    )
    p.add_argument('--tube', default='transparent_test_tube_150mm')
    p.add_argument('--rack', default='test_tube_rack_40_272x112x85')
    p.add_argument(
        '--tube-box',
        type=box,
        help='可选试管框；省略时使用后端已配置的自动检测器，本地模式不调用LLM',
    )
    p.add_argument(
        '--rack-box',
        type=box,
        help=(
            '可选试管架框；省略时同样由后端自动检测，'
            '未得到唯一候选则采集失败并提示改为传框'
        ),
    )
    free = p.add_mutually_exclusive_group(required=True)
    free.add_argument(
        '--free-slots', help='明确确认空闲的模型孔位，例如A01,A02,B03'
    )
    free.add_argument(
        '--empty-rack',
        action='store_true',
        help='声明架子所有候选孔均为空；不自动检测占用',
    )
    p.add_argument(
        '--release-gap-mm',
        type=float,
        default=3.,
        help='底端距支撑面的间隙，0至5毫米；默认3毫米',
    )
    p.add_argument(
        '--execute',
        action='store_true',
        help='完整方案通过后实际执行；默认仅规划',
    )
    p.add_argument(
        '--output',
        type=pathlib.Path,
        default=pathlib.Path('work/fixed_pick_place'),
    )
    args = p.parse_args()
    if not 0 <= args.release_gap_mm <= 5:
        p.error('释放间隙须为0至5毫米')
    try:
        workflow(
            client.RobotBackend(
                args.base_url, args.token_file, arm=args.arm
            ),
            args,
        )
    except client.BackendError as error:
        if error.payload.get('code') != 'INVALID_ARGUMENTS':
            raise
        # The backend validates requests before starting anything, so nothing
        # was planned or moved.
        p.exit(
            2,
            (
                '后端不接受本脚本的请求格式（INVALID_ARGUMENTS：'
                + str(error)
                + (
                    '）。请求在开始前即被拒绝，未发送机械臂或夹爪动作。\n'
                    '最常见的原因是8089仍在运行更新之前的代码，'
                    '与本脚本版本不一致：先运行 scripts/restart_tr'
                    'acker.py 重启后再试。\n'
                )
            ),
        )


if __name__ == '__main__':
    main()
