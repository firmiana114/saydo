#!/usr/bin/env python3
"""Stop and start the 8089 tracker on this machine; sends no robot or gripper command."""

import argparse
import fcntl
import json
import logging
import os
import pathlib
import re
import signal
import subprocess
import sys
import time
import urllib.request

# 本文件位于 <根目录>/scripts/，按路径直接运行，不依赖 saydo 包。
ROOT = pathlib.Path(__file__).resolve().parents[1]
HOST, PORT = '192.168.101.65', 8089
RUNNER_URL = 'http://127.0.0.1:8091/api/task-runs'
LIVE = ROOT / 'work/foundationpose_live'
# Same start-up arguments as the documented manual command; extra arguments are
# appended.
TRACKER_ARGS = [
    '--host',
    HOST,
    '--port',
    str(PORT),
    '--serial',
    '405622073572',
    '--agent-token-file',
    str(pathlib.Path.home() / '.local/share/saydo-mcp/robot-backend-token'),
    '--mesh',
    'work/object_models/transparent_test_tube_150mm/model/model.obj',
    '--auto-start',
]
FINISHED = {'succeeded', 'blocked', 'failed', 'cancelled'}
LOG = logging.getLogger('restart_tracker')


def get_json(url, timeout=5):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(url, timeout=timeout) as response:
        return json.load(response)


def listening_pid(text):
    """PID from one `ss -ltnpH` line, or None when nothing listens."""
    found = re.search(r'pid=(\d+)', text)
    return int(found.group(1)) if found else None


def current_pid():
    result = subprocess.run(
        ['ss', '-ltnpH', 'sport = :%d' % PORT],
        capture_output=True,
        text=True,
        check=True,
    )
    return listening_pid(result.stdout)


def blockers(status, runs):
    """Reasons a restart would lose state; `runs` is None when the task runner is unreachable."""
    grasp = status.get('grasp') or {}
    reasons = []
    if grasp.get('busy'):
        reasons.append('机械臂任务正在执行')
    if grasp.get('held_object') is not None:
        reasons.append('存在夹持记录，重启后无法继续调姿或放置')
    if runs is not None:
        if runs.get('stopping'):
            reasons.append('任务执行器正在停止任务')
        active = [
            r.get('run_id')
            for r in runs.get('runs', [])
            if r.get('state') not in FINISHED
        ]
        if active:
            reasons.append(
                '任务执行器有未结束的任务：' + ', '.join(str(i) for i in active)
            )
    return reasons


def check_idle():
    status = get_json('http://%s:%d/api/status' % (HOST, PORT))
    try:
        runs = get_json(RUNNER_URL, timeout=8)
    except (OSError, ValueError) as error:
        LOG.warning(
            '任务执行器不可达，跳过其任务检查：%s', type(error).__name__
        )
        runs = None
    return blockers(status, runs)


def children(pid):
    result = subprocess.run(
        ['pgrep', '-P', str(pid)], capture_output=True, text=True
    )
    return [int(p) for p in result.stdout.split()]


def stop(pid, timeout=30):
    command = pathlib.Path(
        '/proc/%d/cmdline' % pid
    ).read_bytes().replace(b'\0', b' ').decode(errors='replace')
    # 同时认得旧的启动方式（脚本路径）和新的启动方式（-m 模块名）。
    if 'live_tracker' not in command:
        raise RuntimeError(
            '占用 %d 端口的进程 %d 不是跟踪服务，未停止：%s'
            % (PORT, pid, command[:120])
        )
    # The resident detector child exits with its parent.
    waiting = [pid] + children(pid)
    LOG.info('停止跟踪服务 PID %d（含子进程 %s）', pid, waiting[1:])
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + timeout
    while any(pathlib.Path('/proc/%d' % p).exists() for p in waiting):
        if time.monotonic() > deadline:
            raise RuntimeError(
                '%d 秒内进程未退出，未强制结束，请人工检查：%s'
                % (timeout, waiting)
            )
        time.sleep(.2)
    LOG.info('旧进程已全部退出')


def start(extra):
    LIVE.mkdir(parents=True, exist_ok=True)
    log_path = LIVE / time.strftime('restart_script_%Y%m%d_%H%M%S.log')
    script = (
        'source scripts/runtime_env.sh && exec python '
        '-m saydo.workbench.live_tracker "$@"'
    )
    with log_path.open('wb') as log:
        # A new session with no inherited terminal, so the caller's ssh can
        # return.
        process = subprocess.Popen(
            ['bash', '-c', script, 'bash', *TRACKER_ARGS, *extra],
            cwd=ROOT,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    LOG.info(
        '已启动跟踪服务 PID %d，日志 %s',
        process.pid,
        log_path.relative_to(ROOT),
    )
    return process, log_path


def wait_ready(process, log_path, listen_timeout=120, model_timeout=180):
    deadline = time.monotonic() + listen_timeout
    while current_pid() != process.pid:
        if process.poll() is not None or time.monotonic() > deadline:
            tail = log_path.read_text(errors='replace').splitlines()[-15:]
            raise RuntimeError(
                '跟踪服务未能开始监听 %d 端口，日志结尾：\n%s'
                % (PORT, '\n'.join(tail))
            )
        time.sleep(.5)
    (LIVE / 'server.pid').write_text('%d\n' % process.pid)
    LOG.info('已监听 %s:%d，PID 文件已更新；等待模型就绪', HOST, PORT)
    deadline = time.monotonic() + model_timeout
    status = {}
    while time.monotonic() < deadline and process.poll() is None:
        try:
            status = get_json('http://%s:%d/api/status' % (HOST, PORT))
        except (OSError, ValueError):
            status = {}
        if status.get('model_ready') or status.get('error'):
            break
        time.sleep(1)
    return status


def acquire_lock():
    """Hold an exclusive lock for this run so two restarts cannot stop each other's new process."""
    LIVE.mkdir(parents=True, exist_ok=True)
    handle = (LIVE / 'restart.lock').open('a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        handle.close()
        raise RuntimeError(
            '另一次重启正在进行，本次未执行；等它结束后查看结果'
        ) from error
    return handle


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='关闭并启动 8089 跟踪服务；不发送机械臂或夹爪指令。'
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='跳过“无任务、无夹持”检查（会丢失夹持记录）',
    )
    parser.add_argument(
        'tracker_args',
        nargs='*',
        help='写在 -- 之后，追加给 live_tracker 的参数',
    )
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s',
        datefmt='%H:%M:%S',
    )
    try:
        lock = acquire_lock()  # Released when this process exits.
        pid = current_pid()
        if pid is None:
            LOG.info('%d 端口当前无服务，直接启动', PORT)
        else:
            if args.force:
                LOG.warning('已指定 --force，跳过任务与夹持检查')
            else:
                try:
                    reasons = check_idle()
                except (OSError, ValueError) as error:
                    raise RuntimeError(
                        (
                            '无法读取服务状态以确认空闲（%s）；确认可重启后加 '
                            '--force'
                        )
                        % type(error).__name__
                    ) from error
                if reasons:
                    raise RuntimeError(
                        '未重启：'
                        + '；'.join(reasons)
                        + '。处理后重试，或加 --force'
                    )
            stop(pid)
        process, log_path = start(args.tracker_args)
        status = wait_ready(process, log_path)
    except (RuntimeError, OSError, subprocess.CalledProcessError) as error:
        LOG.error('%s', error)
        return 1
    grasp = status.get('grasp') or {}
    LOG.info(
        '状态：state=%s model_ready=%s error=%s 找框=%s 夹持=%s',
        status.get('state'),
        status.get('model_ready'),
        status.get('error'),
        status.get('detector_backend'),
        grasp.get('held_object'),
    )
    if not status.get('model_ready') or status.get('error'):
        LOG.error(
            '服务已监听，但模型未就绪或服务报告了错误，请查看日志 %s',
            log_path.relative_to(ROOT),
        )
        return 1
    LOG.info('重启完成：http://%s:%d/', HOST, PORT)
    return 0


if __name__ == '__main__':
    sys.exit(main())
