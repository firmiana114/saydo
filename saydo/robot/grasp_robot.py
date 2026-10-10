"""Narrow RealMan JSON transport. Motion requires an explicitly enabled client."""

import json
import socket
import time


READS = {
    'get_current_arm_state',
    'get_current_tool_frame',
    'get_current_work_frame',
    'get_joint_en_state',
    'get_joint_err_flag',
    'get_arm_software_info',
    'get_rm_plus_mode',
    'get_rm_plus_base_info',
    'get_rm_plus_state_info',
    'get_rm_plus_reg',
}

# 连续关节运动里，每条指令等待控制器接受确认的总时间上限，单位秒。
ACCEPTANCE_TIMEOUT_S = 5.

# 夹爪经末端 RM Plus 接口通信时，控制器应报告的模式值（即波特率）。
# 报告 0 表示该接口未启用，此时读不到夹爪状态，也无法下发开合指令。
RM_PLUS_MODE = 115200


def rm_plus_mode_error(mode: object) -> str:
    """返回夹爪通信模式不符时给操作者看的提示，写明当前值与要求值。

    Args:
        mode: 控制器对 get_rm_plus_mode 的应答里的 mode 字段。

    Returns:
        可直接作为异常信息的中文提示。
    """
    return (
        f'夹爪 RM Plus 通信未就绪（当前模式 {mode}，要求 {RM_PLUS_MODE}）；'
        '请在示教器恢复末端 RM Plus 模式后重试'
    )


class RobotClient:
    def __init__(self, ip, port=8080, writes=False):
        self.address = (ip, port)
        self.writes = writes
        self.motion_log = []

    def query(self, command, **values):
        if command not in READS:
            raise ValueError('只读接口拒绝写入命令')
        reply = self._request({'command': command, **values})
        required = {
            'get_rm_plus_state_info': 'state_info',
            'get_rm_plus_base_info': 'base_info',
        }
        if command in required:
            field = required[command]
            if (
                not isinstance(reply, dict)
                or reply.get('get_state') is False
                or not isinstance(reply.get(field), dict)
            ):
                raise RuntimeError(
                    f'夹爪 RM Plus 查询 {command} 未返回有效 {field}；'
                    '请检查末端通信模式、供电和连接'
                )
        return reply

    def _request(self, payload):
        command = payload['command']
        with socket.create_connection(self.address, timeout=2) as connection:
            connection.settimeout(2)
            connection.sendall((json.dumps(payload) + '\r\n').encode())
            with connection.makefile('rb') as stream:
                for _ in range(16):
                    line = stream.readline(16384)
                    if not line:
                        raise RuntimeError('控制器断开连接')
                    reply = json.loads(line)
                    expected = command.removeprefix('get_')
                    if (
                        reply.get('command') in (command, expected)
                        or reply.get('state') == expected
                    ):
                        return reply
        raise RuntimeError('未收到匹配的控制器响应')

    def register(self, address):
        reply = self.query('get_rm_plus_reg', addr=address, length=1)
        values = reply.get('plus_reg_data')
        if (
            not isinstance(values, list)
            or len(values) != 1
            or not isinstance(values[0], int)
        ):
            raise ValueError(
                f'寄存器 {address} 没有返回实际数值，不能验证限力/限速'
            )
        return values[0]

    def set_register(self, address, value):
        if not self.writes or address not in (1180, 1220, 1470, 1023):
            raise ValueError('寄存器写入未启用或不在许可范围')
        if (
            not isinstance(value, int)
            or not 0 <= value <= 1000
            or address == 1023 and value != 1
        ):
            raise ValueError('寄存器值越界')
        reply = self._request(
            {
                'command': 'set_rm_plus_reg',
                'addr': address,
                'length': 1,
                'data': [value],
            }
        )
        if reply.get('set_state') is not True:
            raise RuntimeError('夹爪命令未被接受')

    def move_gripper(self, position):
        if (
            not self.writes
            or not isinstance(position, int)
            or not 0 <= position <= 1000
        ):
            raise ValueError('夹爪写入未启用或开度越界')
        if (
            self._request(
                {'command': 'hand_follow_pos', 'hand_pos': [position]}
            ).get('set_state')
            is not True
        ):
            raise RuntimeError('夹爪开度命令未被接受')

    def stop(self):
        if not self.writes:
            return
        failures = []
        for payload in [
            {'command': 'set_arm_stop'},
            {
                'command': 'set_rm_plus_reg',
                'addr': 1023,
                'length': 1,
                'data': [1],
            },
        ]:
            try:
                result = self._request(payload)
                if result.get('arm_stop', result.get('set_state')) is not True:
                    failures.append(str(result))
            except Exception as error:
                failures.append(str(error))
        if failures:
            raise RuntimeError('停止指令未全部确认：' + '; '.join(failures))

    def move_linear(self, pose, speed, cancelled, watchdog, timeout=60):
        if (
            not self.writes
            or len(pose) != 6
            or not all(isinstance(v, int) for v in pose)
            or not 1 <= speed <= 100
        ):
            raise ValueError('运动未启用或运动参数无效')
        command = {
            'command': 'movel',
            'pose': pose,
            'v': speed,
            'r': 0,
            'trajectory_connect': 0,
        }
        return self._move(command, cancelled, watchdog, timeout)

    def move_joint_pose(self, pose, speed, cancelled, watchdog, timeout=60):
        if (
            not self.writes
            or len(pose) != 6
            or not all(type(v) is int for v in pose)
            or not 1 <= speed <= 100
        ):
            raise ValueError('目标位姿关节运动未启用或参数无效')
        return self._move(
            {
                'command': 'movej_p',
                'pose': pose,
                'v': speed,
                'r': 0,
                'trajectory_connect': 0,
            },
            cancelled,
            watchdog,
            timeout,
        )

    def move_joints(self, joints, speed, cancelled, watchdog, timeout=60):
        if (
            not self.writes
            or len(joints) != 7
            or not all(type(v) is int for v in joints)
            or not 1 <= speed <= 100
        ):
            raise ValueError('关节运动未启用或参数无效')
        return self._move(
            {
                'command': 'movej',
                'joint': joints,
                'v': speed,
                'r': 0,
                'trajectory_connect': 0,
            },
            cancelled,
            watchdog,
            timeout,
        )

    def move_joints_chain(
        self,
        targets: list,
        speed: int,
        cancelled: object,
        watchdog: object,
        timeout: float = 60,
    ) -> None:
        """让机械臂依次经过一串关节角目标、中途不停：会发送运动指令。

        做法是用控制器的“轨迹连接”：前面每条 `movej` 都带
        `trajectory_connect=1`（与下一条一起规划，先不执行），最后一条
        带 0，控制器把它们作为一整条轨迹执行。逐条单独发送时每一步
        各自加速、停下；连起来之后中间点不停。

        2026-10-10 时这条路只有试验脚本
        `scripts/trajectory_connect_test.py` 在用：连接后中间点是精确
        经过还是抄近路、到位反馈怎么给，厂家文档没有写，以实测为准。

        Args:
            targets: 关节角目标序列，每个 7 个整数（0.001 度），至少
                一个；只有一个时等同于 `move_joints`。
            speed: 速度百分比，1～100。
            cancelled: 取消标志（`threading.Event`），置位后停止等待
                并报错；不会自动发送停止指令，由调用方处理。
            watchdog: 等待期间每 0.3 秒调用一次的检查函数，它抛出的
                异常原样传出。
            timeout: 从最后一条指令被接受起，等待整条轨迹完成的时间
                上限，单位秒。

        Raises:
            ValueError: 没有启用运动，或参数无效。
            RuntimeError: 控制器拒绝某一条指令、报告轨迹没有成功完成、
                断开连接，或操作员取消。
            TimeoutError: 等待接受确认或轨迹完成超时。
        """
        if (
            not self.writes
            or not targets
            or any(
                len(joints) != 7 or not all(type(v) is int for v in joints)
                for joints in targets
            )
            or not 1 <= speed <= 100
        ):
            raise ValueError('连续关节运动未启用或参数无效')
        started = time.monotonic()
        record = {
            'started_at': time.time(),
            'stage': getattr(self, 'motion_stage', None),
            'command': {
                'command': 'movej_chain',
                'joints': [list(joints) for joints in targets],
                'v': speed,
            },
            'timeout_s': timeout,
            'send_completed': False,
            'accepted': False,
            'accepted_count': 0,
            'responses': [],
            'responses_dropped': 0,
        }
        self.motion_log.append(record)
        try:
            self._execute_chain(
                targets, speed, cancelled, watchdog, timeout, record
            )
            record['outcome'] = 'completed'
        except Exception as error:
            record.update(
                outcome='failed',
                error=str(error),
                error_type=type(error).__name__,
            )
            raise
        finally:
            record['elapsed_s'] = time.monotonic() - started

    def _execute_chain(
        self,
        targets: list,
        speed: int,
        cancelled: object,
        watchdog: object,
        timeout: float,
        record: dict,
    ) -> None:
        """在一条连接上逐条发送连接的 `movej`，并等到整条轨迹完成。

        每条指令等到控制器的接受确认后再发下一条。完成的标志是一条
        `current_trajectory_state` 消息，其中 `trajectory_state` 为真且
        `trajectory_connect` 不为 1（1 表示后面还连着轨迹）。控制器的
        全部应答连同时间记在 `record['responses']` 里。
        """
        if cancelled.is_set():
            raise RuntimeError('操作员取消抓取')
        watchdog()
        with socket.create_connection(self.address, timeout=2) as connection:
            connection.settimeout(.2)
            began = time.monotonic()
            data = b''
            sent = accepted = 0
            deadline = began + ACCEPTANCE_TIMEOUT_S
            last_check = 0.
            while True:
                if sent == accepted and sent < len(targets):
                    payload = {
                        'command': 'movej',
                        'joint': list(targets[sent]),
                        'v': speed,
                        'r': 0,
                        'trajectory_connect': int(sent < len(targets) - 1),
                    }
                    connection.sendall((json.dumps(payload) + '\r\n').encode())
                    sent += 1
                    record['send_completed'] = sent == len(targets)
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f'连续关节运动等待超时（已发送 {sent} 条、控制器'
                        f'已接受 {accepted} 条，共 {len(targets)} 条）'
                    )
                if cancelled.is_set():
                    raise RuntimeError('操作员取消抓取')
                if time.monotonic() - last_check > .3:
                    watchdog()
                    last_check = time.monotonic()
                try:
                    chunk = connection.recv(16384)
                except socket.timeout:
                    continue
                if not chunk:
                    raise RuntimeError('运动期间控制器断开连接')
                data += chunk
                if len(data) > 65536:
                    raise RuntimeError('控制器响应异常')
                while b'\n' in data:
                    line, data = data.split(b'\n', 1)
                    if not line.strip():
                        continue
                    raw = line.decode('utf-8', errors='replace')
                    response = {
                        'elapsed_s': time.monotonic() - began,
                        'raw': raw[:2048],
                        'truncated': len(raw) > 2048,
                    }
                    if len(record['responses']) >= 96:
                        record['responses'].pop(0)
                        record['responses_dropped'] += 1
                    record['responses'].append(response)
                    reply = json.loads(line)
                    if not isinstance(reply, dict):
                        raise ValueError('控制器运动应答不是 JSON 对象')
                    if reply.get('command') == 'movej':
                        if reply.get('receive_state') is not True:
                            raise RuntimeError(
                                f'控制器拒绝第 {accepted + 1} 条连续关节'
                                f'运动指令：{raw[:512]}'
                            )
                        accepted += 1
                        record['accepted_count'] = accepted
                        record['accepted'] = accepted == len(targets)
                        if record['accepted']:
                            # 全部接受之后才开始计整条轨迹的时间。
                            deadline = time.monotonic() + timeout
                    if reply.get('state') == 'current_trajectory_state':
                        if reply.get('trajectory_state') is not True:
                            raise RuntimeError(
                                f'控制器报告连续关节运动没有成功完成：'
                                f'{raw[:512]}'
                            )
                        if reply.get('trajectory_connect') == 1:
                            # 到了中间的一个点，后面还连着轨迹。
                            continue
                        record['completion_reply'] = response
                        if accepted < len(targets):
                            raise RuntimeError(
                                f'只接受了 {accepted} 条指令就收到整条轨迹'
                                f'完成的消息：{raw[:512]}'
                            )
                        return

    def _move(self, command, cancelled, watchdog, timeout):
        started = time.monotonic()
        record = {
            'started_at': time.time(),
            'stage': getattr(self, 'motion_stage', None),
            'command': command,
            'timeout_s': timeout,
            'send_completed': False,
            'accepted': False,
            'responses': [],
            'responses_dropped': 0,
        }
        self.motion_log.append(record)
        try:
            self._execute_move(command, cancelled, watchdog, timeout, record)
            record['outcome'] = 'completed'
        except Exception as error:
            record.update(
                outcome='failed',
                error=str(error),
                error_type=type(error).__name__,
            )
            raise
        finally:
            record['elapsed_s'] = time.monotonic() - started

    def _execute_move(self, command, cancelled, watchdog, timeout, record):
        if not self.writes:
            raise ValueError('运动未启用')
        if cancelled.is_set():
            raise RuntimeError('操作员取消抓取')
        watchdog()
        with socket.create_connection(self.address, timeout=2) as connection:
            connection.settimeout(.2)
            connection.sendall((json.dumps(command) + '\r\n').encode())
            record['send_completed'] = True
            deadline = time.monotonic() + timeout
            data = b''
            accepted = False
            last_check = 0.
            while time.monotonic() < deadline:
                if cancelled.is_set():
                    raise RuntimeError('操作员取消抓取')
                if time.monotonic() - last_check > .3:
                    watchdog()
                    last_check = time.monotonic()
                try:
                    chunk = connection.recv(16384)
                except socket.timeout:
                    continue
                if not chunk:
                    raise RuntimeError('运动期间控制器断开连接')
                data += chunk
                if len(data) > 65536:
                    raise RuntimeError('控制器响应异常')
                while b'\n' in data:
                    line, data = data.split(b'\n', 1)
                    if not line.strip():
                        continue
                    raw = line.decode('utf-8', errors='replace')
                    response = {
                        'elapsed_s': time.monotonic() - (deadline - timeout),
                        'raw': raw[:2048],
                        'truncated': len(raw) > 2048,
                    }
                    if len(record['responses']) >= 32:
                        record['responses'].pop(0)
                        record['responses_dropped'] += 1
                    record['responses'].append(response)
                    reply = json.loads(line)
                    if not isinstance(reply, dict):
                        raise ValueError('控制器运动应答不是 JSON 对象')
                    if reply.get('command') == command['command']:
                        if reply.get('receive_state') is not True:
                            raise RuntimeError(
                                f"控制器拒绝 {command['command']} 指令："
                                f"{raw[:512]}"
                            )
                        accepted = True
                        record['accepted'] = True
                        record['acceptance_reply'] = response
                    if reply.get('state') == 'current_trajectory_state':
                        record['completion_reply'] = response
                        if not accepted:
                            raise RuntimeError(
                                f"收到运动结果但尚未收到 {command['command']} "
                                f"的接受确认：{raw[:512]}"
                            )
                        if reply.get('trajectory_state') is not True:
                            raise RuntimeError(
                                f"控制器报告 {command['command']} "
                                f"轨迹未成功完成：{raw[:512]}"
                            )
                        return
            raise TimeoutError(
                f"{command['command']} 等待运动完成超时（{timeout} 秒，"
                f"已收到接受确认：{accepted}）"
            )
