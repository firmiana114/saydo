"""Narrow RealMan JSON transport. Motion requires an explicitly enabled client."""
import json
import socket
import time


READS = {'get_current_arm_state', 'get_current_tool_frame', 'get_current_work_frame', 'get_joint_en_state',
         'get_joint_err_flag', 'get_arm_software_info', 'get_rm_plus_mode', 'get_rm_plus_base_info',
         'get_rm_plus_state_info', 'get_rm_plus_reg'}


class RobotClient:
    def __init__(self, ip, port=8080, writes=False):
        self.address = (ip, port)
        self.writes = writes

    def query(self, command, **values):
        if command not in READS:
            raise ValueError('只读接口拒绝写入命令')
        return self._request({'command': command, **values})

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
                    if reply.get('command') in (command, expected) or reply.get('state') == expected:
                        return reply
        raise RuntimeError('未收到匹配的控制器响应')

    def register(self, address):
        reply = self.query('get_rm_plus_reg', addr=address, length=1)
        values = reply.get('plus_reg_data')
        if not isinstance(values, list) or len(values) != 1 or not isinstance(values[0], int):
            raise ValueError(f'寄存器 {address} 没有返回实际数值，不能验证限力/限速')
        return values[0]

    def set_register(self, address, value):
        if not self.writes or address not in (1180, 1220, 1470, 1023):
            raise ValueError('寄存器写入未启用或不在许可范围')
        if not isinstance(value, int) or not 0 <= value <= 1000 or address == 1023 and value != 1:
            raise ValueError('寄存器值越界')
        reply = self._request({'command': 'set_rm_plus_reg', 'addr': address, 'length': 1, 'data': [value]})
        if reply.get('set_state') is not True:
            raise RuntimeError('夹爪命令未被接受')

    def move_gripper(self, position):
        if not self.writes or not isinstance(position, int) or not 0 <= position <= 1000:
            raise ValueError('夹爪写入未启用或开度越界')
        if self._request({'command': 'hand_follow_pos', 'hand_pos': [position]}).get('set_state') is not True:
            raise RuntimeError('夹爪开度命令未被接受')

    def stop(self):
        if not self.writes:
            return
        failures = []
        for payload in [{'command': 'set_arm_stop'}, {'command': 'set_rm_plus_reg', 'addr': 1023, 'length': 1, 'data': [1]}]:
            try:
                result = self._request(payload)
                if result.get('arm_stop', result.get('set_state')) is not True:
                    failures.append(str(result))
            except Exception as error:
                failures.append(str(error))
        if failures:
            raise RuntimeError('停止指令未全部确认：' + '; '.join(failures))

    def move_linear(self, pose, speed, cancelled, watchdog, timeout=60):
        if not self.writes or len(pose) != 6 or not all(isinstance(v, int) for v in pose) or not 1 <= speed <= 100:
            raise ValueError('运动未启用或运动参数无效')
        command = {'command': 'movel', 'pose': pose, 'v': speed, 'r': 0, 'trajectory_connect': 0}
        return self._move(command, cancelled, watchdog, timeout)

    def move_joints(self, joints, speed, cancelled, watchdog, timeout=60):
        if not self.writes or len(joints) != 7 or not all(type(v) is int for v in joints) or not 1 <= speed <= 100:
            raise ValueError('关节运动未启用或参数无效')
        return self._move({'command':'movej', 'joint':joints, 'v':speed, 'r':0, 'trajectory_connect':0},
                          cancelled, watchdog, timeout)

    def _move(self, command, cancelled, watchdog, timeout):
        if not self.writes:
            raise ValueError('运动未启用')
        if cancelled.is_set():
            raise RuntimeError('操作员取消抓取')
        watchdog()
        with socket.create_connection(self.address, timeout=2) as connection:
            connection.settimeout(.2)
            connection.sendall((json.dumps(command) + '\r\n').encode())
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
                    reply = json.loads(line)
                    if reply.get('command') == command['command']:
                        if reply.get('receive_state') is not True:
                            raise RuntimeError('控制器拒绝运动指令')
                        accepted = True
                    if reply.get('state') == 'current_trajectory_state':
                        if not accepted or reply.get('trajectory_state') is not True:
                            raise RuntimeError('轨迹未成功完成')
                        return
            raise TimeoutError('运动未在时限内完成')
