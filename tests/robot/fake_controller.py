"""系统边界上的替身：扮演机械臂控制器，供多臂测试使用。

只替换与控制器通信的那一层（`grasp_robot.RobotClient`），被测的动作
控制器、接口和分发代码全部是真实的。
"""

from __future__ import annotations

import copy
import json
import pathlib
import threading
from typing import Any, Callable, Dict, List

import yaml

from saydo.robot import arms
from saydo.robot import grasp_robot

TOOL_RESPONSE = {
    'state': 'current_tool_frame',
    'tool_name': 'grip_tcp',
    'pose': [0, 0, 222500, 0, 0, 2611],
}
# 与上面的工具位姿对应的变换：只沿法兰 Z 轴平移（测试不关心绕轴转角）。
FLANGE_T_GRIP_TCP = [
    [1., 0., 0., 0.],
    [0., 1., 0., 0.],
    [0., 0., 1., .2225],
    [0., 0., 0., 1.],
]
IDLE_GRIPPER = {
    'sys_err': 0,
    'sys_state': 0,
    'dof_err': [0],
    'dof_state': [2],
    'pos': [1000],
    'speed': [0],
    'force': [0],
}


class FakeController:
    """一台控制器的替身：记录收到的指令，运动可以被测试卡住再放行。"""

    def __init__(self, ip: str) -> None:
        """创建替身；初始时七个关节都在零位，运动不被卡住。

        Args:
            ip: 这台控制器的地址，测试用它区分两台臂。
        """
        self.ip = ip
        self.joint = [0] * 7
        # 进入运动后置位；测试用它确认“这台臂确实已经在动”。
        self.moving = threading.Event()
        # 清除后运动会停在半途，直到测试重新置位。
        self.release = threading.Event()
        self.release.set()
        self.targets = []
        self.stops = 0
        self.stop_error = None
        self.write_flags = []

    def query(self, command: str, **unused: Any) -> dict:
        """应答一条只读查询，状态一律正常、机械臂静止。

        Args:
            command: 查询指令名。

        Returns:
            与真实控制器同形的应答。
        """
        replies = {
            'get_arm_software_info': {'Product_version': 'RM75-BI'},
            'get_joint_en_state': {'en_state': [1] * 7},
            'get_joint_err_flag': {'err_flag': [0] * 7},
            'get_current_tool_frame': TOOL_RESPONSE,
            'get_current_work_frame': {'frame_name': 'Base', 'pose': [0] * 6},
            'get_rm_plus_mode': {'mode': grasp_robot.RM_PLUS_MODE},
            'get_rm_plus_state_info': {'state_info': IDLE_GRIPPER},
            'get_current_arm_state': {
                'arm_state': {
                    'joint': list(self.joint),
                    'pose': [0, 0, 300000, 0, 0, 0],
                    'err': [0],
                }
            },
        }
        return copy.deepcopy(replies[command])

    def move_joints(
        self,
        target: List[int],
        speed: int,
        cancelled: threading.Event,
        watchdog: Callable[[], None],
        timeout: float = 60,
    ) -> None:
        """记录一次关节运动；被测试卡住时等到放行才“到位”。

        Args:
            target: 七个关节的目标，控制器原始单位（0.001 度）。
            speed: 速度百分比。
            cancelled: 取消标志，替身不使用。
            watchdog: 运动期间的检查函数，替身不调用。
            timeout: 超时，单位秒，替身不使用。
        """
        self.targets.append(list(target))
        self.moving.set()
        if not self.release.wait(5):
            raise AssertionError(f'{self.ip} 的运动一直没有被测试放行')
        self.joint = list(target)

    def stop(self) -> None:
        """记录一次停止指令；设置了 `stop_error` 时抛出它。"""
        self.stops += 1
        if self.stop_error is not None:
            raise self.stop_error


class Fleet:
    """按地址提供控制器替身，用来替换 `grasp_robot.RobotClient`。"""

    def __init__(self) -> None:
        self.controllers: Dict[str, FakeController] = {}

    def controller(self, ip: str) -> FakeController:
        """返回某个地址的控制器替身，首次使用时创建。

        Args:
            ip: 控制器地址。

        Returns:
            该地址的替身；同一地址始终是同一个对象。
        """
        return self.controllers.setdefault(ip, FakeController(ip))

    def __call__(
        self, ip: str, port: int = 8080, writes: bool = False
    ) -> FakeController:
        controller = self.controller(ip)
        controller.write_flags.append(writes)
        return controller


def write_arm_config(
    root: pathlib.Path,
    arm: arms.Arm,
    ip: str,
    reset_joint_deg: List[float],
) -> pathlib.Path:
    """在临时项目根目录下写出一台臂的抓取配置和工具记录。

    Args:
        root: 临时项目根目录。
        arm: 要配置的臂。
        ip: 写入配置的控制器地址。
        reset_joint_deg: 复位姿态的七个关节角度，单位度。

    Returns:
        抓取配置文件的路径。
    """
    tool_file = f'config/calibration/{arm.arm_id}_tool.yaml'
    tool_path = root / tool_file
    tool_path.parent.mkdir(parents=True, exist_ok=True)
    tool_path.write_text(
        yaml.safe_dump(
            {
                'controller_readback': {'tool_response_raw': TOOL_RESPONSE},
                'flange_T_grip_tcp': FLANGE_T_GRIP_TCP,
            }
        )
    )
    path = arm.config_path(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                'robot_ip': ip,
                'robot_port': 8080,
                'tool_file': tool_file,
                'handeye_file': 'unused',
                'arm_speed_percent': 10,
                'pregrasp_distance_m': .05,
                'table_z_m': 0.,
                'return_after_lift': {'joint_deg': reset_joint_deg},
                'gripper': {
                    'vendor': 'ZX',
                    'device_id': 1,
                    'force_units': 'native',
                    'force_control_verified': True,
                    'force_limit_raw': 10,
                    'force_target_raw': 10,
                    'open_position': 1000,
                    'closed_position': 0,
                    'contact_position_margin': 20,
                    'close_timeout_s': 2,
                },
            },
            indent=2,
        )
    )
    return path
