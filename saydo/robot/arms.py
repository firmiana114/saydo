"""机械臂登记表：程序认识哪几台臂，每台臂的配置和运行记录放在哪里。

本模块只回答“有哪些臂、各自的文件在哪、各自的设备锁在哪”。它不连接
控制器，也不保存地址、工具、标定这类随机器变化的参数——那些写在每台臂
自己的抓取配置文件里（`Arm.config_file`）。

为什么需要它：程序最初只为一台臂编写，配置、运行记录和设备锁都只有
一份。接入第二台臂后，每台臂要有各自的一份，否则两台臂的任务记录会
混在一起，也分不清是哪台臂正在运动。

1 臂沿用最初的路径（`config/grasp/trial.json`、`work/grasp/`），这样它的
历史记录、已有脚本和文档都不用改。
"""

from __future__ import annotations

import contextlib
import dataclasses
import fcntl
import pathlib
from typing import Iterator, Tuple

DEFAULT_ARM_ID = 'arm1'


@dataclasses.dataclass(frozen=True)
class Arm:
    """一台机械臂在程序里的身份。

    Attributes:
        arm_id: 程序内部的编号，出现在接口参数、任务记录和日志里。
        label: 给操作者看的名字。
        side: `left` 或 `right`，供智能体理解“左手、右手”。
        config_file: 抓取配置文件，相对项目根目录。里面有控制器地址、
            工具与手眼标定文件的位置、桌面高度、复位姿态等。
        placement_file: 放置设置文件，相对项目根目录。桌面放置范围是
            按该臂的基座坐标写的，所以每台臂一份。
        reorientation_file: 调姿设置文件，相对项目根目录。
        state_directory: 运行记录目录，相对项目根目录。任务诊断、场景
            快照、方案库和设备锁都在这里。
    """

    arm_id: str
    label: str
    side: str
    config_file: str
    placement_file: str
    reorientation_file: str
    state_directory: str

    def config_path(self, root: pathlib.Path) -> pathlib.Path:
        """返回抓取配置文件的完整路径。

        Args:
            root: 项目根目录。

        Returns:
            该臂抓取配置文件的路径；文件不一定存在。
        """
        return pathlib.Path(root) / self.config_file

    def state_path(self, root: pathlib.Path) -> pathlib.Path:
        """返回运行记录目录的完整路径。

        Args:
            root: 项目根目录。

        Returns:
            该臂运行记录目录的路径；目录不一定存在。
        """
        return pathlib.Path(root) / self.state_directory

    def lock_path(self, root: pathlib.Path) -> pathlib.Path:
        """返回设备锁文件的完整路径。

        设备锁是一个文件：谁以排他方式锁住它，谁才能指挥这台臂。每台
        臂一把，所以两台臂可以同时各执行一个任务。

        Args:
            root: 项目根目录。

        Returns:
            该臂设备锁文件的路径；文件不一定存在。
        """
        return self.state_path(root) / 'robot.lock'


ARMS: Tuple[Arm, ...] = (
    Arm(
        arm_id='arm1',
        label='1臂（左臂）',
        side='left',
        config_file='config/grasp/trial.json',
        placement_file='config/grasp/placement.json',
        reorientation_file='config/grasp/reorientation.json',
        state_directory='work/grasp',
    ),
    Arm(
        arm_id='arm2',
        label='2臂（右臂）',
        side='right',
        config_file='config/grasp/arm2.json',
        placement_file='config/grasp/arm2_placement.json',
        reorientation_file='config/grasp/arm2_reorientation.json',
        state_directory='work/grasp_arm2',
    ),
)


def by_id(arm_id: str) -> Arm:
    """按编号查找一台臂。

    Args:
        arm_id: 臂的编号，例如 `arm1`。

    Returns:
        对应的登记项。

    Raises:
        ValueError: 没有这个编号。
    """
    for arm in ARMS:
        if arm.arm_id == arm_id:
            return arm
    known = '、'.join(arm.arm_id for arm in ARMS)
    raise ValueError(f'未知的机械臂编号 {arm_id!r}，可用的编号：{known}')


def default() -> Arm:
    """返回缺省的那台臂（1 臂）。

    接口里不写臂的编号时用它，保证只有一台臂时的用法不变。

    Returns:
        1 臂的登记项。
    """
    return by_id(DEFAULT_ARM_ID)


def configured(root: pathlib.Path) -> Tuple[Arm, ...]:
    """列出这台机器上已经配置好的臂。

    判断依据是该臂的抓取配置文件是否存在。1 臂总是算在内：它的配置
    缺失属于部署错误，应当在使用时报出来，而不是让它悄悄消失。

    Args:
        root: 项目根目录。

    Returns:
        已配置的臂，顺序与登记表一致。
    """
    return tuple(
        arm
        for arm in ARMS
        if arm.arm_id == DEFAULT_ARM_ID or arm.config_path(root).is_file()
    )


@contextlib.contextmanager
def all_locked(root: pathlib.Path) -> Iterator[None]:
    """同时占住全部已配置臂的设备锁，用于修改各臂共用的数据。

    夹法库和物品模型设置是各臂共用的。修改它们时任何一台臂都不能正在
    执行任务，否则那台臂可能读到改了一半的数据。这里不等待：只要有
    一把锁被占用就立即报错，由调用方提示操作者稍后再试。

    各处都按登记表的顺序加锁，所以两处同时调用不会互相卡死。

    Args:
        root: 项目根目录。

    Yields:
        无。离开 `with` 块时释放全部锁。

    Raises:
        BlockingIOError: 有一台臂的设备锁正被占用。
    """
    with contextlib.ExitStack() as stack:
        for arm in configured(root):
            path = arm.lock_path(root)
            path.parent.mkdir(parents=True, exist_ok=True)
            lock = stack.enter_context(path.open('a'))
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
