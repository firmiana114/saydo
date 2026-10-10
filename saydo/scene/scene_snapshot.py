"""Process-scoped frozen scene evidence and immutable calibration binding."""

import copy
import hashlib
import json
import pathlib
import re
import time
from typing import Optional
import uuid
import numpy as np
from saydo.geometry import grasp_geometry


def arm_still(a, b):
    pa, pb = (
        grasp_geometry.controller_pose(a['pose']),
        grasp_geometry.controller_pose(b['pose']),
    )
    if (
        np.max(np.abs(np.asarray(a['joint']) - b['joint'])) > 30
        or np.linalg.norm(pa[:3, 3] - pb[:3, 3]) > .0002
        or grasp_geometry.rotation_error(pa[:3, :3], pb[:3, :3]) > .003
    ):
        raise ValueError('采集场景期间机械臂移动，快照无效')


class SceneStore:
    """场景快照库，每台臂一个。

    一份快照记录某一时刻各目标物品在这台臂基座坐标里的位姿，以及
    采集时所依据的标定。快照只在采集它的那次后端运行期间有效，并且
    有时限。
    """
    def __init__(
        self, root: pathlib.Path, state_directory: str = 'work/grasp'
    ) -> None:
        """创建一台臂的场景快照库；不读写磁盘。

        快照里的物品位姿是按拍摄它的那台臂的基座坐标记录的，两台臂
        的基座坐标不同，所以每台臂一个库，不能混用。

        Args:
            root: 项目根目录。
            state_directory: 该臂的运行记录目录，相对项目根目录；
                缺省是 1 臂的目录。
        """
        self.root = pathlib.Path(root) / state_directory / 'scenes'
        self.session = uuid.uuid4().hex
        self.active = None
        self.records = {}

    def save(self, scene):
        self.root.mkdir(parents=True, exist_ok=True)
        path = self.root / (scene['scene_id'] + '.json')
        tmp = path.with_suffix('.tmp')
        tmp.write_text(
            json.dumps(scene, ensure_ascii=False, allow_nan=False, indent=2)
            + '\n'
        )
        tmp.replace(path)
        self.records[scene['scene_id']] = copy.deepcopy(scene)
        while len(self.records) > 8:
            oldest = next(
                k
                for k in self.records
                if k != self.active and k != scene['scene_id']
            )
            self.records.pop(oldest)

    def begin(self, targets):
        if self.active:
            old = self.get(self.active)
            old['state'] = 'superseded'
            self.save(old)
        scene = {
            'scene_id': uuid.uuid4().hex,
            'session_id': self.session,
            'state': 'capturing',
            'created_at': time.time(),
            'expires_at': time.time() + 1800,
            'environment_static': True,
            'coordinate_frame': 'Base',
            'instances': [{**row, 'state': 'pending'} for row in targets],
            'motion_sent': False,
        }
        self.save(scene)
        self.active = scene['scene_id']
        return scene

    def abandon(self) -> Optional[str]:
        """放弃当前的场景上下文：把它标为已被取代，此后不再有效。

        场景上下文在采集开始时建立，要到其中的物品全部放置完才结束。
        只规划不执行、或者采集失败时，它会一直留着。操作者在页面上
        直接点抓取时用本方法把它放弃，否则那次点击会被拒绝。

        Returns:
            被放弃的场景编号；本来就没有场景上下文时为 None。
        """
        ident = self.active
        if ident is None:
            return None
        scene = self.get(ident)
        scene['state'] = 'superseded'
        self.save(scene)
        self.active = None
        return ident

    def get(self, ident):
        if (
            not isinstance(ident, str)
            or not re.fullmatch('[a-f0-9]{32}', ident)
        ):
            raise ValueError('场景编号无效')
        if ident in self.records:
            return copy.deepcopy(self.records[ident])
        path = self.root / (ident + '.json')
        if (
            not path.is_file()
            or path.is_symlink()
            or path.stat().st_size > 2 * 1024 * 1024
        ):
            raise ValueError('场景不存在或无效')
        return json.loads(path.read_text())

    def require(self, ident):
        scene = self.get(ident)
        if scene.get('session_id') != self.session:
            raise ValueError('后端已重启，请重新采集场景')
        if scene['state'] != 'ready' or ident != self.active:
            raise ValueError('场景未全部准备齐全或已失效')
        if time.time() > scene['expires_at']:
            raise ValueError('场景超过30分钟有效期，请重新采集')
        if any(row['state'] == 'pending' for row in scene['instances']):
            raise ValueError('场景目标未全部定位')
        return scene

    def item(self, ident, instance_id):
        scene = self.require(ident)
        row = next(
            (r for r in scene['instances'] if r['instance_id'] == instance_id),
            None,
        )
        if row is None:
            raise ValueError('场景中没有该实例')
        return scene, row

    def transition(self, ident, instance_id, state, pose=None):
        scene = self.get(ident)
        row = next(
            r for r in scene['instances'] if r['instance_id'] == instance_id
        )
        row['state'] = state
        row['static_pose_usable'] = state == 'static'
        if pose is not None:
            row['predicted_base_T_object'] = grasp_geometry.transform(
                pose
            ).tolist()
        self.save(scene)
        if all(
            r['role'] != 'source' or r['state'] == 'placed_unverified'
            for r in scene['instances']
        ):
            scene['state'] = 'completed'
            self.save(scene)
            self.active = None
