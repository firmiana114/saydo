"""`arm_collision.h` 与 `collision_model.py` 是否算出同样的间距。

逆解检查器在挑选手肘摆法时用 C++ 版本排除会碰撞的姿态，最终放行与否
由 Python 版本决定。两边必须一致，否则检查器会留下 Python 不认可的
摆法，或者白白丢掉可行的摆法。不需要厂家算法库，只需要 C++ 编译器。
"""

import pathlib
import shutil
import subprocess
import tempfile
import unittest

import numpy as np
from saydo.robot import collision_model

ROOT = pathlib.Path(__file__).resolve().parents[2]

SOURCE = r'''
#include "saydo/robot/ik/arm_collision.h"
#include <cstdio>
#include <iostream>

// 读入外形、若干姿态和若干对姿态，逐个打印间距和是否放行。
int main() {
  saydo_ik::ArmModel model;
  if (!saydo_ik::ReadArmModel(std::cin, &model)) {
    return 2;
  }
  auto read = [](saydo_ik::Joints* q) {
    for (float& value : *q) {
      std::cin >> value;
    }
  };
  int count = 0;
  std::cin >> count;
  for (int i = 0; i < count; ++i) {
    saydo_ik::Joints q{};
    read(&q);
    const auto clearance = saydo_ik::MeasureClearance(model, q);
    std::printf(
        "%.9f %.9f %d %.9f\n", clearance.table_m, clearance.pole_m,
        int(saydo_ik::ConfigurationClear(model, q)), clearance.other_arm_m);
  }
  std::cin >> count;
  for (int i = 0; i < count; ++i) {
    saydo_ik::Joints from{}, to{};
    read(&from);
    read(&to);
    std::printf("%d\n", int(saydo_ik::MotionClear(model, from, to)));
  }
  return std::cin ? 0 : 2;
}
'''

LOW_DEG = np.array([-178, -130, -178, -135, -178, -128, -360.])


class NativeCollisionTests(unittest.TestCase):
    """用同一批随机姿态对比两种实现。"""

    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('g++') or shutil.which('clang++')
        if not compiler:
            raise unittest.SkipTest('没有 C++ 编译器')
        cls.folder = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.folder.cleanup)
        folder = pathlib.Path(cls.folder.name)
        source = folder / 'test.cc'
        cls.binary = folder / 'test'
        source.write_text(SOURCE)
        subprocess.run(
            [
                compiler, '-std=c++17', '-O2', '-I', str(ROOT), str(source),
                '-o', str(cls.binary),
            ],
            check=True,
            capture_output=True,
        )
        cls.tool = np.eye(4)
        cls.tool[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        cls.tool[:3, 3] = [.001, -.002, .2275]
        cls.environment = collision_model.Environment(
            table_z_m=-.0275,
            pole_xy_m=(-.0782, -.2633),
            pole_radius_m=.0141,
            pole_margin_m=.03,
        )

    def run_native(self, environment, joints, pairs=()):
        values = collision_model.native_model_values(self.tool, environment)
        values.append(len(joints))
        values.extend(np.asarray(joints, dtype=float).ravel().tolist())
        values.append(len(pairs))
        values.extend(np.asarray(pairs, dtype=float).ravel().tolist())
        result = subprocess.run(
            [str(self.binary)],
            input=' '.join(str(value) for value in values) + '\n',
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = [line.split() for line in result.stdout.splitlines()]
        return rows[:len(joints)], rows[len(joints):]

    def random_joints(self, count, seed):
        random = np.random.default_rng(seed)
        return LOW_DEG + random.random((count, 7)) * -2 * LOW_DEG

    def test_clearances_match_the_python_model(self):
        joints = self.random_joints(400, 1)
        rows, _ = self.run_native(self.environment, joints)
        agreed = 0
        for q, (table, pole, clear, _) in zip(joints, rows):
            # 检查器内部用单精度保存关节角。
            state = collision_model.check_configuration(
                q.astype(np.float32).astype(float), self.tool,
                self.environment,
            )
            self.assertAlmostEqual(
                float(table), state['table_clearance_m'], places=6
            )
            self.assertAlmostEqual(
                float(pole), state['pole_clearance_m'], places=6
            )
            # C++ 比 Python 严 0.2 毫米：它放行的，Python 一定放行。
            if clear == '1':
                self.assertIsNone(state['violation'])
                agreed += 1
            elif state['violation'] is None:
                margin = min(
                    state['pole_clearance_m']
                    - self.environment.pole_margin_m,
                    state['table_clearance_m'] + .0005,
                )
                self.assertLess(margin, .00021)
            else:
                agreed += 1
        self.assertGreater(agreed, 390)
        self.assertTrue(any(row[2] == '1' for row in rows))
        self.assertTrue(any(row[2] == '0' for row in rows))

    def test_motion_it_accepts_passes_the_python_path_check(self):
        starts = self.random_joints(300, 2)
        # 终点离起点不远，这样才有相当一部分路径是放行的。
        ends = starts + np.random.default_rng(3).normal(0, 6, starts.shape)
        pairs = np.hstack([starts, ends]).astype(np.float32).astype(float)
        _, rows = self.run_native(self.environment, [], pairs)
        accepted = 0
        for pair, (clear,) in zip(pairs, rows):
            report = collision_model.check_path(
                [pair[:7], pair[7:]], self.tool, self.environment
            )
            # 途中各点由 C++ 检查；终点本身在搜索里另有一次检查。
            end_state = collision_model.check_configuration(
                pair[7:], self.tool, self.environment
            )
            if clear == '1' and end_state['violation'] is None:
                self.assertIsNone(report['violation'])
                accepted += 1
        self.assertGreater(accepted, 20)

    def test_nothing_is_checked_without_obstacles(self):
        joints = self.random_joints(20, 4)
        for environment in (None, collision_model.Environment()):
            rows, _ = self.run_native(environment, joints)
            self.assertTrue(
                all(row == ['inf', 'inf', '1', 'inf'] for row in rows)
            )

    def test_table_only_and_pole_only(self):
        joints = self.random_joints(50, 5)
        table_only = collision_model.Environment(table_z_m=.1)
        rows, _ = self.run_native(table_only, joints)
        self.assertTrue(all(row[1] == 'inf' for row in rows))
        for q, row in zip(joints, rows):
            state = collision_model.check_configuration(
                q.astype(np.float32).astype(float), self.tool, table_only
            )
            self.assertAlmostEqual(
                float(row[0]), state['table_clearance_m'], places=6
            )
        pole_only = collision_model.Environment(
            pole_xy_m=(.3, .1), pole_radius_m=.02, pole_margin_m=.01
        )
        rows, _ = self.run_native(pole_only, joints)
        self.assertTrue(all(row[0] == 'inf' for row in rows))

    def test_other_arm_clearance_matches_the_python_model(self):
        # 另一台臂：底座在本臂旁边 0.52 米、转了约 170 度，摆成观测姿态。
        other_base = np.eye(4)
        yaw = np.radians(170.5)
        other_base[:2, :2] = [
            [np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]
        ]
        other_base[:3, 3] = [-.046, -.516, 0.]
        starts, ends, radii, _, _ = collision_model.capsules_in_base(
            [39., 43.7, 1.7, -71.6, -1.3, -116., -52.5], self.tool
        )
        environment = collision_model.Environment(
            other_arm_capsules=(
                starts @ other_base[:3, :3].T + other_base[:3, 3],
                ends @ other_base[:3, :3].T + other_base[:3, 3],
                radii,
            ),
            other_arm_margin_m=.03,
        )
        joints = self.random_joints(400, 6)
        rows, _ = self.run_native(environment, joints)
        near = blocked = 0
        for q, (_, _, clear, other) in zip(joints, rows):
            state = collision_model.check_configuration(
                q.astype(np.float32).astype(float), self.tool, environment
            )
            expected = state['other_arm_clearance_m']
            if other == 'inf':
                # 包围盒不相交时检查器不细算，此时间距一定大于余量。
                self.assertGreater(expected, .03)
            else:
                # 细算过的值不一定是最小的那个（离得远的被包围盒跳过），
                # 但只要真实间距在“余量 + 2 毫米”以内，它就是准确的。
                self.assertGreaterEqual(float(other), expected - 1e-6)
                if expected < .032:
                    self.assertAlmostEqual(float(other), expected, places=6)
                    near += 1
            if clear == '1':
                self.assertIsNone(state['violation'])
            elif state['violation'] is None:
                self.assertLess(expected - .03, .00021)
            else:
                blocked += 1
        self.assertGreater(near, 20)
        self.assertGreater(blocked, 20)
        self.assertTrue(any(row[2] == '1' for row in rows))

    def test_malformed_model_is_refused(self):
        values = collision_model.native_model_values(
            self.tool, self.environment
        )
        for broken in (values[:-3], [3] + values[1:], ['nan'] + values[1:]):
            result = subprocess.run(
                [str(self.binary)],
                input=' '.join(str(value) for value in broken) + '\n',
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main()
