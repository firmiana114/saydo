"""分层搜索 `ik_beam_search.h` 的测试：不需要厂家算法库，只需要 C++ 编译器。"""

import pathlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]

SOURCE = r'''
#include "saydo/robot/ik/ik_beam_search.h"
#include <cassert>
#include <limits>
#include <stdexcept>

using saydo_ik::Joints;

// 第 1 个关节取指定值、其余为 0 的一组关节角。
static Joints At(float first) {
  Joints q{};
  q[0] = first;
  return q;
}

int main() {
  const Joints start{};
  auto always = [](int, const Joints&, const Joints&) { return true; };

  // 第一个路点有两个解：0（最省力）和 5。只有从 5 出发才能到第二个路点。
  auto fork = [](int index, const Joints& parent) {
    std::vector<Joints> out;
    if (index == 0) {
      out = {At(0), At(5)};
    } else if (parent[0] == 5) {
      out.push_back(At(6));
    }
    return out;
  };
  // 只保留一个解时走进死路；保留两个时找到完整路径。
  auto greedy = saydo_ik::Search(start, {10, 10}, fork, always, 1);
  assert(!greedy.complete && greedy.failed_index == 1);
  auto beam = saydo_ik::Search(start, {10, 10}, fork, always, 2);
  assert(beam.complete && beam.failed_index == -1);
  assert(beam.path.size() == 2);
  assert(beam.path[0][0] == 5 && beam.path[1][0] == 6);
  assert(beam.cost == 6);
  assert(beam.widths.size() == 2 && beam.widths[0] == 2);

  // 相邻关节角变化超过上限的解不会被采用。
  auto blocked = saydo_ik::Search(start, {1, 1}, fork, always, 2);
  assert(!blocked.complete && blocked.failed_index == 1);

  // 回调抛出的异常原样传出，用来中止搜索。
  bool cancelled = false;
  try {
    saydo_ik::Search(
        start, {10, 10},
        [](int, const Joints&) -> std::vector<Joints> {
          throw std::runtime_error("cancel");
        },
        always, 2);
  } catch (const std::runtime_error&) {
    cancelled = true;
  }
  assert(cancelled);

  // 不是有限数的解被丢弃。
  auto invalid = [](int, const Joints&) {
    return std::vector<Joints>{
        At(std::numeric_limits<float>::quiet_NaN())};
  };
  assert(!saydo_ik::Search(start, {10}, invalid, always, 2).complete);

  // 某一步被判为不可行时，同一个解可以改由别的上一个解到达。
  // 第一个路点的解是 1 和 3，第二个路点只有解 2；从 1 走到 2 不可行。
  auto shared = [](int index, const Joints&) {
    return index == 0 ? std::vector<Joints>{At(1), At(3)}
                      : std::vector<Joints>{At(2)};
  };
  int checks = 0;
  auto not_from_one = [&](int index, const Joints& parent, const Joints&) {
    ++checks;
    return !(index == 1 && parent[0] == 1);
  };
  auto detour = saydo_ik::Search(start, {10, 10}, shared, not_from_one, 2);
  assert(detour.complete);
  assert(detour.path[0][0] == 3 && detour.path[1][0] == 2);
  // 这项检查只对将要保留的解调用：第一层 2 次，第二层 2 次。
  assert(checks == 4);
  // 所有连接都不可行时在那个路点失败。
  auto never = [](int index, const Joints&, const Joints&) {
    return index == 0;
  };
  auto stuck = saydo_ik::Search(start, {10, 10}, shared, never, 2);
  assert(!stuck.complete && stuck.failed_index == 1);

  // 保留的名额先给差别大的摆法：0、1、2 彼此接近，100 离得远。只留两个
  // 时留下最省力的 0 和离得远的 100，而不是 0 和 1。
  auto spread = [](int index, const Joints& parent) {
    std::vector<Joints> out;
    if (index == 0) {
      out = {At(0), At(1), At(2), At(100)};
    } else if (parent[0] == 100) {
      out.push_back(At(101));
    }
    return out;
  };
  auto diverse = saydo_ik::Search(start, {360, 10}, spread, always, 2);
  assert(diverse.complete && diverse.path[0][0] == 100);

  // 宽度和变化上限无效时报错。
  for (int width : {0, saydo_ik::kMaxBeamWidth + 1}) {
    bool refused = false;
    try {
      saydo_ik::Search(start, {10}, fork, always, width);
    } catch (const std::invalid_argument&) {
      refused = true;
    }
    assert(refused);
  }
  bool refused = false;
  try {
    saydo_ik::Search(start, {0}, fork, always, 2);
  } catch (const std::invalid_argument&) {
    refused = true;
  }
  assert(refused);
  return 0;
}
'''


class BeamSearchTests(unittest.TestCase):
    """编译并运行一个只依赖搜索头文件的小程序。"""

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
        # 不加 -DNDEBUG：assert 必须生效。
        subprocess.run(
            [
                compiler, '-std=c++17', '-O2', '-I', str(ROOT), str(source),
                '-o', str(cls.binary),
            ],
            check=True,
            capture_output=True,
        )

    def test_search_keeps_alternatives_and_respects_constraints(self):
        result = subprocess.run(
            [str(self.binary)], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
