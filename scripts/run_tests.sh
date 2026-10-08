#!/usr/bin/env bash
# 按各自的 Python 环境运行全部自动测试；只读，不连接机械臂、相机或服务。
#
# 用法（在任意目录）：
#   scripts/run_tests.sh              运行全部套件
#   scripts/run_tests.sh backend      只运行指定套件，可写多个
#
# 套件：backend（定位与抓取后端及脚本）、task_runner（任务执行器）、
# agent（MCP 服务）、devices（相机预览、手眼标定、物品采集）、
# frontend（页面脚本，需要 node）。某个环境在本机不存在时该套件记为跳过。

set -uo pipefail

readonly MCP_PYTHON="${HOME}/.local/share/saydo-mcp/venv/bin/python"
FAILED=0

# 用指定的解释器逐个目录运行 unittest。
# 参数：套件名、解释器路径、测试目录...
run_python_suite() {
  local name="$1"
  local python="$2"
  shift 2
  if [[ ! -x "${python}" ]]; then
    echo "== ${name}: 跳过（没有 ${python}）"
    return
  fi
  local directory
  for directory in "$@"; do
    echo "== ${name}: ${directory}"
    if ! "${python}" -m unittest discover \
        -s "${directory}" -t . -p 'test_*.py' -q 2>&1 | tail -n 4; then
      FAILED=1
    fi
  done
}

# 用 node 逐个运行页面脚本的测试文件。
run_frontend_suite() {
  if ! command -v node > /dev/null; then
    echo "== frontend: 跳过（没有 node）"
    return
  fi
  local file
  local passed=0
  for file in tests/*/*.cjs; do
    if node "${file}" > /dev/null 2>&1; then
      passed=$((passed + 1))
    else
      echo "== frontend: 失败 ${file}" >&2
      FAILED=1
    fi
  done
  echo "== frontend: ${passed} 个文件通过"
}

run_suite() {
  case "$1" in
    backend)
      SAYDO_IK_CHECKER="${PWD}/work/grasp/rm75_ik_check" run_python_suite \
        backend work/foundationpose_env/bin/python \
        tests/geometry tests/robot tests/perception tests/objects \
        tests/scene tests/manipulation tests/workbench tests/scripts
      ;;
    task_runner)
      run_python_suite task_runner work/task_runner_env/bin/python \
        tests/task_runner
      ;;
    agent)
      run_python_suite agent "${MCP_PYTHON}" tests/agent
      ;;
    devices)
      run_python_suite devices .venv/bin/python \
        tests/camera_monitor tests/calibration tests/modeling
      ;;
    frontend)
      run_frontend_suite
      ;;
    *)
      echo "未知的套件：$1" >&2
      FAILED=1
      ;;
  esac
}

main() {
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 2
  local suites=("$@")
  if [[ "${#suites[@]}" -eq 0 ]]; then
    suites=(backend task_runner agent devices frontend)
  fi
  local suite
  for suite in "${suites[@]}"; do
    run_suite "${suite}"
  done
  if [[ "${FAILED}" -ne 0 ]]; then
    echo "有测试失败或套件出错。" >&2
  fi
  exit "${FAILED}"
}

main "$@"
