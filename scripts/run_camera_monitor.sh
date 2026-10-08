#!/usr/bin/env bash
# 启动 8088 相机预览服务；参数原样传给 saydo.camera_monitor.server。
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
exec .venv/bin/python -m saydo.camera_monitor.server "$@"
