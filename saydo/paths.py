"""项目目录的唯一来源：根目录，以及只在各机器本地维护的两个目录。

其他模块一律从这里取路径，不再各自用 __file__ 推算。以后调整 work/
或 config/ 的位置，只需要改这一个文件。

被当作独立脚本直接启动的少数文件（qwen_worker.py、global_camera.py）
不能导入本模块，它们各自保留一行按 __file__ 推算的写法。
"""

import pathlib

# 仓库根目录：本文件位于 <根目录>/saydo/paths.py。
ROOT = pathlib.Path(__file__).resolve().parents[1]
# 模型、权重、运行环境、日志与验证记录；不入 Git。
WORK = ROOT / 'work'
# 各机器本地的标定与运行配置；不入 Git。
CONFIG = ROOT / 'config'
