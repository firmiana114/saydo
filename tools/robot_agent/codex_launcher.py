#!/usr/bin/env python3
"""Start the MSI Codex CLI with credentials from SayDo's private dotenv file."""
import os
import json
from pathlib import Path
import sys
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.task_runner.config import Config


def main():
    try:
        config = Config.load(ROOT / '.env')
        binary = Path.home() / '.local/lib/codex/0.156.0/bin/codex'
        if not binary.is_file():
            raise ValueError('未找到 MSI Codex CLI 0.156.0 的官方可执行文件')
        env = os.environ.copy()
        env['SAYDO_CODEX_API_KEY'] = config.api_key
        if config.llm_proxy:
            for name in ('HTTP_PROXY', 'HTTPS_PROXY', 'http_proxy', 'https_proxy'):
                env[name] = config.llm_proxy
        # Model traffic uses the proxy; local services must remain directly reachable.
        bypass = env.get('NO_PROXY', env.get('no_proxy', ''))
        hosts = ['127.0.0.1', 'localhost', urlsplit(config.robot_url).hostname or '']
        env['NO_PROXY'] = env['no_proxy'] = ','.join(filter(None, [bypass, *hosts]))
        base_url = config.responses_url.removesuffix('/responses')
        args = [str(binary), '-c', 'model_providers.trendengine.base_url=' + json.dumps(base_url), *sys.argv[1:]]
        os.execve(binary, args, env)
    except (OSError, ValueError):
        # Never print parser errors that could contain credential file contents.
        print('Codex 启动失败：请检查项目 .env（权限600）及官方 CLI 安装。', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
