#!/usr/bin/env python3
"""Start the MSI Codex CLI with credentials from SayDo's private dotenv file."""

import os
import json
from pathlib import Path
import sys
import tomllib
from urllib.parse import urlsplit

from saydo import paths

ROOT = paths.ROOT

from saydo.task_runner.config import Config


def disabled_skills(roots, known_paths=()):
    """Disable discovered skills without reading their instruction contents."""
    paths = {
        str(
            (
                Path(path)
                if Path(path).name == 'SKILL.md'
                else Path(path) / 'SKILL.md'
            ).absolute()
        )
        for path in known_paths
    }
    visited = set()
    for root in roots:
        for directory, children, files in os.walk(root, followlinks=True):
            resolved = Path(directory).resolve()
            if resolved in visited:
                children[:] = []
                continue
            visited.add(resolved)
            if 'SKILL.md' in files:
                paths.add(str(Path(directory).absolute() / 'SKILL.md'))
                paths.add(str(resolved / 'SKILL.md'))
    return (
        'skills.config=['
        + ','.join(
            '{path=' + json.dumps(path) + ',enabled=false}'
            for path in sorted(paths)
        )
        + ']'
    )


def main():
    try:
        config = Config.load(ROOT / '.env')
        binary = Path.home() / '.local/lib/codex/0.156.0/bin/codex'
        if not binary.is_file():
            raise ValueError('未找到 MSI Codex CLI 0.156.0 的官方可执行文件')
        env = os.environ.copy()
        # Keep user-level instructions, skills, memories and coding sessions out
        # of the dedicated robot operator, even when CODEX_HOME was inherited.
        agent_home = Path.home() / '.local/share/saydo-codex/codex-home'
        if not (agent_home / 'config.toml').is_file():
            raise ValueError('缺少任务专用 Codex 配置')
        agent_config = tomllib.loads((agent_home / 'config.toml').read_text())
        env['CODEX_HOME'] = str(agent_home)
        env['SAYDO_CODEX_API_KEY'] = config.api_key
        if config.llm_proxy:
            for name in (
                'HTTP_PROXY', 'HTTPS_PROXY', 'http_proxy', 'https_proxy'
            ):
                env[name] = config.llm_proxy
        # Model traffic uses the proxy; local services must remain directly
        # reachable.
        bypass = env.get('NO_PROXY', env.get('no_proxy', ''))
        hosts = [
            '127.0.0.1', 'localhost', urlsplit(config.robot_url).hostname or ''
        ]
        env['NO_PROXY'] = env['no_proxy'] = ','.join(
            filter(None, [bypass, *hosts])
        )
        base_url = config.responses_url.removesuffix('/responses')
        # Start outside the source tree so project instructions and config
        # layers
        # cannot turn the robot-validation session into a repository coding
        # task.
        workspace = Path.home() / '.local/share/saydo-codex/workspace'
        workspace.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chdir(workspace)
        # The host-discovery flag does not suppress repository-local Skills.
        # Refresh explicit disables at every launch, including symlinked Skills.
        skill_roots = [
            agent_home / 'skills',
            Path.home() / '.agents/skills',
            Path('/etc/codex/skills'),
        ]
        for parent in (workspace, *workspace.parents):
            skill_roots.extend(
                [parent / '.agents/skills', parent / '.codex/skills']
            )
        args = [
            str(binary),
            '-c',
            'model_providers.trendengine.base_url=' + json.dumps(base_url),
            *sys.argv[1:],
        ]
        # Subcommand -c flags replace root -c flags in CLI 0.156.0. Apply the
        # Skills restriction in the final command scope, even with exec/debug
        # -c.
        insertion = args.index('--') if '--' in args else len(args)
        args[insertion:insertion] = [
            '-c',
            disabled_skills(
                skill_roots,
                (
                    row['path']
                    for row in agent_config.get('skills', {}).get('config', [])
                ),
            ),
        ]
        os.execve(binary, args, env)
    except (OSError, ValueError):
        # Never print parser errors that could contain credential file contents.
        print(
            'Codex 启动失败：请检查项目 .env（权限600）及官方 CLI 安装。',
            file=sys.stderr,
        )
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
