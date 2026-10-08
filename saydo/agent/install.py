#!/usr/bin/env python3
"""Register only the stdio tools in a Codex config; never starts robot motion."""

import argparse
import json
import os
import pathlib
import secrets
import tomllib

from saydo import paths
from saydo.agent import client


BEGIN = '# BEGIN SAYDO ROBOT MCP'
END = '# END SAYDO ROBOT MCP'


def install(agent_home, python, base_url, token_path=None):
    # Preserve a virtualenv interpreter's path; resolving its symlink loses the
    # venv.
    agent_home, python = (
        pathlib.Path(agent_home).resolve(), pathlib.Path(python).absolute()
    )
    config_path = agent_home / 'config.toml'
    original = config_path.read_text() if config_path.exists() else ''
    config = tomllib.loads(original)
    if 'saydo_robot' in config.get('mcp_servers', {}) and BEGIN not in original:
        raise ValueError(
            'Existing saydo_robot configuration is not managed by this '
            'installer'
        )
    if not python.is_file():
        raise ValueError('Install the isolated MCP Python environment first')
    token_path = (
        pathlib.Path(token_path).absolute()
        if token_path is not None
        else agent_home / 'credentials/robot-backend-token'
    )
    token_path.parent.mkdir(parents=True, mode=0o700, exist_ok=True)
    if not token_path.exists():
        fd = os.open(token_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as f:
            f.write(secrets.token_urlsafe(36) + '\n')
    if token_path.stat().st_mode & 0o077:
        raise ValueError('Backend token must have owner-only permissions')
    client.RobotBackend(base_url, token_path)
    # 以模块方式启动才能按完整包名导入；PYTHONPATH 指向项目根目录。
    args = [
        '-m',
        'saydo.agent.server',
        '--base-url',
        base_url,
        '--token-file',
        str(token_path),
    ]
    environment = json.dumps(str(paths.ROOT))
    block=f'''{BEGIN}
[mcp_servers.saydo_robot]
command = {json.dumps(str(python))}
args = {json.dumps(args)}
env = {{ PYTHONPATH = {environment} }}
startup_timeout_sec = 20
tool_timeout_sec = 25
enabled = true
default_tools_approval_mode = "writes"

[mcp_servers.saydo_robot.tools.stop_robot]
approval_mode = "approve"
{END}
'''
    if BEGIN in original:
        start = original.index(BEGIN)
        end = original.index(END, start) + len(END)
        updated = original[:start] + block.rstrip() + original[end:]
    else:
        updated = original.rstrip() + '\n\n' + block
    parsed = tomllib.loads(updated)
    assert {
        k: v for k, v in parsed.items() if k != 'mcp_servers'
    } == {k: v for k, v in config.items() if k != 'mcp_servers'}
    agent_home.mkdir(parents=True, mode=0o700, exist_ok=True)
    temporary = config_path.with_name('.config.robot-mcp.tmp')
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(updated)
            f.flush()
            os.fsync(f.fileno())
        current = config_path.read_text() if config_path.exists() else ''
        if current != original:
            raise ValueError('Agent config changed concurrently')
        temporary.replace(config_path)
    finally:
        temporary.unlink(missing_ok=True)
    return {
        'server': 'saydo_robot',
        'token_file': str(token_path),
        'python': str(python),
        'motion_sent': False,
        'backend_url': base_url,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--agent-home',
        type=pathlib.Path,
        default=pathlib.Path.home() / '.codex',
    )
    parser.add_argument(
        '--python',
        type=pathlib.Path,
        default=pathlib.Path.home() / '.local/share/saydo-mcp/venv/bin/python',
    )
    parser.add_argument(
        '--token-file',
        type=pathlib.Path,
        default=(
            pathlib.Path.home() / '.local/share/saydo-mcp/robot-backend-token'
        ),
    )
    parser.add_argument('--base-url', default='http://127.0.0.1:8089')
    args = parser.parse_args()
    print(
        json.dumps(
            install(
                args.agent_home, args.python, args.base_url, args.token_file
            ),
            ensure_ascii=False,
        )
    )


if __name__ == '__main__':
    main()
