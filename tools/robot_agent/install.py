#!/usr/bin/env python3
"""Register the stdio tools in the isolated agent config; never starts robot motion."""
import argparse
import json
import os
from pathlib import Path
import secrets
import tomllib

from client import RobotBackend


BEGIN = '# BEGIN SAYDO ROBOT MCP'
END = '# END SAYDO ROBOT MCP'


def install(agent_home, python, base_url):
    # Preserve a virtualenv interpreter's path; resolving its symlink loses the venv.
    agent_home, python = Path(agent_home).resolve(), Path(python).absolute()
    config_path = agent_home/'config.toml'
    original = config_path.read_text()
    config = tomllib.loads(original)
    if 'saydo_robot' in config.get('mcp_servers', {}) and BEGIN not in original:
        raise ValueError('Existing saydo_robot configuration is not managed by this installer')
    if not python.is_file():raise ValueError('Install the isolated MCP Python environment first')
    token_path = agent_home/'credentials/robot-backend-token'
    token_path.parent.mkdir(parents=True, mode=0o700, exist_ok=True)
    if not token_path.exists():
        fd=os.open(token_path, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
        with os.fdopen(fd,'w') as f:f.write(secrets.token_urlsafe(36)+'\n')
    if token_path.stat().st_mode & 0o077:raise ValueError('Backend token must have owner-only permissions')
    RobotBackend(base_url, token_path)
    server=Path(__file__).with_name('server.py').resolve()
    args=[str(server),'--base-url',base_url,'--token-file',str(token_path)]
    block=f'''{BEGIN}
[mcp_servers.saydo_robot]
command = {json.dumps(str(python))}
args = {json.dumps(args)}
startup_timeout_sec = 20
tool_timeout_sec = 25
enabled = true
default_tools_approval_mode = "writes"

[mcp_servers.saydo_robot.tools.stop_robot]
approval_mode = "approve"
{END}
'''
    if BEGIN in original:
        start=original.index(BEGIN);end=original.index(END,start)+len(END)
        updated=original[:start]+block.rstrip()+original[end:]
    else:updated=original.rstrip()+'\n\n'+block
    parsed=tomllib.loads(updated)
    assert parsed['model']==config['model'] and parsed.get('model_reasoning_effort')==config.get('model_reasoning_effort')
    temporary=config_path.with_name('.config.robot-mcp.tmp')
    fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    try:
        with os.fdopen(fd,'w') as f:f.write(updated);f.flush();os.fsync(f.fileno())
        if config_path.read_text()!=original:raise ValueError('Agent config changed concurrently')
        temporary.replace(config_path)
    finally:temporary.unlink(missing_ok=True)
    return {'server':'saydo_robot','token_file':str(token_path),'python':str(python),'model':parsed['model'],
            'motion_sent':False,'backend_url':base_url}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--agent-home',type=Path,default=Path.home()/'.local/share/saydo-agent/codex-home')
    parser.add_argument('--python',type=Path,default=Path.home()/'.local/share/saydo-agent/mcp-venv/bin/python')
    parser.add_argument('--base-url',default='http://127.0.0.1:8089')
    args=parser.parse_args();print(json.dumps(install(args.agent_home,args.python,args.base_url),ensure_ascii=False))


if __name__=='__main__':main()
