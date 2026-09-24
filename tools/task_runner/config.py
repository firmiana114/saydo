"""Machine-local configuration; dotenv values are data, never shell code."""
from dataclasses import dataclass, field
from pathlib import Path
import json
import os
import re
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[2]
UI_MODELS = ('gpt-5.5', 'gpt-5.6-luna', 'gpt-5.6-terra', 'gpt-5.6-sol', 'gpt-6-astra')
UI_REASONING_EFFORTS = ('low', 'medium', 'high')


def read_env(path):
    path = Path(path)
    if path.stat().st_mode & 0o077:
        raise ValueError('.env 必须仅当前用户可读写（chmod 600）')
    values = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        key, sep, value = line.partition('=')
        if not sep or not key.startswith('SAYDO_'):
            raise ValueError('.env 仅接受 SAYDO_ 开头的 KEY=value')
        value = value.strip()
        if value.startswith('"'):
            value = json.loads(value)
        elif value.startswith("'") and value.endswith("'"):
            value = value[1:-1]
        if not isinstance(value, str) or '\n' in value or '\r' in value:
            raise ValueError('无效的 .env 值')
        values[key] = value
    return values


@dataclass
class Config:
    api_key: str = field(repr=False)
    responses_url: str = 'https://trendengine.site/v1/responses'
    model: str = 'gpt-5.6-terra'
    fallback_model: str = ''
    reasoning_effort: str = 'medium'
    llm_proxy: str | None = None
    robot_url: str = 'http://127.0.0.1:8089'
    token_file: Path = field(default_factory=lambda: Path.home()/'.local/share/saydo-mcp/robot-backend-token')
    mode: str = 'execute'
    state_dir: Path = ROOT/'work/task_runner'
    max_turns: int = 20
    run_timeout: float = 600
    action_timeout: float = 120
    vision_timeout: float = 90
    llm_timeout: float = 90
    poll_interval: float = .5

    @classmethod
    def load(cls, path):
        values = read_env(path)
        def get(name, default):
            key = 'SAYDO_'+name
            return os.environ.get(key, values.get(key, default))
        config = cls(api_key=get('LLM_API_KEY', ''),
            responses_url=get('LLM_RESPONSES_URL', cls.responses_url),
            model=get('LLM_MODEL', cls.model), fallback_model=get('LLM_FALLBACK_MODEL', cls.fallback_model),
            reasoning_effort=get('LLM_REASONING_EFFORT', cls.reasoning_effort),
            llm_proxy=get('LLM_PROXY', '') or None,
            robot_url=get('ROBOT_URL', cls.robot_url),
            token_file=Path(get('ROBOT_TOKEN_FILE', str(Path.home()/'.local/share/saydo-mcp/robot-backend-token'))).expanduser(),
            mode=get('RUNNER_MODE', cls.mode), state_dir=Path(get('RUNNER_STATE_DIR', str(cls.state_dir))).expanduser())
        url = urlsplit(config.responses_url)
        if url.scheme != 'https' or not url.hostname or url.username or url.password or url.query or url.fragment or not url.path.endswith('/responses'):
            raise ValueError('模型地址必须是无嵌入凭据的 HTTPS Responses 完整地址')
        if not config.api_key or config.mode not in ('read_only', 'preview', 'execute'):
            raise ValueError('缺少模型密钥或执行模式无效')
        if config.llm_proxy:
            proxy = urlsplit(config.llm_proxy)
            if proxy.scheme not in ('http', 'https') or not proxy.hostname or proxy.username or proxy.password or proxy.query or proxy.fragment:
                raise ValueError('模型代理地址无效')
        if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,100}', config.model):
            raise ValueError('模型名称无效')
        if config.fallback_model and not re.fullmatch(r'[A-Za-z0-9_.:-]{1,100}', config.fallback_model):
            raise ValueError('回退模型名称无效')
        return config
