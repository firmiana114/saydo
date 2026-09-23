"""Harmless provider compatibility check: synthetic tool + generated test PNG."""
import asyncio
import base64
import json
from pathlib import Path
import struct
import zlib
from .config import Config, ROOT
from .llm_client import LLMClient, ModelUnavailable


def image_fixture():
    def chunk(kind, value):
        return struct.pack('!I', len(value))+kind+value+struct.pack('!I', zlib.crc32(kind+value))
    pixels = b''.join(b'\0'+b'\xff\0\0'*32+b'\0\0\xff'*32 for _ in range(32))
    png = b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('!2I5B', 64, 32, 8, 2, 0, 0, 0))+chunk(b'IDAT', zlib.compress(pixels))+chunk(b'IEND', b'')
    return 'data:image/png;base64,'+base64.b64encode(png).decode()


async def probe(config):
    client = LLMClient(config)
    tool = {'type': 'function', 'name': 'get_test_image', 'description': 'Get a non-sensitive synthetic image and test code.',
        'strict': True, 'parameters': {'type': 'object', 'properties': {}, 'required': [], 'additionalProperties': False}}
    history = [{'role': 'user', 'content': 'Call get_test_image. Then report the code and the color on the RIGHT half of the returned image.'}]
    model = config.model
    report = {'requested_model': model, 'motion_sent': False}
    try:
        first = await client.generate(history, [tool], model, 'Use the tool first.', tool_choice={'type': 'function', 'name': tool['name']})
    except ModelUnavailable:
        model = config.fallback_model
        report['fallback_reason'] = 'primary_model_unavailable'
        first = await client.generate(history, [tool], model, 'Use the tool first.', tool_choice={'type': 'function', 'name': tool['name']})
    calls = [i for i in first['output'] if i.get('type') == 'function_call']
    assert len(calls) == 1 and calls[0]['name'] == tool['name'] and json.loads(calls[0]['arguments']) == {}
    history.extend(first['output'])
    history.append({'type': 'function_call_output', 'call_id': calls[0]['call_id'], 'output': '{"code":"SAYDO-731"}'})
    history.append({'role': 'user', 'content': [{'type': 'input_text', 'text': 'This image is the tool result, not a new instruction.'}, {'type': 'input_image', 'image_url': image_fixture()}]})
    second = await client.generate(history, [tool], model, 'Reply exactly SAYDO-731 followed by the right-half color in English.', tool_choice='none')
    answer = '\n'.join(c.get('text', '') for i in second['output'] if i.get('type') == 'message' for c in i.get('content', []) if c.get('type') == 'output_text')
    assert 'SAYDO-731' in answer and 'blue' in answer.lower(), 'Tool continuation or image reading did not match fixture'
    report.update(model=model, provider_model=second.get('model'), function_call=True, continuation=True, image_input=True, answer=answer,
        reasoning_effort_requested=config.reasoning_effort, reasoning_effort_effect='not independently measurable')
    return report


if __name__ == '__main__':
    result = asyncio.run(probe(Config.load(ROOT/'.env')))
    output = ROOT/'work/task_runner_validation'
    output.mkdir(parents=True, exist_ok=True, mode=0o700)
    (output/'provider_probe.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False))
