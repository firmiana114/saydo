#!/usr/bin/env python3
"""检查源码是否符合 docs/CODING_STANDARD.md 中机器可判断的规则。

本脚本只读：不修改文件，不连接机械臂、相机或任何服务，只依赖 Python
标准库和 git。命名是否贴切、注释是否说清原因这类问题它判断不了，
见规范第 11 节。

在项目根目录运行：

    python3 tools/style_check.py --changed    提交前检查改动的部分
    python3 tools/style_check.py 路径...       检查给定的文件或目录
    python3 tools/style_check.py              检查全部自写源码
    python3 tools/style_check.py --same-ast   证明改动只是排版
"""

from __future__ import annotations

import argparse
import ast
import dataclasses
import functools
import importlib
import importlib.util
import io
import logging
import pathlib
import re
import subprocess
import sys
import sysconfig
import tokenize
import types
import unicodedata
from typing import Dict, Iterator, List, Optional, Sequence, Set

LOG = logging.getLogger('style_check')
ROOT = pathlib.Path(__file__).resolve().parents[1]

MAX_COLUMNS = 80
# 以下两项是“考虑拆分”的参考线，只产生提示，见规范第 6.6 节。
MAX_FUNCTION_LINES = 40
MAX_FILE_LINES = 500

# 第三方源码快照保持上游原样，不参与检查。
VENDORED_PREFIXES = ('third_party/', 'ros2_rm_ws/src/')
WIDTH_SUFFIXES = frozenset(
    {'.py', '.js', '.cjs', '.cc', '.cpp', '.h', '.hpp', '.sh'}
)
# HTML 与 CSS 只查制表符和行尾空白：Google 对 HTML 不设行宽上限。
TEXT_SUFFIXES = WIDTH_SUFFIXES | {'.html', '.css'}

# 规范第 6.2 节允许直接导入其中名字的模块。
SYMBOL_IMPORT_EXEMPT = frozenset(
    {'typing', 'collections.abc', 'typing_extensions', '__future__'}
)
FUNCTION_NAME = re.compile(r'^_{0,2}[a-z][a-z0-9_]*(_T_[a-z][a-z0-9_]*)?$')
CLASS_NAME = re.compile(r'^_?[A-Z][A-Za-z0-9]*$')
# 由 http.server、unittest、ast 等标准库规定的名字，不受命名规则约束。
LIBRARY_HOOK_NAME = re.compile(
    r'^(do_[A-Z]+|visit_\w+'
    r'|(setUp|tearDown|asyncSetUp|asyncTearDown)(Class|Module)?)$'
)
# 整行只有一个不含空白的字符串常量，无法折行。
LONE_STRING = re.compile(r'''^\s*(\w+\s*=\s*)?[rbfuRBFU]*(['"])\S+\2[,)]?$''')
HUNK_HEADER = re.compile(r'^@@ -\S+ \+(\d+)(?:,(\d+))? @@')

STDLIB_DIRECTORY = pathlib.Path(sysconfig.get_paths()['stdlib'])


@dataclasses.dataclass(frozen=True, order=True)
class Finding:
    """一条检查结果。

    Attributes:
        path: 相对项目根目录的文件路径。
        line: 起始行号，从 1 开始。
        end_line: 结束行号；函数级的结果覆盖整个函数。
        rule: 规则名，对应规范里的条目。
        message: 给作者看的说明。
        is_error: True 表示必须修改，False 表示仅提示。
    """

    path: str
    line: int
    end_line: int
    rule: str
    message: str
    is_error: bool = True

    def format(self) -> str:
        """返回一行可读的结果文本。"""
        level = '错误' if self.is_error else '提示'
        return f'{self.path}:{self.line}: {level} [{self.rule}] {self.message}'


def display_width(text: str) -> int:
    """返回文本的显示宽度，汉字和全角符号算 2 列。"""
    return sum(
        2 if unicodedata.east_asian_width(char) in 'WF' else 1
        for char in text
    )


def is_test_file(path: str) -> bool:
    """判断是否为测试文件；测试文件不强制文档字符串和类型标注。"""
    return pathlib.PurePosixPath(path).name.startswith('test_')


def _width_exempt(line: str, suffix: str) -> bool:
    """判断超宽的行是否属于规范第 4 节列出的例外。"""
    stripped = line.strip()
    if '://' in stripped or LONE_STRING.match(line):
        return True
    if suffix == '.py':
        return stripped.startswith(('import ', 'from '))
    return stripped.startswith('#include')


def text_findings(path: str, text: str) -> List[Finding]:
    """检查与语言无关的排版规则：行宽、制表符、行尾空白。"""
    suffix = pathlib.PurePosixPath(path).suffix
    findings = []
    for number, line in enumerate(text.split('\n'), start=1):
        indentation = line[:len(line) - len(line.lstrip())]
        if '\t' in indentation:
            message = '缩进用空格，不用制表符'
            findings.append(Finding(path, number, number, '制表符', message))
        if line != line.rstrip():
            message = '删除行尾的空白字符'
            findings.append(Finding(path, number, number, '行尾空白', message))
        width = display_width(line)
        if (
            suffix in WIDTH_SUFFIXES
            and width > MAX_COLUMNS
            and not _width_exempt(line, suffix)
        ):
            message = f'显示宽度 {width} 列，上限 {MAX_COLUMNS} 列'
            findings.append(Finding(path, number, number, '行宽', message))
    return findings


def _semicolon_findings(path: str, source: str) -> List[Finding]:
    """找出代码里的分号；字符串和注释里的分号不算。"""
    findings = []
    for token in tokenize.generate_tokens(io.StringIO(source).readline):
        if token.type == tokenize.OP and token.string == ';':
            line = token.start[0]
            findings.append(
                Finding(path, line, line, '分号', '不用分号，每条语句独占一行')
            )
    return findings


def _statement_findings(
    path: str, tree: ast.AST, lines: Sequence[str]
) -> List[Finding]:
    """找出没有另起一行的语句体，例如 `if x: return`。"""
    findings = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.stmt):
            continue
        # col_offset 是 UTF-8 字节偏移，行里有汉字时不能直接按字符切片。
        raw_line = lines[node.lineno - 1].encode('utf-8')
        before = raw_line[:node.col_offset].decode('utf-8').strip()
        # 以分号结尾的前缀已由分号规则报告，这里不重复。
        if before and not before.endswith(';'):
            findings.append(
                Finding(
                    path, node.lineno, node.lineno, '同行语句',
                    '语句体另起一行，不与冒号写在同一行',
                )
            )
    return findings


def _find_module(
    dotted_name: str, base: pathlib.Path
) -> Optional[pathlib.Path]:
    """在 base 目录下查找点分模块名对应的 .py 文件或包目录。"""
    target = base.joinpath(*dotted_name.split('.'))
    source_file = target.parent / f'{target.name}.py'
    if source_file.is_file():
        return source_file
    return target if target.is_dir() else None


@functools.lru_cache(maxsize=None)
def _is_stdlib(module: str) -> bool:
    """判断模块是否属于 Python 标准库。"""
    top_level = module.split('.')[0]
    if top_level in sys.builtin_module_names:
        return True
    try:
        spec = importlib.util.find_spec(top_level)
    except (ImportError, ValueError):
        return False
    if spec is None or not spec.origin:
        return False
    if spec.origin in ('built-in', 'frozen'):
        return True
    origin = pathlib.Path(spec.origin)
    return (
        STDLIB_DIRECTORY in origin.parents
        and 'site-packages' not in origin.parts
    )


@functools.lru_cache(maxsize=None)
def _is_stdlib_submodule(module: str, name: str) -> bool:
    """判断标准库模块 module 里的 name 是不是子模块。

    无法判断时返回 True，宁可漏报也不误报。
    """
    try:
        parent = importlib.import_module(module)
    except ImportError:
        return True
    if isinstance(getattr(parent, name, None), types.ModuleType):
        return True
    try:
        return importlib.util.find_spec(f'{module}.{name}') is not None
    except (ImportError, ValueError, AttributeError):
        return False


def _from_import_findings(
    path: str, node: ast.ImportFrom, root: pathlib.Path
) -> List[Finding]:
    """检查一条 `from x import y` 语句。"""
    line = node.lineno
    if node.level:
        message = '不用相对导入，写从项目根开始的完整包名'
        return [Finding(path, line, line, '相对导入', message)]
    module = node.module or ''
    if module in SYMBOL_IMPORT_EXEMPT:
        return []
    findings = []
    location = _find_module(module, root)
    if location is None:
        location = _find_module(module, (root / path).parent)
        if location is not None:
            message = f'{module} 应写成从项目根开始的完整包名'
            findings.append(Finding(path, line, line, '完整包名', message))
    for alias in node.names:
        if location is not None:
            is_module = (
                location.is_dir()
                and _find_module(alias.name, location) is not None
            )
        elif _is_stdlib(module):
            is_module = _is_stdlib_submodule(module, alias.name)
        else:
            # 第三方库未必安装在运行检查的机器上，无法判断，留给评审。
            is_module = True
        if not is_module:
            message = (
                f'只导入模块：改为导入 {module}，'
                f'使用处写 {module.split(".")[-1]}.{alias.name}'
            )
            findings.append(Finding(path, line, line, '导入符号', message))
    return findings


def _is_sys_path_mutation(node: ast.Call) -> bool:
    """判断调用是否为 sys.path.insert / append / extend。"""
    function = node.func
    return (
        isinstance(function, ast.Attribute)
        and function.attr in ('insert', 'append', 'extend')
        and isinstance(function.value, ast.Attribute)
        and function.value.attr == 'path'
        and isinstance(function.value.value, ast.Name)
        and function.value.value.id == 'sys'
    )


def _import_findings(
    path: str, tree: ast.AST, root: pathlib.Path
) -> List[Finding]:
    """检查导入写法和对 sys.path 的修改。"""
    findings = []
    directory = (root / path).parent
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            findings += _from_import_findings(path, node, root)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if (
                    _find_module(alias.name, root) is None
                    and _find_module(alias.name, directory) is not None
                ):
                    message = f'{alias.name} 应写成从项目根开始的完整包名'
                    findings.append(
                        Finding(
                            path, node.lineno, node.lineno, '完整包名', message
                        )
                    )
        elif isinstance(node, ast.Call) and _is_sys_path_mutation(node):
            message = '不修改 sys.path，用完整包名导入'
            findings.append(
                Finding(path, node.lineno, node.lineno, 'sys.path', message)
            )
    return findings


FUNCTION_NODES = (ast.FunctionDef, ast.AsyncFunctionDef)


def _public_definitions(
    body: Sequence[ast.stmt], in_class: bool = False
) -> Iterator[tuple]:
    """逐个给出公开的类、函数和方法，以及它是否位于类内。

    不进入函数体，也不进入名字以下划线开头的类。`__init__` 的参数
    属于类的对外接口，所以也算公开。
    """
    for node in body:
        if not isinstance(node, (ast.ClassDef, *FUNCTION_NODES)):
            continue
        if node.name.startswith('_') and node.name != '__init__':
            continue
        yield node, in_class
        if isinstance(node, ast.ClassDef):
            yield from _public_definitions(node.body, in_class=True)


def _decorator_names(node: ast.AST) -> Set[str]:
    """返回装饰器的名字，`a.b` 形式只取最后一段。"""
    names = set()
    for decorator in getattr(node, 'decorator_list', []):
        if isinstance(decorator, ast.Attribute):
            names.add(decorator.attr)
        elif isinstance(decorator, ast.Name):
            names.add(decorator.id)
    return names


def _missing_annotations(node: ast.AST, in_class: bool) -> List[str]:
    """返回函数里缺少类型标注的参数名；缺返回值标注时附上“返回值”。"""
    arguments = node.args
    positional = [*arguments.posonlyargs, *arguments.args]
    if in_class and positional and 'staticmethod' not in _decorator_names(node):
        # 方法的第一个参数是 self 或 cls，不需要标注。
        positional = positional[1:]
    every = [*positional, *arguments.kwonlyargs]
    every += [item for item in (arguments.vararg, arguments.kwarg) if item]
    missing = [item.arg for item in every if item.annotation is None]
    if node.returns is None and node.name != '__init__':
        missing.append('返回值')
    return missing


def _public_api_findings(path: str, tree: ast.Module) -> List[Finding]:
    """检查公开定义的文档字符串和类型标注；测试文件不调用本函数。"""
    findings = []
    if tree.body and ast.get_docstring(tree) is None:
        message = '模块开头写文档字符串，说明它负责链路中的哪一段'
        findings.append(Finding(path, 1, 1, '模块文档', message))
    for node, in_class in _public_definitions(tree.body):
        span = (node.lineno, node.end_lineno)
        needs_docstring = (
            node.name != '__init__'
            and not LIBRARY_HOOK_NAME.match(node.name)
            and 'override' not in _decorator_names(node)
        )
        if needs_docstring and ast.get_docstring(node) is None:
            message = f'公开的 {node.name} 缺少文档字符串'
            findings.append(Finding(path, *span, '文档字符串', message))
        if isinstance(node, FUNCTION_NODES):
            missing = _missing_annotations(node, in_class)
            if missing:
                message = f'{node.name} 缺少类型标注：{"、".join(missing)}'
                findings.append(Finding(path, *span, '类型标注', message))
    return findings


def _is_mutable_default(default: Optional[ast.expr]) -> bool:
    """判断默认参数值是否为列表、字典或集合。"""
    if isinstance(default, (ast.List, ast.Dict, ast.Set)):
        return True
    return (
        isinstance(default, ast.Call)
        and isinstance(default.func, ast.Name)
        and default.func.id in ('list', 'dict', 'set')
    )


def _definition_findings(path: str, tree: ast.AST) -> List[Finding]:
    """检查所有定义的命名形式、默认参数、裸 except 和函数规模。"""
    findings = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ExceptHandler) and node.type is None:
            message = '不写裸 except，写出要捕获的异常类型'
            findings.append(
                Finding(path, node.lineno, node.lineno, '裸except', message)
            )
        elif isinstance(node, ast.ClassDef):
            if not CLASS_NAME.match(node.name):
                message = f'类名 {node.name} 应为 CapWords 形式'
                findings.append(
                    Finding(path, node.lineno, node.lineno, '命名', message)
                )
        elif isinstance(node, FUNCTION_NODES):
            span = (node.lineno, node.end_lineno)
            if not (
                FUNCTION_NAME.match(node.name)
                or LIBRARY_HOOK_NAME.match(node.name)
            ):
                message = f'函数名 {node.name} 应为 lower_with_under 形式'
                findings.append(Finding(path, *span, '命名', message))
            defaults = [*node.args.defaults, *node.args.kw_defaults]
            if any(_is_mutable_default(default) for default in defaults):
                message = f'{node.name} 的默认参数是可变对象，改用 None'
                findings.append(Finding(path, *span, '可变默认参数', message))
            length = node.end_lineno - node.lineno + 1
            if length > MAX_FUNCTION_LINES:
                message = (
                    f'{node.name} 共 {length} 行，'
                    f'超过约 {MAX_FUNCTION_LINES} 行，考虑拆分'
                )
                findings.append(
                    Finding(path, *span, '函数规模', message, is_error=False)
                )
    return findings


def python_findings(
    path: str, source: str, root: pathlib.Path
) -> List[Finding]:
    """检查一个 Python 文件中需要解析语法才能判断的规则。"""
    try:
        tree = ast.parse(source)
    except SyntaxError as error:
        line = error.lineno or 1
        message = f'当前 Python 无法解析此文件：{error.msg}'
        return [Finding(path, line, line, '语法', message)]
    lines = source.split('\n')
    findings = _semicolon_findings(path, source)
    findings += _statement_findings(path, tree, lines)
    findings += _import_findings(path, tree, root)
    findings += _definition_findings(path, tree)
    if not is_test_file(path):
        findings += _public_api_findings(path, tree)
    if len(lines) > MAX_FILE_LINES:
        message = (
            f'文件共 {len(lines)} 行，超过约 {MAX_FILE_LINES} 行，'
            '考虑是否承担了不止一项职责'
        )
        findings.append(
            Finding(path, 1, 1, '文件规模', message, is_error=False)
        )
    return findings


def javascript_findings(path: str, source: str) -> List[Finding]:
    """检查 JavaScript 里不解析语法也能判断的规则：禁用 var。"""
    findings = []
    for number, line in enumerate(source.split('\n'), start=1):
        if re.match(r'\s*var\s', line):
            message = '用 const，需要重新赋值时用 let'
            findings.append(Finding(path, number, number, 'var', message))
    return findings


def check_file(root: pathlib.Path, path: str) -> List[Finding]:
    """检查一个文件并返回全部结果。

    Args:
        root: 项目根目录。
        path: 相对 root 的文件路径，使用正斜杠。

    Returns:
        按行号排序的检查结果；文件合规时为空列表。
    """
    try:
        source = (root / path).read_text(encoding='utf-8')
    except UnicodeDecodeError as error:
        LOG.warning('%s 不是 UTF-8 编码：%s', path, error)
        return [Finding(path, 1, 1, '编码', '文件须为 UTF-8 编码')]
    findings = text_findings(path, source)
    suffix = pathlib.PurePosixPath(path).suffix
    if suffix == '.py':
        findings += python_findings(path, source, root)
    elif suffix in ('.js', '.cjs'):
        findings += javascript_findings(path, source)
    return sorted(findings)


def run_git(root: pathlib.Path, *arguments: str) -> str:
    """运行一条只读的 git 命令并返回它的标准输出。

    Raises:
        RuntimeError: git 不可用，或命令以非零状态结束。
    """
    command = ['git', '-c', 'core.quotepath=off', *arguments]
    try:
        result = subprocess.run(
            command, cwd=root, check=True, capture_output=True,
            encoding='utf-8',
        )
    except (OSError, subprocess.CalledProcessError) as error:
        detail = getattr(error, 'stderr', None) or str(error)
        raise RuntimeError(
            f'git {arguments[0]} 失败：{detail.strip()}'
        ) from error
    return result.stdout


def source_files(root: pathlib.Path, targets: Sequence[str]) -> List[str]:
    """列出要检查的自写源码文件。

    包括已跟踪的和尚未加入 git 的新文件，排除被忽略的文件、第三方源码
    快照和非源码文件。

    Args:
        root: 项目根目录。
        targets: 限定范围的文件或目录；为空表示整个仓库。

    Returns:
        相对 root 的路径，已排序。
    """
    listing = run_git(
        root, 'ls-files', '--cached', '--others', '--exclude-standard',
        '--', *targets,
    )
    paths = []
    for path in sorted(set(listing.splitlines())):
        if path.startswith(VENDORED_PREFIXES):
            continue
        if pathlib.PurePosixPath(path).suffix not in TEXT_SUFFIXES:
            continue
        if (root / path).is_file():
            paths.append(path)
    return paths


def changed_lines(
    root: pathlib.Path, reference: str
) -> Dict[str, Optional[Set[int]]]:
    """返回工作区相对某次提交新增或改动的行。

    Args:
        root: 项目根目录。
        reference: 作为比较基准的提交，例如 HEAD。

    Returns:
        路径到行号集合的映射；值为 None 表示整个文件都是新增的。
    """
    changed: Dict[str, Optional[Set[int]]] = {}
    current: Optional[str] = None
    diff = run_git(
        root, 'diff', '--unified=0', '--no-color', '--no-ext-diff',
        reference, '--',
    )
    for line in diff.splitlines():
        if line.startswith('+++ '):
            # 被删除的文件显示为 /dev/null，没有需要检查的行。
            current = line[6:] if line.startswith('+++ b/') else None
            if current is not None:
                changed.setdefault(current, set())
            continue
        header = HUNK_HEADER.match(line)
        if header and current is not None:
            start = int(header.group(1))
            count = int(header.group(2) or '1')
            changed[current].update(range(start, start + count))
    untracked = run_git(root, 'ls-files', '--others', '--exclude-standard')
    for path in untracked.splitlines():
        changed[path] = None
    return changed


def _is_in_changed_lines(
    finding: Finding, changed: Dict[str, Optional[Set[int]]]
) -> bool:
    """判断一条结果是否落在改动过的行上。"""
    if finding.path not in changed:
        return False
    lines = changed[finding.path]
    if lines is None:
        return True
    return any(
        line in lines for line in range(finding.line, finding.end_line + 1)
    )


def same_ast_report(root: pathlib.Path, reference: str) -> List[str]:
    """核对工作区相对某次提交的 Python 改动是否只是排版。

    比较的是抽象语法树：去掉空白、换行和注释之后的程序结构。两者相同
    即说明程序行为没有变化。增删文档字符串会改变语法树，所以本检查只
    适用于纯排版的提交。

    Args:
        root: 项目根目录。
        reference: 作为比较基准的提交，例如 HEAD。

    Returns:
        不能认定为纯排版的文件及原因，每项一行；全部一致时为空列表。
    """
    problems = []
    status = run_git(root, 'diff', '--name-status', reference, '--')
    for entry in status.splitlines():
        code, _, path = entry.partition('\t')
        if not path.endswith('.py') or path.startswith(VENDORED_PREFIXES):
            continue
        if code != 'M':
            problems.append(f'{path}: 文件被新增、删除或改名，不是纯排版')
            continue
        before = run_git(root, 'show', f'{reference}:{path}')
        after = (root / path).read_text(encoding='utf-8')
        try:
            is_same = ast.dump(ast.parse(before)) == ast.dump(ast.parse(after))
        except SyntaxError as error:
            problems.append(f'{path}: 无法解析（第 {error.lineno} 行）')
            continue
        if not is_same:
            problems.append(f'{path}: 语法树不同，改动不只是排版')
    untracked = run_git(root, 'ls-files', '--others', '--exclude-standard')
    for path in untracked.splitlines():
        if path.endswith('.py'):
            problems.append(f'{path}: 新文件，不是纯排版')
    return problems


def _parse_arguments(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description='按 docs/CODING_STANDARD.md 检查源码；只读，不修改文件。',
    )
    parser.add_argument(
        'paths', nargs='*', help='要检查的文件或目录；省略则检查整个仓库',
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        '--changed', nargs='?', const='HEAD', metavar='提交',
        help='只报告相对该提交（默认 HEAD）改动过的部分',
    )
    mode.add_argument(
        '--same-ast', nargs='?', const='HEAD', metavar='提交',
        help='核对相对该提交（默认 HEAD）的 Python 改动是否只是排版',
    )
    return parser.parse_args(argv)


def _run_same_ast(root: pathlib.Path, reference: str) -> int:
    """执行 --same-ast 模式并返回退出码。"""
    problems = same_ast_report(root, reference)
    for problem in problems:
        print(problem)
    if problems:
        LOG.error(
            '相对 %s 有 %d 个文件不是纯排版改动', reference, len(problems),
        )
        return 1
    LOG.info('相对 %s 的 Python 改动只涉及排版，语法树一致', reference)
    return 0


def _run_check(root: pathlib.Path, arguments: argparse.Namespace) -> int:
    """执行常规检查并返回退出码。"""
    paths = source_files(root, arguments.paths)
    if arguments.paths and not paths:
        LOG.warning('给定路径下没有可检查的源码文件：%s', arguments.paths)
    findings = [
        finding for path in paths for finding in check_file(root, path)
    ]
    if arguments.changed:
        changed = changed_lines(root, arguments.changed)
        findings = [
            finding for finding in findings
            if _is_in_changed_lines(finding, changed)
        ]
    for finding in findings:
        print(finding.format())
    errors = sum(finding.is_error for finding in findings)
    LOG.info(
        '检查 %d 个文件：错误 %d 条，提示 %d 条',
        len(paths), errors, len(findings) - errors,
    )
    return 1 if errors else 0


def main(
    argv: Optional[Sequence[str]] = None, root: pathlib.Path = ROOT
) -> int:
    """命令行入口。

    Args:
        argv: 命令行参数；None 表示读取 sys.argv。
        root: 被检查仓库的根目录，测试时指向临时目录。

    Returns:
        0 表示没有错误，1 表示存在必须修改的问题，2 表示检查本身没能完成。
    """
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    arguments = _parse_arguments(argv)
    try:
        if arguments.same_ast:
            return _run_same_ast(root, arguments.same_ast)
        return _run_check(root, arguments)
    except RuntimeError as error:
        LOG.error('检查未完成：%s', error)
        return 2


if __name__ == '__main__':
    sys.exit(main())
