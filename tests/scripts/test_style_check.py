"""style_check 的单元测试：每条规则各有合规与不合规的样例。"""

from __future__ import annotations

import contextlib
import io
import logging
import pathlib
import subprocess
import tempfile
import textwrap
import unittest
from typing import List

from scripts import style_check


def rules(findings: List[style_check.Finding]) -> List[str]:
    return [finding.rule for finding in findings]


class RepositoryTestCase(unittest.TestCase):
    """在临时目录里建一个真实的 git 仓库，不触碰本项目的文件。"""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = pathlib.Path(directory.name).resolve()
        # 被测脚本的日志写到标准错误，测试时关掉以免淹没结果。
        logging.disable(logging.CRITICAL)
        self.addCleanup(logging.disable, logging.NOTSET)
        self.git('init', '--quiet')

    def git(self, *arguments: str) -> None:
        identity = ['-c', 'user.name=test', '-c', 'user.email=test@example.com']
        subprocess.run(
            ['git', *identity, *arguments], cwd=self.root, check=True,
            capture_output=True,
        )

    def write(self, path: str, text: str) -> None:
        target = self.root / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(textwrap.dedent(text), encoding='utf-8')

    def commit(self) -> None:
        self.git('add', '--all')
        self.git('commit', '--quiet', '--message', 'snapshot')

    def check(self, path: str, text: str) -> List[style_check.Finding]:
        self.write(path, text)
        return style_check.check_file(self.root, path)

    def run_main(self, *arguments: str) -> tuple:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            code = style_check.main(list(arguments), root=self.root)
        return code, output.getvalue()


class TextRuleTest(unittest.TestCase):

    def test_wide_characters_count_as_two_columns(self):
        self.assertEqual(style_check.display_width('ab'), 2)
        self.assertEqual(style_check.display_width('试管'), 4)

    def test_line_wider_than_limit_is_reported_with_its_width(self):
        line = 'x = ' + ' + '.join(['1'] * 26)
        findings = style_check.text_findings('a.py', line)
        self.assertEqual(rules(findings), ['行宽'])
        self.assertIn(f'{len(line)} 列', findings[0].message)

    def test_forty_one_chinese_characters_exceed_eighty_columns(self):
        findings = style_check.text_findings('a.py', '# ' + '字' * 40)
        self.assertEqual(rules(findings), ['行宽'])

    def test_unbreakable_lines_are_exempt_from_width(self):
        long_name = 'm' * 90
        exempt = [
            f'# 见 https://example.com/{long_name}',
            f'from package import {long_name}',
            f"DIGEST = '{long_name}'",
        ]
        for line in exempt:
            with self.subTest(line=line[:20]):
                self.assertEqual(style_check.text_findings('a.py', line), [])

    def test_html_has_no_width_limit(self):
        line = '<p>' + 'x' * 100 + '</p>'
        self.assertEqual(style_check.text_findings('a.html', line), [])

    def test_tab_indentation_and_trailing_space_are_reported(self):
        findings = style_check.text_findings('a.sh', '\techo 1\necho 2 ')
        self.assertEqual(rules(findings), ['制表符', '行尾空白'])


class PythonRuleTest(RepositoryTestCase):

    def test_semicolon_in_code_is_reported_but_not_inside_string(self):
        findings = self.check('test_a.py', """\
            first = 1; second = 2
            text = 'a;b'
            """)
        self.assertEqual(rules(findings), ['分号'])
        self.assertEqual(findings[0].line, 1)

    def test_body_on_header_line_is_reported(self):
        findings = self.check('test_a.py', """\
            def area(side):
                if side < 0: return 0
                else: return side * side
            class Empty: pass
            """)
        self.assertEqual(rules(findings), ['同行语句'] * 3)
        self.assertEqual([finding.line for finding in findings], [2, 3, 4])

    def test_normal_layout_with_elif_and_decorator_is_accepted(self):
        findings = self.check('test_a.py', """\
            import functools


            @functools.lru_cache(maxsize=None)
            def sign(value):
                label = '正数'
                if value > 0:
                    return label
                elif value < 0:
                    return '负数'
                return '零'
            """)
        self.assertEqual(findings, [])

    def test_importing_a_class_from_stdlib_is_reported(self):
        findings = self.check('test_a.py', 'from pathlib import Path\n')
        self.assertEqual(rules(findings), ['导入符号'])
        self.assertIn('pathlib.Path', findings[0].message)

    def test_importing_modules_and_typing_names_is_accepted(self):
        findings = self.check('test_a.py', """\
            from __future__ import annotations

            import pathlib
            from os import path
            from typing import Optional
            from unittest import mock
            from urllib import request
            """)
        self.assertEqual(findings, [])

    def test_third_party_import_is_left_to_review(self):
        findings = self.check(
            'test_a.py', 'from package_not_installed import name\n'
        )
        self.assertEqual(findings, [])

    def test_relative_import_is_reported(self):
        self.write('package/config.py', '"""配置。"""\n')
        findings = self.check('package/test_a.py', 'from . import config\n')
        self.assertEqual(rules(findings), ['相对导入'])

    def test_internal_module_import_by_full_name_is_accepted(self):
        self.write('package/geometry.py', '"""几何。"""\n')
        findings = self.check(
            'package/test_a.py', 'from package import geometry\n'
        )
        self.assertEqual(findings, [])

    def test_internal_symbol_import_is_reported(self):
        self.write('package/geometry.py', '"""几何。"""\n')
        findings = self.check(
            'package/test_a.py', 'from package.geometry import transform\n'
        )
        self.assertEqual(rules(findings), ['导入符号'])

    def test_sibling_import_without_package_name_is_reported(self):
        self.write('package/geometry.py', '"""几何。"""\n')
        findings = self.check('package/test_a.py', """\
            import geometry
            from geometry import transform
            """)
        self.assertEqual(rules(findings), ['完整包名', '完整包名', '导入符号'])

    def test_sys_path_mutation_is_reported(self):
        findings = self.check('test_a.py', """\
            import sys

            sys.path.insert(0, '.')
            """)
        self.assertEqual(rules(findings), ['sys.path'])

    def test_public_function_needs_docstring_and_annotations(self):
        findings = self.check('module.py', """\
            \"\"\"示例模块。\"\"\"


            def lift(height_m, *, speed_percent: int):
                return height_m
            """)
        self.assertEqual(rules(findings), ['文档字符串', '类型标注'])
        self.assertIn('height_m、返回值', findings[1].message)

    def test_documented_and_annotated_module_is_accepted(self):
        findings = self.check('module.py', """\
            \"\"\"示例模块。\"\"\"


            class Gripper:
                \"\"\"夹爪。\"\"\"

                def __init__(self, force: int):
                    self.force = force

                def open(self) -> None:
                    \"\"\"会发送夹爪指令：张开夹爪。\"\"\"

                @staticmethod
                def limit(force: int) -> int:
                    \"\"\"返回限幅后的力度。\"\"\"
                    return min(force, 100)

                def do_GET(self) -> None:
                    return None

                def _helper(self, value):
                    return value
            """)
        self.assertEqual(findings, [])

    def test_constructor_parameters_need_annotations(self):
        findings = self.check('module.py', """\
            \"\"\"示例模块。\"\"\"


            class Gripper:
                \"\"\"夹爪。\"\"\"

                def __init__(self, force):
                    self.force = force
            """)
        self.assertEqual(rules(findings), ['类型标注'])

    def test_missing_module_docstring_is_reported(self):
        findings = self.check('module.py', 'LIMIT_M = 0.2\n')
        self.assertEqual(rules(findings), ['模块文档'])

    def test_test_files_are_exempt_from_docstrings_and_annotations(self):
        findings = self.check('test_module.py', """\
            def test_lift_reaches_height(case):
                return case
            """)
        self.assertEqual(findings, [])

    def test_naming_default_and_bare_except_are_reported(self):
        findings = self.check('test_a.py', """\
            class lower_case:
                pass


            def MixedCase(items=[]):
                try:
                    return items
                except:
                    return None
            """)
        self.assertEqual(
            rules(findings), ['命名', '可变默认参数', '命名', '裸except']
        )

    def test_library_hook_names_and_transform_names_are_accepted(self):
        findings = self.check('test_a.py', """\
            def base_T_object():
                return None


            def asyncSetUp():
                return None
            """)
        self.assertEqual(findings, [])

    def test_long_function_is_a_hint_not_an_error(self):
        body = '\n'.join(f'    value_{index} = {index}' for index in range(41))
        findings = self.check('test_a.py', f'def build():\n{body}\n')
        self.assertEqual(rules(findings), ['函数规模'])
        self.assertFalse(findings[0].is_error)

    def test_unparsable_file_is_reported_instead_of_crashing(self):
        findings = self.check('test_a.py', 'def broken(:\n')
        self.assertEqual(rules(findings), ['语法'])

    def test_lower_layer_importing_higher_layer_is_reported(self):
        self.write('saydo/manipulation/grasp_control.py', '"""抓取。"""\n')
        findings = self.check(
            'saydo/geometry/transforms.py',
            '"""变换。"""\n\nfrom saydo.manipulation import grasp_control\n',
        )
        self.assertEqual(rules(findings), ['分层'])
        self.assertIn('geometry 不能导入 manipulation', findings[0].message)

    def test_higher_layer_importing_lower_layers_is_accepted(self):
        self.write('saydo/geometry/transforms.py', '"""变换。"""\n')
        self.write('saydo/contracts/motion_contract.py', '"""约定。"""\n')
        self.write('saydo/paths.py', '"""路径。"""\n')
        findings = self.check('saydo/manipulation/grasp_control.py', """\
            \"\"\"抓取。\"\"\"

            from saydo import paths
            from saydo.contracts import motion_contract
            from saydo.geometry import transforms
            """)
        self.assertEqual(findings, [])

    def test_side_package_may_only_import_its_listed_packages(self):
        self.write('saydo/workbench/live_tracker.py', '"""服务。"""\n')
        findings = self.check(
            'saydo/agent/server.py',
            '"""接口。"""\n\nfrom saydo.workbench import live_tracker\n',
        )
        self.assertEqual(rules(findings), ['分层'])

    def test_unregistered_package_is_reported(self):
        findings = self.check('saydo/newpackage/module.py', '"""新包。"""\n')
        self.assertEqual(rules(findings), ['分层'])
        self.assertIn('未在分层表里登记', findings[0].message)

    def test_javascript_var_is_reported(self):
        findings = self.check('panel.js', """\
            var count = 0;
            const total = 1;
            """)
        self.assertEqual(rules(findings), ['var'])


class RepositoryModeTest(RepositoryTestCase):

    def test_vendored_and_non_source_files_are_not_listed(self):
        self.write('tools/a.py', '"""模块。"""\n')
        self.write('third_party/vendor/b.py', 'a=1;b=2\n')
        self.write('notes.md', 'x' * 200 + '\n')
        self.assertEqual(
            style_check.source_files(self.root, []), ['tools/a.py']
        )

    def test_changed_mode_reports_only_new_violations(self):
        self.write('test_old.py', 'first = 1; second = 2\nthird = 3\n')
        self.commit()
        self.write(
            'test_old.py', 'first = 1; second = 2\nthird = 3; fourth = 4\n'
        )
        code, output = self.run_main('--changed')
        self.assertEqual(code, 1)
        self.assertEqual(
            output.strip(),
            'test_old.py:2: 错误 [分号] 不用分号，每条语句独占一行',
        )

    def test_changed_mode_checks_whole_new_file(self):
        self.write('test_old.py', 'value = 1\n')
        self.commit()
        self.write('test_new.py', 'first = 1; second = 2\n')
        code, output = self.run_main('--changed')
        self.assertEqual(code, 1)
        self.assertIn('test_new.py:1', output)

    def test_changed_mode_reports_function_touched_anywhere(self):
        self.write('module.py', """\
            \"\"\"示例模块。\"\"\"


            def lift(height_m):
                return height_m
            """)
        self.commit()
        self.write('module.py', """\
            \"\"\"示例模块。\"\"\"


            def lift(height_m):
                return height_m * 2
            """)
        _, output = self.run_main('--changed')
        self.assertIn('[文档字符串]', output)
        self.assertIn('[类型标注]', output)

    def test_clean_repository_exits_with_zero(self):
        self.write('tools/a.py', '"""模块。"""\n')
        self.commit()
        self.assertEqual(self.run_main(), (0, ''))

    def test_hints_alone_do_not_fail_the_check(self):
        body = '\n'.join(f'    value_{index} = {index}' for index in range(41))
        self.write('test_a.py', f'def build():\n{body}\n')
        code, output = self.run_main()
        self.assertEqual(code, 0)
        self.assertIn('提示 [函数规模]', output)

    def test_git_failure_exits_with_two(self):
        self.write('test_a.py', 'value = 1\n')
        code, _ = self.run_main('--changed', 'no-such-commit')
        self.assertEqual(code, 2)

    def test_same_ast_accepts_pure_reformatting(self):
        self.write('test_a.py', "a=1;b={'k':[1,2]}\nif a:b=None  # 说明\n")
        self.commit()
        self.write('test_a.py', """\
            a = 1
            b = {
                'k': [1, 2],
            }
            if a:
                b = None
            """)
        self.assertEqual(self.run_main('--same-ast'), (0, ''))

    def test_same_ast_rejects_logic_change_and_new_file(self):
        self.write('test_a.py', 'limit = 1\n')
        self.commit()
        self.write('test_a.py', 'limit = 2\n')
        self.write('test_b.py', 'value = 1\n')
        code, output = self.run_main('--same-ast')
        self.assertEqual(code, 1)
        self.assertIn('test_a.py: 语法树不同', output)
        self.assertIn('test_b.py: 新文件', output)


if __name__ == '__main__':
    unittest.main()
