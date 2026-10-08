from pathlib import Path
import tempfile
import tomllib
import unittest

from saydo.agent.codex_launcher import disabled_skills


class SkillIsolationTests(unittest.TestCase):
    def test_keep_disabled_bundled_skills_before_first_extraction(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / 'skills/.system/skill-creator'
            rows = tomllib.loads(
                disabled_skills([], [missing])
            )['skills']['config']
            self.assertEqual(
                rows, [{'path': str(missing / 'SKILL.md'), 'enabled': False}]
            )

    def test_new_skills_are_disabled_on_the_next_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(
                tomllib.loads(disabled_skills([root]))['skills']['config'], []
            )
            skill = root / 'new-coding-skill'
            skill.mkdir()
            (skill / 'SKILL.md').write_text('must never enter the task context')
            rows = tomllib.loads(disabled_skills([root]))['skills']['config']
            self.assertIn(
                {'path': str(skill.resolve() / 'SKILL.md'), 'enabled': False},
                rows,
            )

    def test_symlinked_skills_and_cycles(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source'
            source.mkdir()
            (source / 'SKILL.md').write_text('coding instructions')
            skills = root / 'skills'
            skills.mkdir()
            (skills / 'linked').symlink_to(source, target_is_directory=True)
            (source / 'cycle').symlink_to(skills, target_is_directory=True)
            rows = tomllib.loads(
                disabled_skills([skills, skills])
            )['skills']['config']
            self.assertEqual(
                {row['path'] for row in rows},
                {
                    str(source.resolve() / 'SKILL.md'),
                    str(skills / 'linked/SKILL.md'),
                },
            )
            self.assertTrue(all(row['enabled'] is False for row in rows))


if __name__ == '__main__':
    unittest.main()
