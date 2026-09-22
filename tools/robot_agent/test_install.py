from pathlib import Path
import sys
import tempfile
import tomllib
import unittest
from install import install


class InstallTests(unittest.TestCase):
    def test_preserves_isolation_model_and_token_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);config=root/'config.toml'
            config.write_text('model="gpt-6-astra"\nmodel_reasoning_effort="high"\nproject_doc_max_bytes=0\n[features]\nplugins=false\n')
            interpreter=root/'venv/bin/python';interpreter.parent.mkdir(parents=True);interpreter.symlink_to(sys.executable)
            first=install(root,interpreter,'http://127.0.0.1:8089');before=config.read_bytes();token=Path(first['token_file']).read_bytes()
            second=install(root,interpreter,'http://127.0.0.1:8089')
            self.assertEqual(config.read_bytes(),before);self.assertEqual(Path(second['token_file']).read_bytes(),token)
            data=tomllib.loads(config.read_text())
            self.assertEqual(data['model'],'gpt-6-astra');self.assertEqual(data['project_doc_max_bytes'],0)
            self.assertFalse(data['features']['plugins'])
            self.assertEqual(data['mcp_servers']['saydo_robot']['command'],str(interpreter))
            self.assertNotIn(token.decode().strip(),config.read_text())

    def test_refuses_overwriting_unmanaged_server(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);config=root/'config.toml';config.write_text('model="gpt-6-astra"\n[mcp_servers.saydo_robot]\ncommand="existing"\n')
            before=config.read_bytes()
            with self.assertRaises(ValueError):install(root,sys.executable,'http://127.0.0.1:8089')
            self.assertEqual(config.read_bytes(),before)


if __name__=='__main__':unittest.main()
