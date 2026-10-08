import pathlib
import tempfile
import unittest
from saydo.task_runner import store as task_runner_store
from saydo.task_runner import reconcile
from tests.task_runner import test_adapter


class ReconcileTests(unittest.TestCase):
    def test_only_quiescent_backend_can_clear_uncertainty_without_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            store = task_runner_store.Store(directory)
            run = store.create('操作', 'gpt-5.6-luna', 'execute')
            store.prepare(run['run_id'], 'c', 'grasp', {'object_id': 'b'}, 1)
            run.update(state='blocked', outcome_unknown=True)
            store.save(run)
            backend = test_adapter.Backend()
            backend.state['grasp']['stopping'] = True
            with self.assertRaises(ValueError):
                reconcile.reconcile(store, backend, run['run_id'])
            self.assertTrue(store.has_uncertain_runs())
            backend.state['grasp']['stopping'] = False
            result = reconcile.reconcile(store, backend, run['run_id'])
            self.assertFalse(store.has_uncertain_runs())
            self.assertFalse(result['resumed'])
            self.assertTrue(all(value is None for _, value in backend.requests))

    def test_missing_task_requires_separate_operator_acknowledgement(self):
        with tempfile.TemporaryDirectory() as directory:
            store = task_runner_store.Store(directory)
            run = store.create('操作', 'gpt-5.6-luna', 'execute')
            store.prepare(run['run_id'], 'c', 'grasp', {'object_id': 'b'}, 1)
            run.update(state='blocked', outcome_unknown=True)
            store.save(run)
            backend = test_adapter.Backend()
            backend.query_error = True
            with self.assertRaises(ValueError):
                reconcile.reconcile(store, backend, run['run_id'])
            self.assertTrue(store.has_uncertain_runs())
            reconcile.reconcile(
                store, backend, run['run_id'], confirmed_not_submitted=True
            )
            self.assertFalse(store.has_uncertain_runs())
