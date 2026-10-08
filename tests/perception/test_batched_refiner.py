import sys
import types
import unittest
from unittest import mock
import numpy as np
from saydo.perception import batched_refiner


class BatchingTests(unittest.TestCase):
    def test_depth_transform_batches_and_merges_all_fields(self):
        try:
            import torch
        except ImportError:
            self.skipTest('torch required')

        class Batch:
            def __init__(self, pose=None, rgb=None):
                self.poseA = pose
                self.rgbAs = rgb
                self.unused = None

            def select_by_indices(self, ids):
                return Batch(self.poseA[ids], self.rgbAs[ids])

        sizes = []

        def transform(batch, **kwargs):
            sizes.append(len(batch.poseA))
            batch.rgbAs = batch.rgbAs * 2
            return batch

        scorer = types.SimpleNamespace(
            model=types.SimpleNamespace(extract_feat=lambda a, b: a + b),
            dataset=types.SimpleNamespace(transform_batch=transform),
        )
        batched_refiner.batch_score_features(scorer, 32)
        poses = torch.arange(77).reshape(-1, 1)
        rgb = poses.float() + 1
        result = scorer.dataset.transform_batch(
            Batch(poses, rgb), H_ori=360, W_ori=640, bound=1
        )
        self.assertEqual(sizes, [32, 32, 13])
        self.assertTrue(torch.equal(result.poseA, poses))
        self.assertTrue(torch.equal(result.rgbAs, rgb * 2))
        self.assertIsNone(result.unused)

    def test_refiner_keeps_every_candidate_in_order(self):
        backend = mock.Mock()
        backend.predict.side_effect = lambda **kw: (kw['ob_in_cams'] + 1, None)
        torch = types.SimpleNamespace(
            cat=lambda xs, dim: np.concatenate(xs, axis=dim),
            cuda=types.SimpleNamespace(empty_cache=lambda: None),
        )
        poses = np.arange(77).reshape(-1, 1)
        with mock.patch.dict(sys.modules, {'torch': torch}):
            result, _ = batched_refiner.BatchedRefiner(backend, 32).predict(
                ob_in_cams=poses, iteration=5
            )
        np.testing.assert_array_equal(result, poses + 1)
        self.assertEqual(
            [
                len(c.kwargs['ob_in_cams'])
                for c in backend.predict.call_args_list
            ],
            [32, 32, 13],
        )
        self.assertTrue(
            all(
                c.kwargs['iteration'] == 5
                for c in backend.predict.call_args_list
            )
        )

    def test_score_feature_batching_keeps_full_pair_order(self):
        original = mock.Mock(side_effect=lambda a, b: a + b)
        scorer = types.SimpleNamespace(
            model=types.SimpleNamespace(
                extract_feat=original, att_cross=object()
            ),
            dataset=types.SimpleNamespace(transform_batch=mock.Mock()),
        )
        attention = scorer.model.att_cross
        batched_refiner.batch_score_features(scorer, 32)
        torch = types.SimpleNamespace(
            cat=lambda xs, dim: np.concatenate(xs, axis=dim)
        )
        a = np.arange(77).reshape(-1, 1)
        with mock.patch.dict(sys.modules, {'torch': torch}):
            result = scorer.model.extract_feat(a, a * 2)
        np.testing.assert_array_equal(result, a * 3)
        self.assertIs(scorer.model.att_cross, attention)
        self.assertEqual(
            [len(c.args[0]) for c in original.call_args_list], [32, 32, 13]
        )
