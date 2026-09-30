import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
from batched_refiner import BatchedRefiner,batch_score_features


class BatchingTests(unittest.TestCase):
    def test_refiner_keeps_every_candidate_in_order(self):
        backend=Mock();backend.predict.side_effect=lambda **kw:(kw['ob_in_cams']+1,None)
        torch=SimpleNamespace(cat=lambda xs,dim:np.concatenate(xs,axis=dim),cuda=SimpleNamespace(empty_cache=lambda:None))
        poses=np.arange(77).reshape(-1,1)
        with patch.dict(sys.modules,{'torch':torch}):result,_=BatchedRefiner(backend,32).predict(ob_in_cams=poses,iteration=5)
        np.testing.assert_array_equal(result,poses+1)
        self.assertEqual([len(c.kwargs['ob_in_cams']) for c in backend.predict.call_args_list],[32,32,13])
        self.assertTrue(all(c.kwargs['iteration']==5 for c in backend.predict.call_args_list))

    def test_score_feature_batching_keeps_full_pair_order(self):
        original=Mock(side_effect=lambda a,b:a+b);scorer=SimpleNamespace(model=SimpleNamespace(extract_feat=original,att_cross=object()))
        attention=scorer.model.att_cross;batch_score_features(scorer,32)
        torch=SimpleNamespace(cat=lambda xs,dim:np.concatenate(xs,axis=dim))
        a=np.arange(77).reshape(-1,1)
        with patch.dict(sys.modules,{'torch':torch}):result=scorer.model.extract_feat(a,a*2)
        np.testing.assert_array_equal(result,a*3);self.assertIs(scorer.model.att_cross,attention)
        self.assertEqual([len(c.args[0]) for c in original.call_args_list],[32,32,13])
