import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import live_tracker
from llm_detection import LLMBoxDetector, DetectionCancelled, parse_detection
from object_catalog import select_object, save_detection_prompt


def response(box=None, status='found'):
    return {'status':'completed','output':[{'type':'function_call','name':'report_regions',
        'arguments':json.dumps({'status':status,'candidates':[{'box':box or [20,30,100,120],'description':'纸杯'}] if status=='found' else []})}]}


class DetectionTests(unittest.TestCase):
    def frame(self):
        return {'bgr':np.zeros((360,640,3),np.uint8),'time':time.time()}

    def test_payload_and_padding(self):
        payloads=[]
        detector=LLMBoxDetector(request=lambda payload:(payloads.append(payload) or response()))
        boxes=detector.detect(self.frame(),'无把手纸杯',lambda:False)
        self.assertEqual(boxes[0]['box'],[16,26,104,124])
        p=payloads[0];self.assertEqual(p['model'],'gpt-5.6-terra');self.assertEqual(p['reasoning'],{'effort':'low'})
        self.assertFalse(p['store']);self.assertEqual(p['tools'][0]['name'],'report_regions')
        self.assertIn('无把手纸杯',p['input'][0]['content'][0]['text'])

    def test_invalid_or_missing_boxes_never_reach_segmentation(self):
        for box in [[-1,0,50,50],[0,0,641,50],[30,30,20,100],[1,1,4,4],[True,0,100,100],[0,0,float('nan'),100]]:
            with self.subTest(box=box),self.assertRaises(ValueError):parse_detection(response(box),640,360)
        for r in [[],{'status':'incomplete'},response(status='not_found'),response(status='uncertain')]:
            with self.subTest(response=r),self.assertRaises(ValueError):parse_detection(r,640,360)
        r=response();r['output']*=2
        with self.assertRaises(ValueError):parse_detection(r,640,360)

    def test_cancel_discards_late_reply_and_prevents_parallel_requests(self):
        entered=threading.Event();release=threading.Event();cancel=threading.Event();errors=[]
        def request(_):entered.set();release.wait(2);return response()
        detector=LLMBoxDetector(request=request)
        def detect():
            try:detector.detect(self.frame(),'纸杯',cancel.is_set)
            except Exception as error:errors.append(error)
        thread=threading.Thread(target=detect);thread.start();self.assertTrue(entered.wait(1));cancel.set();thread.join(1)
        self.assertFalse(thread.is_alive());self.assertIsInstance(errors[0],DetectionCancelled)
        with self.assertRaisesRegex(ValueError,'尚未退出'):detector.detect(self.frame(),'杯',lambda:False)
        release.set()
        for _ in range(100):
            if not detector.inflight.locked():break
            time.sleep(.005)
        self.assertFalse(detector.inflight.locked())

    def test_timeout_and_stale_frame(self):
        release=threading.Event()
        detector=LLMBoxDetector(timeout=.01,request=lambda _:(release.wait(.2) or response()))
        try:
            with self.assertRaisesRegex(ValueError,'超过'):detector.detect(self.frame(),'杯',lambda:False)
        finally:release.set()
        frame=self.frame();frame['time']-=31
        with self.assertRaisesRegex(ValueError,'时限'):
            LLMBoxDetector(request=lambda _:response()).detect(frame,'杯',lambda:False)


class PromptTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.root=Path(temp.name)
        self.folder=self.root/'objects/cup';self.folder.mkdir(parents=True)
        (self.folder/'model.obj').write_text('# mesh')
        (self.folder/'tracking.json').write_text(json.dumps({'name':'纸杯','detector_class':None,'pose_mode':'5d'}))
        self.models=self.root/'objects'

    def save(self,prompt):
        row=select_object(self.models,'cup')
        return save_detection_prompt(self.models,'cup',prompt,row['detection_prompt_revision'])

    def test_storage_revision_conflict_fallback_and_pose_independence(self):
        original=select_object(self.models,'cup');row=self.save(' 无把手的一次性纸杯 ')
        self.assertEqual(row['effective_detection_prompt'],'无把手的一次性纸杯')
        self.assertEqual(row['pose_mode_revision'],original['pose_mode_revision'])
        with self.assertRaisesRegex(ValueError,'已变化'):
            save_detection_prompt(self.models,'cup','马克杯',original['detection_prompt_revision'])
        row=self.save('');self.assertEqual(row['effective_detection_prompt'],'纸杯')
        self.assertEqual(len(list((self.folder/'detection_backups').glob('*.json'))),1)
        for value in [None,42,'x'*2001]:
            with self.assertRaises(ValueError):self.save(value)

    def test_symlink_rejected(self):
        (self.root/'outside').write_text('{}');(self.folder/'detection.json').symlink_to(self.root/'outside')
        with self.assertRaises(ValueError):select_object(self.models,'cup')

    def tracker(self):
        args=SimpleNamespace(mesh=self.folder/'model.obj',models_root=self.models,serial='test',auto_recover=True,recovery_interval=0,detector='llm',grasp_config=live_tracker.ROOT/'config/grasp/trial.json')
        with patch.object(live_tracker,'ROOT',self.root):tracker=live_tracker.Tracker(args)
        tracker.state.update(state='tracking',model_ready=True);return tracker

    def test_hot_update_keeps_tracking_but_invalidates_pending_recognition(self):
        t=self.tracker();self.assertTrue(t.state['auto_detection_supported'])
        for state in ['tracking','detecting']:
            t.state['state']=state;generation=t.generation;row=select_object(self.models,'cup')
            t.set_detection_prompt({'id':'cup','prompt':state,'revision':row['detection_prompt_revision']})
            self.assertEqual(t.state['state'],'tracking' if state=='tracking' else 'ready')
            self.assertEqual(t.generation,generation+(state!='tracking'))
        self.assertEqual(t.state['effective_detection_prompt'],'detecting')

    def test_bounded_recovery_and_expired_candidate(self):
        t=self.tracker();t.latest={'time':time.time(),'sequence':1};t.automatic();t.mark_lost('失败')
        self.assertGreater(t.retry_at,time.monotonic()+4)
        for _ in range(2):t.automatic(recovery=True);t.mark_lost('失败')
        self.assertEqual(t.state['state'],'lost');self.assertFalse(t.recover_if_due())
        t.automatic();self.assertEqual(t.state['recovery_attempts'],0)
        t.state.update(state='choose_candidate',bottle_candidates=[{'box':[0,0,30,30]}]);t.frozen['time']-=31
        with self.assertRaisesRegex(ValueError,'30秒'):t.choose({'selection_id':t.generation,'index':0})


if __name__=='__main__':unittest.main()
