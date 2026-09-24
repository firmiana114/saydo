"""One persistent arm speed for new tasks; never sends a robot command."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import time


def checked_speed(value):
    if type(value) is not int or not 1 <= value <= 25:
        raise ValueError('机械臂速度必须为1–25的整数百分比')
    return value


def reorientation_speed_settings(config, settings):
    return {**settings, 'arm_speed_percent':checked_speed(config['arm_speed_percent'])}


def placement_speed_settings(config, settings):
    speed=checked_speed(config['arm_speed_percent'])
    return {**settings, 'transfer_speed_percent':speed, 'descent_speed_percent':speed}


class MotionSpeedMixin:
    def motion_speed_configuration(self):
        try:
            raw=self.config_path.read_bytes();config=json.loads(raw)
            return {'value':checked_speed(config['arm_speed_percent']), 'minimum':1, 'maximum':25,
                    'revision':hashlib.sha256(raw).hexdigest(), 'scope':'all_arm_motion', 'error':None}
        except (OSError,ValueError,KeyError,TypeError) as error:
            return {'value':None,'minimum':1,'maximum':25,'error':str(error)}

    def set_arm_speed(self, value, expected_revision):
        value=checked_speed(value)
        with self.lock:
            if self.status['busy'] or self.status.get('stopping'):
                raise ValueError('机械臂任务进行中，不能更改速度；请等待任务结束')
            directory=self.root/'work/grasp';directory.mkdir(parents=True,exist_ok=True)
            report={'at':time.time(),'requested_speed_percent':value,'saved':False,'motion_sent':False}
            temporary=None
            try:
                with (directory/'robot.lock').open('a') as lock:
                    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    original_bytes=self.config_path.read_bytes();original=original_bytes.decode()
                    if expected_revision!=hashlib.sha256(original_bytes).hexdigest():
                        raise ValueError('速度或配置已变化，请重新载入当前速度后保存')
                    config=json.loads(original);report['previous_speed_percent']=config['arm_speed_percent']
                    config['arm_speed_percent']=value
                    updated,count=re.subn(r'("arm_speed_percent"\s*:\s*)\d+',lambda m:m[1]+str(value),original)
                    if count!=1 or json.loads(updated)!=config:
                        raise ValueError('速度配置格式不明确，未保存')
                    if updated!=original:
                        with tempfile.NamedTemporaryFile(mode='w',dir=self.config_path.parent,prefix='.speed-',delete=False) as f:
                            temporary=Path(f.name);os.fchmod(f.fileno(),self.config_path.stat().st_mode&0o777)
                            f.write(updated);f.flush();os.fsync(f.fileno())
                        if self.config_path.read_bytes()!=original_bytes:
                            raise ValueError('配置同时被修改，未保存速度')
                        temporary.replace(self.config_path)
                    report['saved']=True
                    self.status.update(plan=None,reorientation_plan=None,placement_plan=None,object_motion_plan=None)
            except Exception as error:
                report.update(error=str(error),error_type=type(error).__name__)
                raise
            finally:
                if temporary is not None:temporary.unlink(missing_ok=True)
                records=directory/'speed_commands';records.mkdir(exist_ok=True)
                (records/f'{time.time_ns()}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            return {**self.motion_speed_configuration(),'saved':True,'motion_sent':False,
                    'message':f'已保存统一速度{value}%，下次抓取、姿态调整、放置和复位生效。'}
