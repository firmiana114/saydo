"""Shared backend rotation step bound for every object-orientation entrypoint."""
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

MAX_ROTATION_STEP_DEG=15.


def checked_step(value=15.):
    if type(value) not in (int,float) or not np.isfinite(value) or not 1<=value<=MAX_ROTATION_STEP_DEG:
        raise ValueError('旋转步长必须为1–15度')
    return float(value)


def interpolate(a,b,fraction):
    return Slerp([0.,1.],Rotation.from_matrix(np.stack([a,b])))([fraction]).as_matrix()[0]


def fractions(a,b,step):
    step=checked_step(step)
    angle=np.rad2deg(np.arccos(np.clip((np.trace(np.asarray(a).T@b)-1)/2,-1,1)))
    # Controller RPY is quantized to 0.001 rad. Two rounded endpoints can add
    # up to 0.172 degrees of relative rotation; reserve 0.2 degrees so the
    # actual commanded endpoint delta still respects the requested maximum.
    count=max(1,int(np.ceil(angle/(step-.2))))
    return [i/count for i in range(1,count+1)]
