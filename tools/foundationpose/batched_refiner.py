"""Bound independent refinement batches without dropping pose hypotheses."""
class BatchedRefiner:
    def __init__(self,refiner,batch_size=32):
        self.refiner=refiner;self.batch_size=batch_size

    def predict(self,**kwargs):
        poses=kwargs['ob_in_cams']
        if len(poses)<=self.batch_size or kwargs.get('get_vis'):
            return self.refiner.predict(**kwargs)
        import torch
        outputs=[]
        for start in range(0,len(poses),self.batch_size):
            pose,_=self.refiner.predict(**{**kwargs,'ob_in_cams':poses[start:start+self.batch_size]})
            outputs.append(pose)
            torch.cuda.empty_cache()
        return torch.cat(outputs,dim=0),None
