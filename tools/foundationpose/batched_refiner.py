"""Bound independent refinement batches without dropping pose hypotheses."""


class BatchedRefiner:
    def __init__(self, refiner, batch_size=32):
        self.refiner = refiner
        self.batch_size = batch_size

    def predict(self, **kwargs):
        poses = kwargs['ob_in_cams']
        if len(poses) <= self.batch_size or kwargs.get('get_vis'):
            return self.refiner.predict(**kwargs)
        import torch
        outputs = []
        for start in range(0, len(poses), self.batch_size):
            pose, _ = self.refiner.predict(
                **{**kwargs, 'ob_in_cams': poses[start:start + self.batch_size]}
            )
            outputs.append(pose)
            torch.cuda.empty_cache()
        return torch.cat(outputs, dim=0), None


def batch_score_features(scorer, batch_size=32):
    """Batch independent image features; retain full cross-candidate attention."""
    original = scorer.model.extract_feat

    def extract(A, B):
        if len(A) <= batch_size:
            return original(A, B)
        import torch
        return torch.cat(
            [
                original(A[i:i + batch_size], B[i:i + batch_size])
                for i in range(0, len(A), batch_size)
            ],
            dim=0,
        )

    scorer.model.extract_feat = extract
    original_transform = scorer.dataset.transform_batch

    def transform(batch, *args, **kwargs):
        count = len(batch.poseA)
        if count <= batch_size:
            return original_transform(batch, *args, **kwargs)
        import torch
        chunks = []
        for start in range(0, count, batch_size):
            indices = torch.arange(
                start, min(start + batch_size, count), device=batch.poseA.device
            )
            chunks.append(
                original_transform(
                    batch.select_by_indices(indices), *args, **kwargs
                )
            )
        output = type(batch)()
        for key, value in chunks[0].__dict__.items():
            setattr(
                output,
                key,
                (
                    None
                    if value is None
                    else torch.cat(
                        [getattr(part, key) for part in chunks], dim=0
                    )
                ),
            )
        return output

    scorer.dataset.transform_batch = transform
    return scorer


class AutocastSam:
    """Keep SAM weights intact while bounding encoder/decoder activation memory."""

    def __init__(self, sam):
        self.sam = sam

    def set_image(self, *args, **kwargs):
        import torch
        with torch.autocast('cuda', dtype=torch.float16):
            return self.sam.set_image(*args, **kwargs)

    def predict(self, *args, **kwargs):
        import torch
        with torch.autocast('cuda', dtype=torch.float16):
            return self.sam.predict(*args, **kwargs)
