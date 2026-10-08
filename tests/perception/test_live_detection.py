import unittest
import warnings
import numpy as np
import torch
from torchvision.models.detection.anchor_utils import DefaultBoxGenerator

from saydo.perception.live_detection import predict_detection


class AnchorProbe(torch.nn.Module):
    """Exercise the actual Torchvision SSD anchor code without downloading weights."""

    def __init__(self):
        super().__init__()
        self.weight = torch.nn.Parameter(torch.zeros(1, device='cuda'))
        self.generator = DefaultBoxGenerator([[2, 3]] * 6)

    def forward(self, images):
        assert images[0].is_cuda
        boxes = self.generator._grid_default_boxes([[2, 2]] * 6, [320, 320])
        return [{'boxes': boxes.to(images[0].device)}]


@unittest.skipUnless(
    torch.cuda.is_available(),
    'CUDA required for upstream global-default regression',
)
class DetectionDeviceTests(unittest.TestCase):
    def test_redetection_after_foundationpose_global_cuda_default(self):
        previous = torch.empty(0).type()
        try:
            probe = AnchorProbe()
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', UserWarning)
                torch.set_default_tensor_type('torch.cuda.FloatTensor')
            with self.assertRaisesRegex(RuntimeError, 'same device'):
                probe.generator._grid_default_boxes([[2, 2]] * 6, [320, 320])
            for _ in range(3):
                result = predict_detection(
                    probe, np.zeros((32, 32, 3), np.uint8)
                )
                self.assertTrue(result['boxes'].is_cuda)
                self.assertTrue(
                    torch.empty(0).is_cuda,
                    'adapter must restore FoundationPose defaults',
                )
        finally:
            torch.set_default_tensor_type(previous)


if __name__ == '__main__':
    unittest.main()
