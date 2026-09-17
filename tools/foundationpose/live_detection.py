"""Device isolation between upstream FoundationPose and Torchvision detection."""
import torch


def predict_detection(detector, rgb):
    # FoundationPose's refiner changes the global default tensor type to CUDA.
    # Torchvision SSD builds CPU anchors internally; its implicit factories must
    # remain on CPU. Explicit image/model tensors still use the detector device.
    # This scoped context restores the caller's defaults on exit, including error.
    with torch.device('cpu'), torch.inference_mode():
        device = next(detector.parameters()).device
        image = torch.from_numpy(rgb).permute(2, 0, 1).to(device=device, dtype=torch.float32) / 255
        return detector([image])[0]
