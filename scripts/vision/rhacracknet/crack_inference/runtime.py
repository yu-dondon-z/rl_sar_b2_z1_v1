"""Small, dependency-light inference wrapper around RHACrackNet."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import time

import cv2
import numpy as np
from PIL import Image
import torch


SUPPORTED_MODELS = ("RHANet", "RHANet_dep")


@dataclass
class SegmentationResult:
    probability: np.ndarray
    mask: np.ndarray
    overlay_bgr: np.ndarray
    inference_ms: float


def build_model(model_name: str) -> torch.nn.Module:
    if model_name not in SUPPORTED_MODELS:
        raise ValueError(f"Unsupported model {model_name!r}; choose one of {SUPPORTED_MODELS}")
    config = SimpleNamespace(model=SimpleNamespace(model_name=model_name))
    if model_name == "RHANet":
        from models.RHANet.rhanet import RHANet
    else:
        from models.RHANet_dep.rhanet_dep import RHANet
    return RHANet(config)


def _extract_state_dict(checkpoint):
    if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
        checkpoint = checkpoint["model_state_dict"]
    if not isinstance(checkpoint, dict):
        raise TypeError("Checkpoint must contain a PyTorch state dictionary")
    state_dict = {}
    for key, value in checkpoint.items():
        key = key[7:] if key.startswith("module.") else key
        # thop.profile() registers these statistics as buffers on every
        # visited module. They are not learned parameters and must not make a
        # training checkpoint incompatible with a clean inference model.
        if key.endswith("total_ops") or key.endswith("total_params"):
            continue
        state_dict[key] = value
    return state_dict


class CrackSegmenter:
    """Load a model once and segment RGB/BGR frames repeatedly."""

    def __init__(
        self,
        checkpoint_path: str | Path | None,
        model_name: str = "RHANet_dep",
        device: str = "auto",
        threshold: float = 0.5,
        input_size: tuple[int, int] | None = None,
        allow_random_weights: bool = False,
    ):
        if not 0.0 <= threshold <= 1.0:
            raise ValueError("threshold must be within [0, 1]")
        if device == "auto":
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.threshold = float(threshold)
        self.input_size = input_size
        self.model_name = model_name
        self.model = build_model(model_name).to(self.device)

        if checkpoint_path is None:
            if not allow_random_weights:
                raise ValueError(
                    "A trained checkpoint is required. Use --allow-random-weights "
                    "only for pipeline testing."
                )
        else:
            checkpoint_path = Path(checkpoint_path).expanduser().resolve()
            if not checkpoint_path.is_file():
                raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")
            payload = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
            state_dict = _extract_state_dict(payload)
            incompatible = self.model.load_state_dict(state_dict, strict=True)
            if incompatible.missing_keys or incompatible.unexpected_keys:
                raise RuntimeError(
                    f"Checkpoint mismatch: missing={incompatible.missing_keys}, "
                    f"unexpected={incompatible.unexpected_keys}"
                )
        self.model.eval()

    @staticmethod
    def _to_tensor(rgb: np.ndarray) -> torch.Tensor:
        array = np.ascontiguousarray(rgb, dtype=np.float32) / 255.0
        return torch.from_numpy(array.transpose(2, 0, 1)).unsqueeze(0)

    @torch.inference_mode()
    def predict_bgr(self, frame_bgr: np.ndarray) -> SegmentationResult:
        if frame_bgr is None or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("Expected a non-empty HxWx3 BGR frame")
        original_h, original_w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        if self.input_size is not None:
            height, width = self.input_size
            rgb = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_LINEAR)

        tensor = self._to_tensor(rgb).to(self.device)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        start = time.perf_counter()
        output = self.model(tensor)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        inference_ms = (time.perf_counter() - start) * 1000.0

        probability = output[0, 0].detach().float().cpu().numpy()
        if probability.shape != (original_h, original_w):
            probability = cv2.resize(
                probability, (original_w, original_h), interpolation=cv2.INTER_LINEAR
            )
        probability = np.clip(probability, 0.0, 1.0)
        mask = (probability >= self.threshold).astype(np.uint8) * 255

        color_mask = np.zeros_like(frame_bgr)
        color_mask[..., 1] = mask
        overlay = cv2.addWeighted(frame_bgr, 0.78, color_mask, 0.55, 0.0)
        return SegmentationResult(probability, mask, overlay, inference_ms)

    def predict_image(self, image_path: str | Path) -> SegmentationResult:
        image_path = Path(image_path)
        with Image.open(image_path) as image:
            rgb = np.asarray(image.convert("RGB"))
        return self.predict_bgr(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    def warmup(self, height: int, width: int, iterations: int = 3) -> None:
        dummy = np.zeros((height, width, 3), dtype=np.uint8)
        for _ in range(max(0, iterations)):
            self.predict_bgr(dummy)
