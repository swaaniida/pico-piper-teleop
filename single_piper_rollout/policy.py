"""LeRobot 0.4.4 ACT loading and observation preprocessing."""

from __future__ import annotations

import json
from importlib.metadata import version
from pathlib import Path
from typing import Any

import numpy as np


class ActRuntime:
    def __init__(self, bundle: Path, device: str = "cuda") -> None:
        self.bundle = bundle.resolve()
        self.manifest = json.loads((self.bundle / "manifest.json").read_text())
        if self.manifest["action_representation"] != "absolute":
            raise RuntimeError("this runtime currently accepts absolute actions only")
        try:
            import torch
            from lerobot.policies.act.modeling_act import ACTPolicy
            from lerobot.policies.factory import make_pre_post_processors
        except ImportError as exc:
            raise RuntimeError("install the pinned LeRobot inference environment first") from exc
        expected = self.manifest["software"]["lerobot_version"]
        installed = version("lerobot")
        if installed != expected:
            raise RuntimeError(f"LeRobot version must be {expected}, got {installed}")
        self.torch = torch
        model_path = self.bundle / self.manifest["checkpoint"]["pretrained_model_directory"]
        self.policy = ACTPolicy.from_pretrained(model_path).to(device).eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config,
            pretrained_path=model_path,
            preprocessor_overrides={"device_processor": {"device": device}},
            postprocessor_overrides={"device_processor": {"device": "cpu"}},
        )
        self.device = device
        self.input_hw = tuple(self.manifest["features"]["policy_input_shape_chw"][1:])

    def reset(self) -> None:
        self.policy.reset()
        for processor in (self.preprocessor, self.postprocessor):
            reset = getattr(processor, "reset", None)
            if callable(reset):
                reset()

    def _image(self, rgb_hwc: np.ndarray):
        if rgb_hwc.shape != (480, 640, 3) or rgb_hwc.dtype != np.uint8:
            raise ValueError(f"camera frame must be uint8 HWC 480x640x3, got {rgb_hwc.shape}/{rgb_hwc.dtype}")
        tensor = self.torch.from_numpy(np.ascontiguousarray(rgb_hwc)).permute(2, 0, 1).float().div_(255.0)
        if tuple(tensor.shape[1:]) != self.input_hw:
            from torchvision.transforms.functional import resize
            tensor = resize(tensor, list(self.input_hw), antialias=True)
        return tensor

    def predict(self, local_rgb: np.ndarray, global_rgb: np.ndarray, state: np.ndarray) -> np.ndarray:
        if state.shape != (9,) or not np.all(np.isfinite(state)):
            raise ValueError("state must be finite shape (9,)")
        observation: dict[str, Any] = {
            "observation.images.local": self._image(local_rgb),
            "observation.images.global": self._image(global_rgb),
            "observation.state": self.torch.from_numpy(state.astype(np.float32, copy=False)),
        }
        with self.torch.inference_mode():
            batch = self.preprocessor(observation)
            action = self.policy.select_action(batch)
            action = self.postprocessor(action)
        result = action.detach().cpu().numpy()
        result = result[0] if result.ndim == 2 else result
        if result.shape != (9,) or not np.all(np.isfinite(result)):
            raise RuntimeError(f"policy returned invalid action {result.shape}")
        return result.astype(np.float64, copy=False)
