"""EWaSR model loading and image pre/post-processing."""

import os
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


DEFAULT_MODEL_PATHS = (
    "/workspace/boat_ws/models/ewasr_resnet18.onnx",
    "/home/ubuntu2204/smartboat/boat_ws/models/ewasr_resnet18.onnx",
)


def resolve_model_path(configured_path=""):
    """Resolve the runtime model without depending on a demo-script location."""
    candidates = (
        configured_path,
        os.environ.get("SMARTBOAT_EWASR_MODEL", ""),
        *DEFAULT_MODEL_PATHS,
    )
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    return configured_path or DEFAULT_MODEL_PATHS[0]


def preprocess_bgr(color_image, model_width=512, model_height=384):
    """Convert a BGR image to the contiguous NCHW float input EWaSR expects."""
    image = cv2.resize(
        color_image,
        (int(model_width), int(model_height)),
        interpolation=cv2.INTER_LINEAR,
    )
    image = image.astype(np.float32) / 255.0
    image = np.transpose(image, (2, 0, 1))
    image = np.expand_dims(image, axis=0)
    return np.ascontiguousarray(image, dtype=np.float32)


def postprocess_mask(outputs):
    """Convert the first EWaSR logits tensor to a uint8 class mask."""
    if not outputs:
        raise ValueError("EWaSR returned no output tensors")
    prediction = np.asarray(outputs[0])
    if prediction.ndim != 4 or prediction.shape[0] < 1:
        raise ValueError(
            f"unexpected EWaSR output shape: {prediction.shape}"
        )
    return np.argmax(prediction[0], axis=0).astype(np.uint8)


def create_ort_session(model_path, use_fp16=True, cache_root=""):
    """Create the maintained TensorRT/CUDA/CPU fallback provider chain."""
    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = 1
    session_options.inter_op_num_threads = 1

    if not cache_root:
        cache_root = "/workspace/boat_ws/trt_engine_cache"
    precision = "fp16" if use_fp16 else "fp32"
    cache_path = str(Path(cache_root) / f"ewasr_{precision}")
    try:
        Path(cache_path).mkdir(parents=True, exist_ok=True)
    except OSError:
        cache_path = cache_root

    providers = [
        (
            "TensorrtExecutionProvider",
            {
                "device_id": 0,
                "trt_fp16_enable": bool(use_fp16),
                "trt_engine_cache_enable": True,
                "trt_engine_cache_path": cache_path,
            },
        ),
        ("CUDAExecutionProvider", {"device_id": 0}),
        "CPUExecutionProvider",
    ]
    return ort.InferenceSession(
        resolve_model_path(model_path),
        sess_options=session_options,
        providers=providers,
    )
