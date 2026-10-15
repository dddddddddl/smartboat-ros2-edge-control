import os
import time
import numpy as np
import onnxruntime as ort

MODEL_PATH = "/workspace/boat_ws/models/ewasr_resnet18.onnx"

print("ONNX Runtime version:", ort.__version__)
print("Available providers:", ort.get_available_providers())

if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"模型不存在: {MODEL_PATH}")

providers = [
    (
        "TensorrtExecutionProvider",
        {
            "device_id": 0,
            "trt_fp16_enable": True,
            "trt_engine_cache_enable": True,
            "trt_engine_cache_path": "/workspace/boat_ws/trt_engine_cache",
        },
    ),
    "CUDAExecutionProvider",
    "CPUExecutionProvider",
]

print("Creating InferenceSession...")
session = ort.InferenceSession(MODEL_PATH, providers=providers)

print("Session providers actually used:")
print(session.get_providers())

inputs = {}

print("\nModel inputs:")
for inp in session.get_inputs():
    name = inp.name
    shape = inp.shape
    dtype = inp.type

    real_shape = []
    for dim in shape:
        if isinstance(dim, int):
            real_shape.append(dim)
        else:
            real_shape.append(1)

    if dtype == "tensor(float)":
        data = np.random.rand(*real_shape).astype(np.float32)
    elif dtype == "tensor(float16)":
        data = np.random.rand(*real_shape).astype(np.float16)
    elif dtype == "tensor(int64)":
        data = np.random.randint(0, 10, size=real_shape).astype(np.int64)
    elif dtype == "tensor(int32)":
        data = np.random.randint(0, 10, size=real_shape).astype(np.int32)
    else:
        raise TypeError(f"暂时不支持这个输入类型: {dtype}")

    inputs[name] = data
    print(f"- name={name}, model_shape={shape}, test_shape={data.shape}, dtype={data.dtype}")

print("\nRunning first inference...")
start = time.time()
outputs = session.run(None, inputs)
end = time.time()

print(f"First inference time: {(end - start) * 1000:.2f} ms")

print("\nModel outputs:")
for i, out in enumerate(outputs):
    print(f"- output {i}: shape={out.shape}, dtype={out.dtype}")

print("\nRunning benchmark...")
for _ in range(5):
    session.run(None, inputs)

repeat = 20
start = time.time()
for _ in range(repeat):
    session.run(None, inputs)
end = time.time()

print(f"Average inference time: {(end - start) * 1000 / repeat:.2f} ms")

