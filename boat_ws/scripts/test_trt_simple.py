import os
import time
import numpy as np
import onnxruntime as ort

MODEL_PATH = "/workspace/boat_ws/models/ewasr_resnet18.onnx"

print("ONNX Runtime version:", ort.__version__)
print("Available providers:", ort.get_available_providers())

so = ort.SessionOptions()
so.intra_op_num_threads = 1
so.inter_op_num_threads = 1

providers = [
    (
        "TensorrtExecutionProvider",
        {
            "device_id": 0,
            "trt_fp16_enable": False,
            "trt_engine_cache_enable": True,
            "trt_engine_cache_path": "/workspace/boat_ws/trt_engine_cache",
        },
    ),
    ("CUDAExecutionProvider", {"device_id": 0}),
    "CPUExecutionProvider",
]

print("Creating TensorRT session...")
sess = ort.InferenceSession(
    MODEL_PATH,
    sess_options=so,
    providers=providers
)

print("TensorRT session created ✅")
print("Session providers:", sess.get_providers())

input_name = sess.get_inputs()[0].name
x = np.random.rand(1, 3, 384, 512).astype(np.float32)

print("Running first TensorRT inference...")
start = time.time()
outputs = sess.run(None, {input_name: x})
end = time.time()

print(f"First inference time: {(end - start) * 1000:.2f} ms")

for i, out in enumerate(outputs):
    print(f"Output {i}: shape={out.shape}, dtype={out.dtype}")

print("Running benchmark...")
for _ in range(5):
    sess.run(None, {input_name: x})

repeat = 20
start = time.time()
for _ in range(repeat):
    sess.run(None, {input_name: x})
end = time.time()

print(f"Average TensorRT inference time: {(end - start) * 1000 / repeat:.2f} ms")

os._exit(0)
