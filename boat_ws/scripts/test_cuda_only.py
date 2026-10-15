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
    ("CUDAExecutionProvider", {"device_id": 0}),
    "CPUExecutionProvider",
]

print("Creating CUDA session...")
sess = ort.InferenceSession(
    MODEL_PATH,
    sess_options=so,
    providers=providers
)

print("CUDA session created ✅")
print("Session providers:", sess.get_providers())

input_name = sess.get_inputs()[0].name
x = np.random.rand(1, 3, 384, 512).astype(np.float32)

print("Running first CUDA inference...")
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

print(f"Average CUDA inference time: {(end - start) * 1000 / repeat:.2f} ms")

# 避免 onnxruntime 退出时 free() 崩溃影响结果展示
os._exit(0)
