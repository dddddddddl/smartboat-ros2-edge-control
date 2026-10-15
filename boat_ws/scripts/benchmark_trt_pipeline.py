import os
import sys
import time
import numpy as np
import cv2
import onnxruntime as ort

MODEL_PATH = "/workspace/boat_ws/models/ewasr_resnet18.onnx"

LOOPS = 1000
WARMUP = 10
THREADS = 4

FRAME_H = 720
FRAME_W = 1280

MODEL_W = 512
MODEL_H = 384

print("=" * 60)
print("Benchmark")
print("=" * 60)
print(f"loops={LOOPS}, warmup={WARMUP}, threads={THREADS}")
print(f"random frame shape=({FRAME_H}, {FRAME_W}, 3), model input resize=({MODEL_W}, {MODEL_H})")

so = ort.SessionOptions()
so.intra_op_num_threads = THREADS
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

sess = ort.InferenceSession(
    MODEL_PATH,
    sess_options=so,
    providers=providers
)

print("providers:", sess.get_providers())

input_name = sess.get_inputs()[0].name

frame = np.random.randint(0, 256, size=(FRAME_H, FRAME_W, 3), dtype=np.uint8)


def preprocess(frame):
    img = cv2.resize(frame, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    img = np.ascontiguousarray(img, dtype=np.float32)
    return img


def postprocess(outputs):
    prediction = outputs[0]
    mask = np.argmax(prediction[0], axis=0).astype(np.uint8)
    return mask


for _ in range(WARMUP):
    x = preprocess(frame)
    outputs = sess.run(None, {input_name: x})
    mask = postprocess(outputs)

preprocess_times = []
inference_times = []
postprocess_times = []
total_times = []

t_wall_start = time.perf_counter()

for i in range(LOOPS):
    t0 = time.perf_counter()

    x = preprocess(frame)

    t1 = time.perf_counter()

    outputs = sess.run(None, {input_name: x})

    t2 = time.perf_counter()

    mask = postprocess(outputs)

    t3 = time.perf_counter()

    preprocess_times.append((t1 - t0) * 1000)
    inference_times.append((t2 - t1) * 1000)
    postprocess_times.append((t3 - t2) * 1000)
    total_times.append((t3 - t0) * 1000)

    if (i + 1) % 100 == 0:
        print(f"progress: {i + 1}/{LOOPS}")

t_wall_end = time.perf_counter()

avg_pre = float(np.mean(preprocess_times))
avg_inf = float(np.mean(inference_times))
avg_post = float(np.mean(postprocess_times))
avg_total = float(np.mean(total_times))

wall_time = t_wall_end - t_wall_start
fps_pipeline = 1000.0 / avg_total
fps_wall = LOOPS / wall_time

print("=" * 60)
print("Result")
print("=" * 60)
print(f"测试帧数：{LOOPS}")
print(f"总墙钟耗时：{wall_time:.4f} s")
print(f"平均预处理：{avg_pre:10.3f} ms")
print(f"平均模型推理：{avg_inf:10.3f} ms")
print(f"平均后处理：{avg_post:10.3f} ms")
print(f"平均总延迟：{avg_total:10.3f} ms")
print(f"FPS pipeline: {fps_pipeline:10.2f}")
print(f"FPS wall:     {fps_wall:10.2f}")
print(f"输出 mask shape: {mask.shape}, dtype={mask.dtype}")
print("=" * 60)

sys.stdout.flush()
os._exit(0)
