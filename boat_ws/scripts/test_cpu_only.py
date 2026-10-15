import onnxruntime as ort

MODEL_PATH = "/workspace/boat_ws/models/ewasr_resnet18.onnx"

print("ONNX Runtime version:", ort.__version__)
print("Available providers:", ort.get_available_providers())

so = ort.SessionOptions()
so.intra_op_num_threads = 1
so.inter_op_num_threads = 1

print("Creating CPU session...")
sess = ort.InferenceSession(
    MODEL_PATH,
    sess_options=so,
    providers=["CPUExecutionProvider"]
)

print("CPU session created ✅")

print("\nInputs:")
for inp in sess.get_inputs():
    print("name:", inp.name)
    print("shape:", inp.shape)
    print("type:", inp.type)
    print()

print("Outputs:")
for out in sess.get_outputs():
    print("name:", out.name)
    print("shape:", out.shape)
    print("type:", out.type)
    print()
