# ================== IMPORTS ==================
import os, wave, tempfile, subprocess, requests, signal
import numpy as np
import sounddevice as sd
import faiss
from sentence_transformers import SentenceTransformer
import torch
import whisper
import onnxruntime as ort
import librosa
import warnings
from threading import Thread, Lock
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ================== FLAGS ==================
DEBUG_WAKEWORD = False
ENABLE_TTS = True

# ================== WARNINGS ==================
warnings.filterwarnings("ignore", category=FutureWarning)
os.environ['ORT_LOGGING_LEVEL'] = '3'

# ================== Embedding + Whisper ==================
embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
whisper_model = whisper.load_model("base").to(device)

llama_url = "http://127.0.0.1:8080/completion"

# ================== GLOBAL ==================
mapping_process = None
latest_detection_text = "Chưa có dữ liệu từ camera"
detection_lock = Lock()

# ================== PROMPT ==================
initial_prompt = (
    "Your name is Azu. You are a compact, smart and polite robot assistant. "
    "Always respond in Vietnamese. Do not use emojis or exaggerated tone. "
    "Be concise, clear, and polite.\n"
    "- When user says 'khởi động mapping' or 'start mapping', respond EXACTLY: Azu sẽ khởi động mapping\n"
    "- When user says 'dừng mapping' or 'stop mapping', respond EXACTLY: Azu sẽ dừng mapping\n"
    "- When user asks 'có gì trước mặt' or similar, describe the camera data accurately.\n"
)

# ================== SOUNDS ==================
current_dir = os.path.dirname(os.path.abspath(__file__))
bip_sound = os.path.join(current_dir, "assets/bip.wav")
bip2_sound = os.path.join(current_dir, "assets/bip2.wav")

docs = ["The Jetson Nano is a compact edge AI computer."]

# ================== TTS CONFIG ==================
PIPER_BIN = "/home/azusa/piper/build/piper"
PIPER_MODEL_VI_ALIAS = "/usr/local/share/piper/models/vi.onnx"
PIPER_MODEL_VI = "/usr/local/share/piper/models/vi_VN-vais1000-medium.onnx"

def _pick_vi_model_path():
    return PIPER_MODEL_VI_ALIAS if os.path.exists(PIPER_MODEL_VI_ALIAS) else PIPER_MODEL_VI

# ================== WAKEWORD ==================
WAKEWORD_MODEL_PATH = "/home/azusa/ros2_ws/src/azu_local/wakewords/Azu.onnx"
WAKEWORD_THRESHOLD = 0.30
WAKE_SR = 16000
WAKE_WINDOW_SAMPLES = WAKE_SR * 1

providers = ['CUDAExecutionProvider','CPUExecutionProvider'] if torch.cuda.is_available() else ['CPUExecutionProvider']
wake_sess = ort.InferenceSession(WAKEWORD_MODEL_PATH, providers=providers)
wake_input = wake_sess.get_inputs()[0].name
wake_output = wake_sess.get_outputs()[0].name

wake_detected = False
wake_buffer = np.zeros(0, dtype=np.float32)

# ================== FAISS ==================
class VectorDatabase:
    def __init__(self, dim):
        self.index = faiss.IndexFlatL2(dim)
        self.documents = []

    def add_documents(self, docs):
        emb = embedding_model.encode(docs)
        self.index.add(np.array(emb, dtype=np.float32))
        self.documents.extend(docs)

    def search(self, text, k=3):
        q = embedding_model.encode([text])[0].astype(np.float32)
        _, idx = self.index.search(np.array([q]), k)
        return [self.documents[i] for i in idx[0]]

db = VectorDatabase(384)
db.add_documents(docs)

# ================== AUDIO ==================
def play_sound(path):
    os.system(f"aplay {path}")

def record_audio(path, duration=10, fs=16000):
    play_sound(bip_sound)
    audio = sd.rec(int(duration*fs), samplerate=fs, channels=1, dtype="int16")
    sd.wait()
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(fs)
        wf.writeframes(audio.tobytes())
    play_sound(bip2_sound)

# ================== WAKEWORD ==================
def extract_logmel(audio):
    if len(audio) < WAKE_WINDOW_SAMPLES:
        audio = np.pad(audio, (0, WAKE_WINDOW_SAMPLES - len(audio)))
    else:
        audio = audio[-WAKE_WINDOW_SAMPLES:]

    mel = librosa.feature.melspectrogram(y=audio, sr=16000, n_fft=400, hop_length=1000, n_mels=96)
    logmel = np.log(mel + 1e-10)

    if logmel.shape[1] < 16:
        logmel = np.pad(logmel, ((0,0),(0,16-logmel.shape[1])))
    else:
        logmel = logmel[:, -16:]

    return logmel.T[np.newaxis, :, :].astype(np.float32)

def run_wake(audio):
    feat = extract_logmel(audio)
    out = wake_sess.run([wake_output], {wake_input: feat})
    return float(out[0][0][0])

def wake_callback(indata, frames, t, status):
    global wake_buffer, wake_detected
    chunk = indata[:,0].astype(np.float32) / 32768.0
    wake_buffer = np.concatenate([wake_buffer, chunk])[-WAKE_WINDOW_SAMPLES:]

    if not wake_detected and len(wake_buffer) >= WAKE_WINDOW_SAMPLES:
        score = run_wake(wake_buffer)
        if DEBUG_WAKEWORD:
            print(f"[WakeWord] score={score:.3f}")
        if score >= WAKEWORD_THRESHOLD:
            if DEBUG_WAKEWORD:
                print("[WakeWord] DETECTED!")
            wake_detected = True

def listen_wakeword():
    global wake_detected, wake_buffer
    wake_detected = False
    wake_buffer = np.zeros(0, dtype=np.float32)

    print("Đang chờ gọi 'Azu'...")
    with sd.InputStream(samplerate=16000, channels=1, dtype="int16", blocksize=4000, callback=wake_callback):
        while not wake_detected:
            sd.sleep(100)
    print("Wakeword Azu được kích hoạt!")

# ================== STT ==================
def transcribe(path):
    res = whisper_model.transcribe(path, language="vi")
    return res.get("text","").strip()

# ================== LLM ==================
def ask_llama(query, ctx):
    data = {
        "prompt": f"{initial_prompt}\nContext: {ctx}\nUser: {query}\nAzu:",
        "max_tokens": 80,
        "temperature": 0.5
    }
    r = requests.post(llama_url, json=data)
    if r.status_code == 200:
        return r.json().get("content","").strip()
    return "Lỗi khi gọi LLM."

def rag_ask(text):
    if any(k in text.lower() for k in ["có gì","thấy gì","vật thể","object","kiểm tra"]):
        with detection_lock:
            return ask_llama(text, latest_detection_text)
    return ask_llama(text, " ".join(db.search(text)))

# ================== TTS ==================
def speak(text):
    if not ENABLE_TTS:
        return
    out = "response.wav"
    subprocess.run(
        [PIPER_BIN,"--model",_pick_vi_model_path(),"--output_file",out],
        input=text.encode("utf-8"), check=True
    )
    os.system(f"aplay {out}")

# ================== ROS2 ==================
class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/vision/detection_text',
            self.detection_callback,
            10
        )
        self.get_logger().info('Detection subscriber started')
    
    def detection_callback(self, msg: String):
        global latest_detection_text
        with detection_lock:
            latest_detection_text = msg.data

def ros2_spin_thread(node):
    rclpy.spin(node)

# ================== MAIN ==================
def main():

    global mapping_process

    rclpy.init()
    detection_node = DetectionSubscriber()

    ros_thread = Thread(target=ros2_spin_thread, args=(detection_node,), daemon=True)
    ros_thread.start()
    print("[ROS2] Detection subscriber started in background")

    try:
        while True:

            listen_wakeword()

            print("[GREETING] Tôi đây")
            speak("Tôi đây")

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                record_audio(tmp.name, duration=10)
                text = transcribe(tmp.name)

            print(f"[STT RAW] '{text}' (len={len(text)})")

            if text.strip() in ["", ".", "..", "..."]:
                print("[NOTICE] Không nghe thấy gì hợp lệ.")
                speak("Tôi không nghe thấy gì.")
                continue

            response = rag_ask(text)
            print(f"[LLM] {response}")
            speak(response)

            # ============================================================
            #                  CHECK + START MAPPING
            # ============================================================
            if "sẽ khởi động mapping" in response.lower():
                print("[MAPPING] Đang khởi động mapping...")

                try:
                    mapping_process = subprocess.Popen(
                        ["ros2", "launch", "my_robot", "full_mapping.launch.py"],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    print(f"[MAPPING] Process PID: {mapping_process.pid}")

                    # KIỂM TRA TRẠNG THÁI SAU 2 GIÂY
                    import time
                    time.sleep(2)

                    if mapping_process.poll() is None:
                        print("[MAPPING] Mapping đang chạy bình thường.")
                    else:
                        print("[MAPPING] LỖI: mapping crash khi start.")
                        err = mapping_process.stderr.read().decode()
                        print(f"[MAPPING ERROR] {err}")

                except Exception as e:
                    print(f"[MAPPING] Lỗi khi khởi chạy mapping: {e}")

            # ============================================================
            #                  STOP MAPPING
            # ============================================================
            elif "sẽ dừng mapping" in response.lower():
                print("[MAPPING] Đang dừng mapping...")

                try:
                    if mapping_process and mapping_process.poll() is None:
                        mapping_process.send_signal(signal.SIGINT)
                        print("[MAPPING] Gửi SIGINT...")
                        mapping_process.wait(timeout=5)
                        print("[MAPPING] Mapping đã dừng.")
                        mapping_process = None
                    else:
                        print("[MAPPING] Không có mapping nào đang chạy.")
                except subprocess.TimeoutExpired:
                    print("[MAPPING] Timeout — buộc kill()")
                    mapping_process.kill()
                    mapping_process = None
                except Exception as e:
                    print(f"[MAPPING] Lỗi khi dừng mapping: {e}")

            print("Hoàn thành một lệnh — quay lại chế độ chờ wake-word\n")

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Stopping...")
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
