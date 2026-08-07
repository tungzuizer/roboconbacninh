"""
Chạy trên Raspberry Pi 5 để nhận dạng 4 pallet trên một kệ 2x2.

Dùng 2 CP2102:
    CP2102 #1: USB cắm vào Control Hub
    CP2102 #2: USB cắm vào Pi → /dev/ttyUSB0
    Nối dây: TX↔RX, GND↔GND giữa 2 con CP2102

Control Hub → Pi:
    SCAN\n  : chụp 5 ảnh, majority vote và trả về vị trí hộp
    STOP\n  : hủy lượt quét hiện tại

Pi → Control Hub khi thành công (gửi từng dòng giống uart_test_pi):
    PICK,<slotId>,<col>,<row>\n   (lặp cho mỗi box)
    END\n

    slotId = 1..4 (box01..box04)
    col = cột trên kệ (1=trái, 2=phải)
    row = hàng trên kệ (1=trên, 2=dưới)

Ví dụ:
    PICK,1,1,1
    PICK,2,1,2
    PICK,3,2,1
    PICK,4,2,2
    END

Pi → Control Hub khi lỗi:
    ERROR,<reason>[,pallet1,pallet2,...]\n
    END\n

Cài đặt tối thiểu:
    sudo apt update && sudo apt install -y python3-picamera2
    pip install onnxruntime numpy opencv-python-headless pyserial

Cấu trúc thư mục gợi ý:
    /home/pi/robocon_ai/
    ├── pi_yolo_serial_onnx.py
    └── models/
        └── best.onnx
"""

import argparse
import time
import threading
from collections import Counter
from pathlib import Path
from typing import List, Dict, Tuple, Optional

import cv2
import numpy as np
import onnxruntime as ort
import serial
from picamera2 import Picamera2


# =========================
# CẤU HÌNH SERIAL
# =========================

# CP2102 #2 cắm USB vào Pi → /dev/ttyUSB0
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200


# =========================
# CẤU HÌNH MODEL
# =========================

BASE_DIR = Path(_file_).resolve().parent
MODEL_PATH = str(BASE_DIR / "models" / "best.onnx")

CLASS_NAMES = [
    "box01",
    "box02",
    "box03",
    "box04",
]

CONF_THRESHOLD = 0.40
IOU_THRESHOLD = 0.45


# =========================
# CẤU HÌNH CAMERA / INPUT
# =========================

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

MODEL_SIZE = 640

# Camera đứng cố định trước một kệ gồm 2 cột x 2 hàng.
GRID_SPLIT_X_NORM = 0.50
GRID_SPLIT_Y_NORM = 0.50

# True: hàng 1 ở trên, hàng 2 ở dưới.
ROW_1_IS_TOP = True

# Chụp đúng 15 ảnh; phải đồng nhất trên 50% số ảnh mới đưa ra kết luận.
VOTE_FRAME_COUNT = 15
MIN_MAJORITY_VOTES = VOTE_FRAME_COUNT // 2 + 1
FRAME_INTERVAL_S = 0.08
CAMERA_WARMUP_S = 0.40

SCAN_SAFETY_TIMEOUT_S = 15.0

GRID_CELLS = [
    (1, 1),
    (2, 1),
    (1, 2),
    (2, 2),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Nhận dạng pallet và gửi kết quả qua USB serial."
    )
    parser.add_argument(
        "--terminal-test",
        action="store_true",
        help="Tự quét một lần bằng camera, in kết quả ra terminal, không mở serial.",
    )
    return parser.parse_args()


ARGS = parse_args()
TERMINAL_TEST = ARGS.terminal_test


# =========================
# KHỞI TẠO MODEL ONNX
# =========================

print("Đang load ONNX model...")
session = ort.InferenceSession(
    MODEL_PATH,
    providers=["CPUExecutionProvider"]
)

input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
output_shape = session.get_outputs()[0].shape

model_input_height = input_shape[2]
model_input_width = input_shape[3]
if (
        isinstance(model_input_height, int)
        and isinstance(model_input_width, int)
        and model_input_height > 0
        and model_input_width > 0
):
    if model_input_height != model_input_width:
        raise ValueError(
            f"Model ONNX chỉ hỗ trợ input vuông, nhận được {input_shape}"
        )
    MODEL_SIZE = model_input_height

print("ONNX input name:", input_name)
print("ONNX input shape:", input_shape)
print("ONNX output shape:", output_shape)
print("Kích thước inference:", f"{MODEL_SIZE}x{MODEL_SIZE}")


# =========================
# KHỞI TẠO CAMERA
# =========================

print("Đang khởi tạo camera...")
picam2 = Picamera2()
_sensor_size = picam2.camera_properties.get("PixelArraySize", (FRAME_WIDTH, FRAME_HEIGHT))
picam2.configure(
    picam2.create_video_configuration(
        main={
            "size": (FRAME_WIDTH, FRAME_HEIGHT),
            "format": "RGB888",
        },
        raw={"size": _sensor_size},
        buffer_count=4,
    )
)
picam2.set_controls({
    "ScalerCrop": [0, 0, _sensor_size[0], _sensor_size[1]]
})


# =========================
# KHỞI TẠO SERIAL (giống uart_test_pi.py)
# =========================

ser: Optional[serial.Serial] = None
if TERMINAL_TEST:
    print("TERMINAL TEST: không mở serial, kết quả sẽ in trực tiếp ra terminal.")
else:
    print(f"Mở serial {SERIAL_PORT} @ {BAUD_RATE}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
        print(f"✅ Kết nối {SERIAL_PORT} thành công")
    except serial.SerialException as e:
        print(f"❌ Không mở được {SERIAL_PORT}: {e}")
        print("   Kiểm tra: ls /dev/ttyUSB*")
        raise


# =========================
# TRẠNG THÁI QUÉT
# =========================

scanning = threading.Event()
stop_requested = threading.Event()


def serial_command_listener() -> None:
    """Thread đọc lệnh từ Control Hub (giống reader_thread trong uart_test_pi)."""
    if ser is None:
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            print(f"  [NHẬN từ Hub] {line}")

            if line == "SCAN":
                if scanning.is_set():
                    print(">> Đang quét, bỏ qua lệnh SCAN lặp")
                else:
                    print(">> Nhận lệnh SCAN")
                    stop_requested.clear()
                    scanning.set()

            elif line == "STOP":
                print(">> Nhận lệnh STOP")
                stop_requested.set()
                scanning.clear()

        except Exception as e:
            print(f"Lỗi đọc serial: {e}")
            time.sleep(0.5)


def letterbox(
        image: np.ndarray,
        new_shape: int = 640,
        color: Tuple[int, int, int] = (114, 114, 114),
) -> Tuple[np.ndarray, float, int, int]:
    """Resize ảnh về new_shape x new_shape nhưng giữ tỷ lệ, padding phần thiếu."""
    h, w = image.shape[:2]
    scale = min(new_shape / w, new_shape / h)
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))

    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_w = new_shape - new_w
    pad_h = new_shape - new_h
    pad_x = pad_w // 2
    pad_y = pad_h // 2
    right = pad_w - pad_x
    bottom = pad_h - pad_y

    padded = cv2.copyMakeBorder(
        resized, pad_y, bottom, pad_x, right,
        cv2.BORDER_CONSTANT, value=color,
    )
    return padded, scale, pad_x, pad_y


def preprocess(frame: np.ndarray) -> Tuple[np.ndarray, float, int, int]:
    """
    Picamera2 format RGB888 trả mảng BGR (thứ tự OpenCV).
    YOLO cần RGB → chuyển BGR→RGB.
    Output: tensor [1, 3, H, W] float32 [0,1].
    """
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img, scale, pad_x, pad_y = letterbox(frame_rgb, MODEL_SIZE)

    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    return img, scale, pad_x, pad_y


def decode_yolo_output(
        output: np.ndarray,
        scale: float,
        pad_x: int,
        pad_y: int,
) -> List[Dict]:
    """Decode output YOLO ONNX. Hỗ trợ cả end-to-end [1,300,6] và truyền thống [1,8,8400]."""
    pred = output
    if pred.ndim == 3:
        pred = pred[0]
    if pred.ndim != 2:
        raise ValueError(f"Output ONNX không hỗ trợ: shape={output.shape}")

    # YOLO end-to-end: [x1, y1, x2, y2, conf, class_id]
    if pred.shape[1] == 6:
        detections = []
        for row in pred:
            x1, y1, x2, y2, conf, cls_value = row
            conf = float(conf)
            cls_id = int(round(float(cls_value)))

            if conf < CONF_THRESHOLD:
                continue
            if cls_id < 0 or cls_id >= len(CLASS_NAMES):
                continue

            x1 = (float(x1) - pad_x) / scale
            y1 = (float(y1) - pad_y) / scale
            x2 = (float(x2) - pad_x) / scale
            y2 = (float(y2) - pad_y) / scale

            x1 = max(0.0, min(FRAME_WIDTH - 1.0, x1))
            y1 = max(0.0, min(FRAME_HEIGHT - 1.0, y1))
            x2 = max(0.0, min(FRAME_WIDTH - 1.0, x2))
            y2 = max(0.0, min(FRAME_HEIGHT - 1.0, y2))

            if x2 - x1 <= 1 or y2 - y1 <= 1:
                continue

            detections.append({
                "class_id": cls_id,
                "class_name": CLASS_NAMES[cls_id],
                "conf": conf,
                "xyxy": [int(x1), int(y1), int(x2), int(y2)],
            })
        return detections

    # YOLO truyền thống: [cx, cy, w, h, score_cls0, score_cls1, ...]
    if pred.shape[0] < pred.shape[1]:
        pred = pred.T

    boxes_xywh = []
    scores = []
    class_ids = []

    for row in pred:
        if len(row) < 4 + len(CLASS_NAMES):
            continue
        cx, cy, w, h = row[0:4]
        class_scores = row[4:4 + len(CLASS_NAMES)]
        cls_id = int(np.argmax(class_scores))
        conf = float(class_scores[cls_id])

        if conf < CONF_THRESHOLD:
            continue

        x1 = (cx - w / 2.0 - pad_x) / scale
        y1 = (cy - h / 2.0 - pad_y) / scale
        x2 = (cx + w / 2.0 - pad_x) / scale
        y2 = (cy + h / 2.0 - pad_y) / scale

        x1 = max(0.0, min(FRAME_WIDTH - 1.0, x1))
        y1 = max(0.0, min(FRAME_HEIGHT - 1.0, y1))
        x2 = max(0.0, min(FRAME_WIDTH - 1.0, x2))
        y2 = max(0.0, min(FRAME_HEIGHT - 1.0, y2))

        bw = x2 - x1
        bh = y2 - y1
        if bw <= 1 or bh <= 1:
            continue

        boxes_xywh.append([int(x1), int(y1), int(bw), int(bh)])
        scores.append(conf)
        class_ids.append(cls_id)

    detections = []
    if len(boxes_xywh) == 0:
        return detections

    indices = cv2.dnn.NMSBoxes(
        bboxes=boxes_xywh,
        scores=scores,
        score_threshold=CONF_THRESHOLD,
        nms_threshold=IOU_THRESHOLD,
    )

    if len(indices) == 0:
        return detections

    for i in indices.flatten():
        x, y, w, h = boxes_xywh[i]
        detections.append({
            "class_id": class_ids[i],
            "class_name": CLASS_NAMES[class_ids[i]],
            "conf": scores[i],
            "xyxy": [x, y, x + w, y + h],
        })
    return detections


def run_onnx(frame: np.ndarray) -> List[Dict]:
    """Chạy ONNX inference cho 1 frame."""
    input_tensor, scale, pad_x, pad_y = preprocess(frame)
    outputs = session.run(None, {input_name: input_tensor})
    return decode_yolo_output(outputs[0], scale, pad_x, pad_y)


def detection_to_cell(detection: Dict) -> Tuple[int, int]:
    """Đổi tâm bounding box thành (cột, hàng) trên kệ 2x2."""
    x1, y1, x2, y2 = detection["xyxy"]
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0

    split_x = FRAME_WIDTH * GRID_SPLIT_X_NORM
    split_y = FRAME_HEIGHT * GRID_SPLIT_Y_NORM

    column = 1 if center_x < split_x else 2
    is_top = center_y < split_y
    row = 1 if is_top == ROW_1_IS_TOP else 2
    return column, row


def best_detection_per_class(detections: List[Dict]) -> Dict[int, Dict]:
    """Mỗi class chỉ giữ box có confidence cao nhất."""
    best: Dict[int, Dict] = {}
    for detection in detections:
        class_id = int(detection["class_id"])
        previous = best.get(class_id)
        if previous is None or detection["conf"] > previous["conf"]:
            best[class_id] = detection
    return best


def resolve_majority_positions(
        votes: Dict[int, Counter],
        confidence_sums: Dict[int, Counter],
) -> Tuple[Optional[Dict[int, Tuple[int, int]]], List[int]]:
    """
    Chọn vị trí cho mỗi pallet dựa trên majority vote.
    Mỗi pallet chọn ô có nhiều phiếu nhất (>= MIN_MAJORITY_VOTES).
    """
    class_ids = list(range(len(CLASS_NAMES)))
    positions: Dict[int, Tuple[int, int]] = {}
    uncertain_pallets: List[int] = []

    for class_id in class_ids:
        if not votes[class_id]:
            uncertain_pallets.append(class_id + 1)
            continue

        best_cell = votes[class_id].most_common(1)[0]
        cell, count = best_cell

        if count >= MIN_MAJORITY_VOTES:
            positions[class_id] = cell
        else:
            uncertain_pallets.append(class_id + 1)

    if uncertain_pallets:
        return None, uncertain_pallets

    return positions, []


def scan_rack_positions() -> Tuple[Optional[Dict[int, Tuple[int, int]]], List[int]]:
    """Chụp 5 ảnh và majority vote vị trí của pallet 1..4."""
    votes: Dict[int, Counter] = {
        class_id: Counter() for class_id in range(len(CLASS_NAMES))
    }
    confidence_sums: Dict[int, Counter] = {
        class_id: Counter() for class_id in range(len(CLASS_NAMES))
    }
    started_at = time.monotonic()

    for frame_number in range(1, VOTE_FRAME_COUNT + 1):
        if stop_requested.is_set():
            return None, []

        if time.monotonic() - started_at > SCAN_SAFETY_TIMEOUT_S:
            raise TimeoutError("SCAN vượt quá thời gian an toàn")

        frame = picam2.capture_array()
        detections = run_onnx(frame)
        best_by_class = best_detection_per_class(detections)
        frame_result = []

        for class_id, detection in best_by_class.items():
            cell = detection_to_cell(detection)
            confidence = float(detection["conf"])
            votes[class_id][cell] += 1
            confidence_sums[class_id][cell] += confidence
            frame_result.append(
                f"P{class_id + 1}=C{cell[0]}R{cell[1]}({confidence:.2f})"
            )

        print(
            f">> Ảnh {frame_number}/{VOTE_FRAME_COUNT}: "
            + (", ".join(frame_result) if frame_result else "không có detection")
        )

        if frame_number < VOTE_FRAME_COUNT:
            time.sleep(FRAME_INTERVAL_S)

    return resolve_majority_positions(votes, confidence_sums)


# =========================
# GỬI DỮ LIỆU (giống uart_test_pi.py)
# =========================

def send_line(text: str) -> None:
    """Gửi 1 dòng qua serial (giống cách uart_test_pi gửi)."""
    msg = text + "\n"
    print(f"  [GỬI đến Hub] {text}")
    if ser is not None:
        ser.write(msg.encode("utf-8"))
        ser.flush()


def send_pick_order(positions: Dict[int, Tuple[int, int]]) -> None:
    """
    Gửi vị trí từng hộp sang Control Hub.

    Format giống uart_test_pi:
        PICK,<slotId>,<col>,<row>\n   (lặp cho mỗi box)
        END\n

    Ví dụ:
        PICK,1,1,1
        PICK,2,1,2
        PICK,3,2,1
        PICK,4,2,2
        END
    """
    for cls_id in sorted(positions.keys()):
        col, row = positions[cls_id]
        slot_id = cls_id + 1  # class_id 0→slotId 1, ...
        send_line(f"PICK,{slot_id},{col},{row}")
        time.sleep(0.05)  # delay nhỏ giữa các dòng, giống uart_test_pi

    send_line("END")


def send_error(reason: str, pallet_numbers: Optional[List[int]] = None) -> None:
    """Gửi dòng lỗi sang Control Hub."""
    fields = ["ERROR", reason]
    if pallet_numbers:
        fields.extend(str(n) for n in pallet_numbers)

    send_line(",".join(fields))
    send_line("END")


# =========================
# TERMINAL TEST
# =========================

def run_terminal_test() -> None:
    """Chạy một lượt scan từ terminal, không cần Control Hub."""
    camera_running = False

    try:
        print(">> TERMINAL TEST: bật camera, chụp 5 ảnh...")
        picam2.start()
        camera_running = True
        time.sleep(CAMERA_WARMUP_S)

        positions, uncertain_pallets = scan_rack_positions()

        if positions is None:
            print(">> Không đủ majority vote:", uncertain_pallets)
            send_error("NO_MAJORITY", uncertain_pallets)
        else:
            print(">> Kết quả:")
            send_pick_order(positions)

    except TimeoutError as error:
        print(">>", error)
        send_error("TIMEOUT")
    except Exception as error:
        print("Lỗi:", error)
        send_error("SCAN_FAILED")
    finally:
        if camera_running:
            picam2.stop()


# =========================
# MAIN
# =========================

def main() -> None:
    if TERMINAL_TEST:
        run_terminal_test()
        return

    # Thread đọc lệnh từ Control Hub (giống reader_thread trong uart_test_pi)
    listener = threading.Thread(target=serial_command_listener, daemon=True)
    listener.start()

    print()
    print("═══════════════════════════════════════")
    print("  Sẵn sàng. Đang chờ lệnh từ Control Hub...")
    print("  Control Hub gửi SCAN → Pi chụp ảnh → gửi PICK kết quả")
    print("═══════════════════════════════════════")
    print()

    while True:
        if not scanning.wait(timeout=0.05):
            continue

        camera_running = False

        try:
            print(">> Bật camera, chụp 5 ảnh...")
            picam2.start()
            camera_running = True
            time.sleep(CAMERA_WARMUP_S)

            positions, uncertain_pallets = scan_rack_positions()

            if stop_requested.is_set():
                print(">> Lượt quét bị hủy bởi STOP")
            elif positions is None:
                print(">> Không đủ majority vote:", uncertain_pallets)
                send_error("NO_MAJORITY", uncertain_pallets)
            else:
                send_pick_order(positions)

        except TimeoutError as error:
            print(">>", error)
            send_error("TIMEOUT")
        except Exception as error:
            print("Lỗi:", error)
            send_error("SCAN_FAILED")
        finally:
            scanning.clear()
            if camera_running:
                print(">> Tắt camera")
                picam2.stop()


if _name_ == "_main_":
    try:
        main()
    except KeyboardInterrupt:
        print("Dừng bằng Ctrl+C")
    finally:
        try:
            if picam2.started:
                picam2.stop()
        except Exception:
            pass

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        print("Đã đóng camera/serial.")