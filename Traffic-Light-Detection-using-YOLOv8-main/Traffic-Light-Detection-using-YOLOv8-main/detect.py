from flask import Flask, request, jsonify
import cv2
import numpy as np
from ultralytics import YOLO
import warnings
import threading
import time

# =========================================================
# FLASK APP
# =========================================================
app = Flask(__name__)

# =========================================================
# LOAD MODEL 1 LẦN
# =========================================================
print("[STARTUP] Loading YOLO model...")

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    model = YOLO("yolo11n.pt")

# Warmup
dummy = np.zeros((320, 320, 3), dtype=np.uint8)
model(dummy, verbose=False)

print("[STARTUP] Model ready!")

# =========================================================
# CONFIG
# =========================================================
CONFIDENCE_THRESHOLD = 0.25   
MIN_CONFIDENCE_TO_SEND = 0.20 
TRAFFIC_LIGHT_CLASS_ID = 9   

ENHANCE_FRAME = True
UPSCALE_FRAME = True
UPSCALE_FACTOR = 2.0

# =========================================================
# STATE
# =========================================================
_lock = threading.Lock()
_processing = False
_last_result = {"light": "NONE"}

# =========================================================
# COLOR CLASSIFICATION
# =========================================================
def classify_light_color(roi):
    if roi is None or roi.size == 0:
        return "NONE"

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Đỏ có 2 dải màu trong HSV
    red_mask_1 = cv2.inRange(
        hsv,
        np.array([0, 80, 80]),
        np.array([10, 255, 255])
    )

    red_mask_2 = cv2.inRange(
        hsv,
        np.array([160, 80, 80]),
        np.array([179, 255, 255])
    )

    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    # Xanh lá
    green_mask = cv2.inRange(
        hsv,
        np.array([30, 35, 40]),
        np.array([95, 255, 255])
    )

    # Dải xanh phụ (nhạy với đèn mờ/xa) và ánh sáng yếu
    green_mask_2 = cv2.inRange(
        hsv,
        np.array([20, 20, 20]),
        np.array([85, 200, 200])
    )

    # Gộp hai dải xanh
    green_mask = cv2.bitwise_or(green_mask, green_mask_2)

    red_score = cv2.countNonZero(red_mask)
    green_score = cv2.countNonZero(green_mask)

    if red_score > green_score and red_score > 20:
        return "RED"

    if green_score > red_score and green_score > 15:
        return "GREEN"

    return "NONE"

# =========================================================
# FRAME ENHANCEMENT
# =========================================================
def enhance_frame(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l_channel = lab[:, :, 0]
    
    clahe = cv2.createCLAHE(
        clipLimit=6.0,
        tileGridSize=(8, 8)
    )
    l_channel = clahe.apply(l_channel)
    lab[:, :, 0] = l_channel
    enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    enhanced = cv2.convertScaleAbs(
        enhanced,
        alpha=1.2,
        beta=20
    )

    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] *= 1.8
    hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
    enhanced = cv2.cvtColor(
        hsv.astype(np.uint8),
        cv2.COLOR_HSV2BGR
    )

    return enhanced

# =========================================================
# UPSCALE
# =========================================================
def upscale_frame(frame, factor=2.0):
    h, w = frame.shape[:2]
    return cv2.resize(
        frame,
        (int(w * factor), int(h * factor)),
        interpolation=cv2.INTER_CUBIC
    )

# =========================================================
# DETECT API
# =========================================================
@app.route('/detect', methods=['POST'])
def detect():
    global _processing
    global _last_result

    print("[DEBUG] ESP32 request received")

    # Nếu server đang bận → trả kết quả cũ
    if _processing:
        return jsonify(_last_result)

    jpg_bytes = request.data

    if not jpg_bytes:
        return jsonify({"light": "NONE"})

    img = cv2.imdecode(
        np.frombuffer(jpg_bytes, np.uint8),
        cv2.IMREAD_COLOR
    )

    cv2.imwrite("debug_original.jpg", img) 

    if img is None:
        return jsonify({"light": "NONE"})

    original = img.copy()

    # =====================================================
    # Enhance
    # =====================================================
    if ENHANCE_FRAME:
        img = enhance_frame(img)

    # =====================================================
    # Resize nhỏ trước để tăng tốc
    # =====================================================
    img = cv2.resize(img, (320, 320))

    # =====================================================
    # Upscale để detect traffic light nhỏ
    # =====================================================
    if UPSCALE_FRAME:
        img_detect = upscale_frame(img, UPSCALE_FACTOR)
    else:
        img_detect = img

    with _lock:
        _processing = True

        try:
            results = model(
                img_detect,
                verbose=False,
                conf=CONFIDENCE_THRESHOLD,
                iou=0.45
            )

            result = results[0]

            if result.boxes is None or len(result.boxes) == 0:
                final_result = {"light": "GREEN"}
            else:
                boxes = result.boxes
                valid_lights = []

                # Lọc qua tất cả các box tìm được
                for i in range(len(boxes)):
                    box_class = int(boxes.cls[i].item())
                    box_conf = boxes.conf[i].item()

                    # CHỈ LẤY: Đèn giao thông (Class 9) VÀ Độ tin cậy >= Ngưỡng cài đặt (60%)
                    if box_class == TRAFFIC_LIGHT_CLASS_ID and box_conf >= MIN_CONFIDENCE_TO_SEND:
                        valid_lights.append({
                            "conf": box_conf,
                            "xyxy": boxes.xyxy[i]
                        })

                # Nếu không có đèn giao thông nào đủ điều kiện -> Trả về GREEN
                if len(valid_lights) == 0:
                    final_result = {"light": "GREEN"}
                else:
                    # Nếu có nhiều đèn thỏa mãn, chọn box có độ tin cậy (confidence) cao nhất
                    valid_lights.sort(key=lambda x: x["conf"], reverse=True)
                    best_box = valid_lights[0]["xyxy"][0]

                    x1, y1, x2, y2 = map(int, best_box)

                    # Scale ngược lại toạ độ nếu đã dùng UPSCALE
                    if UPSCALE_FRAME:
                        x1 = int(x1 / UPSCALE_FACTOR)
                        y1 = int(y1 / UPSCALE_FACTOR)
                        x2 = int(x2 / UPSCALE_FACTOR)
                        y2 = int(y2 / UPSCALE_FACTOR)

                    # Đảm bảo toạ độ không bị tràn viền bức ảnh
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(img.shape[1], x2)
                    y2 = min(img.shape[0], y2)

                    roi = img[y1:y2, x1:x2]

                    # Gửi ROI sang hàm classify màu
                    light = classify_light_color(roi)

                    final_result = {
                        "light": light
                    }

            _last_result = final_result
            print(f"[RESULT] {_last_result}")

            return jsonify(final_result)

        except Exception as e:
            print("[ERROR]", e)
            return jsonify({"light": "NONE"})

        finally:
            _processing = False

# =========================================================
# MAIN
# =========================================================
if __name__ == '__main__':
    app.run(
        host='0.0.0.0',
        port=5000,
        threaded=True,
        debug=False
    )