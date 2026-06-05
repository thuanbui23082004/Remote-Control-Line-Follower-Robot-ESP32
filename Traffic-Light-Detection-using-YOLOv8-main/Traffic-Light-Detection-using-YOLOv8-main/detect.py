"""
detect.py — Fixed & Optimized for ESP32-CAM + YOLO Traffic Light Detection
============================================================================
Fixes applied:
  [FIX-1] Removed premature 320x320 resize — pass original resolution to YOLO
  [FIX-2] Lowered confidence thresholds for real-world ESP32 images
  [FIX-3] Fixed race condition: _processing flag now protected by lock
  [FIX-4] Fixed imwrite before None-check crash
  [FIX-5] enhance_frame is now milder & optional per-ROI (not pre-YOLO)
  [FIX-6] HSV thresholds loosened for ESP32 AWB/overexposure
  [FIX-7] Fixed best_box tensor indexing (shape-safe)
  [FIX-8] ROI debug save added
  [FIX-9] Full verbose logging for production debugging
  [FIX-10] Grayscale / single-channel image guard
  [FIX-11] YOLO imgsz param passed explicitly, no manual resize
  [FIX-12] class ID verified comment + easy override
"""

from flask import Flask, request, jsonify
import cv2
import numpy as np
from ultralytics import YOLO
import warnings
import threading
import time
import os

# =========================================================
# FLASK APP
# =========================================================
app = Flask(__name__)

# =========================================================
# LOAD MODEL
# =========================================================
print("[STARTUP] Loading YOLO model...")

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    model = YOLO("yolo11n.pt")
    # Nếu có custom model: model = YOLO("weights/best.pt")

# Warmup với đúng input size
dummy = np.zeros((640, 640, 3), dtype=np.uint8)
model(dummy, verbose=False)
print("[STARTUP] Model ready!")

# =========================================================
# CONFIG
# =========================================================

# [FIX-2] Hạ xuống để bắt được traffic light qua ESP32-CAM
# ESP32-CAM image quality thấp → YOLO confidence thường chỉ 0.10–0.20
CONFIDENCE_THRESHOLD    = 0.10   # Ngưỡng YOLO filter (thấp để không bỏ sót)
IOU_THRESHOLD           = 0.30
MIN_CONFIDENCE_TO_SEND  = 0.10   # Ngưỡng tối thiểu sau khi filter class

# COCO class 9 = traffic light
# Verify: python -c "from ultralytics import YOLO; m=YOLO('yolo11n.pt'); print(m.names)"
TRAFFIC_LIGHT_CLASS_ID  = 9

# [FIX-1] Không resize thủ công — để YOLO letterbox ảnh gốc
# Chỉ đặt imgsz cho YOLO inference
YOLO_IMGSZ              = 640    # hoặc 1280 nếu muốn detect object nhỏ hơn

# Enhancement: chỉ apply nhẹ, KHÔNG dùng trên ảnh trước YOLO
ENHANCE_ROI_ONLY        = True   # True = enhance ROI trước khi classify màu
                                  # False = không enhance gì cả

# Debug output
DEBUG_SAVE              = True   # Lưu ảnh debug ra disk
DEBUG_DIR               = "debug_frames"

# =========================================================
# STATE — Thread-safe
# =========================================================
_lock           = threading.Lock()
_processing     = False
_last_result    = {"traffic_light": "NONE"}

os.makedirs(DEBUG_DIR, exist_ok=True)

# =========================================================
# HELPER: Verify image channel
# =========================================================
def ensure_bgr(img: np.ndarray) -> np.ndarray:
    """
    [FIX-10] ESP32 đôi khi gửi grayscale JPEG.
    Đảm bảo output luôn là 3-channel BGR.
    """
    if img is None:
        return None
    if len(img.shape) == 2:
        # Grayscale → BGR
        print("[WARN] Received grayscale image, converting to BGR")
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        # BGRA → BGR
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img

# =========================================================
# COLOR CLASSIFICATION (HSV)
# =========================================================
def classify_light_color(roi: np.ndarray) -> str:
    """
    Phân loại màu đèn từ ROI (BGR).
    [FIX-6] HSV threshold nới rộng cho ESP32 AWB + overexposure.
    """
    if roi is None or roi.size == 0:
        print("[CLASSIFY] Empty ROI")
        return "NONE"

    if roi.shape[0] < 3 or roi.shape[1] < 3:
        print(f"[CLASSIFY] ROI quá nhỏ: {roi.shape}")
        return "NONE"

    # [FIX-5] Enhance nhẹ chỉ trên ROI nếu bật
    if ENHANCE_ROI_ONLY:
        roi = _mild_enhance(roi)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # ── ĐỎ ──────────────────────────────────────────────
    # [FIX-6] Hạ saturation threshold xuống 50 (ESP32 AWB làm mờ màu)
    # [FIX-6] Hạ value threshold xuống 50 (overexpose làm V cao bất thường)
    red_lo1 = np.array([0,   50, 50])
    red_hi1 = np.array([12, 255, 255])
    red_lo2 = np.array([155, 50, 50])
    red_hi2 = np.array([179, 255, 255])
    red_mask = (
        cv2.inRange(hsv, red_lo1, red_hi1) |
        cv2.inRange(hsv, red_lo2, red_hi2)
    )

    # ── XANH LÁ ─────────────────────────────────────────
    green_lo = np.array([30, 15, 120])   # S thấp hơn, V cao (vì đang chói sáng)
    green_hi = np.array([100, 255, 255])
    green_mask = cv2.inRange(hsv, green_lo, green_hi)

    # ── VÀNG (bổ sung để không nhầm đỏ/xanh) ───────────
    yellow_lo = np.array([15, 80, 80])
    yellow_hi = np.array([35, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lo, yellow_hi)

    total_pixels = roi.shape[0] * roi.shape[1]
    red_score    = cv2.countNonZero(red_mask)
    green_score  = cv2.countNonZero(green_mask)
    yellow_score = cv2.countNonZero(yellow_mask)

    # Ngưỡng tối thiểu: 5% pixel ROI phải match
    min_threshold = max(10, int(total_pixels * 0.05))

    print(f"[CLASSIFY] ROI={roi.shape} | "
          f"red={red_score} green={green_score} yellow={yellow_score} "
          f"min={min_threshold}")

    scores = {"RED": red_score, "GREEN": green_score, "YELLOW": yellow_score}
    best_color = max(scores, key=scores.get)
    best_score = scores[best_color]

    if best_score >= min_threshold:
        return best_color

    return "NONE"


def _mild_enhance(roi: np.ndarray) -> np.ndarray:
    """
    [FIX-5] Enhancement nhẹ, chỉ dùng trên ROI nhỏ trước khi classify màu.
    KHÔNG dùng trên toàn frame trước YOLO (sẽ phá màu).
    """
    try:
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        l = lab[:, :, 0]
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))  # Nhẹ hơn nhiều
        lab[:, :, 0] = clahe.apply(l)
        enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        # Tăng brightness nhẹ
        enhanced = cv2.convertScaleAbs(enhanced, alpha=1.1, beta=10)
        return enhanced
    except Exception as e:
        print(f"[ENHANCE] Failed: {e}")
        return roi

# =========================================================
# DETECT ENDPOINT
# =========================================================
@app.route('/detect', methods=['POST'])
def detect():
    global _processing, _last_result

    req_time = time.time()
    print(f"\n[REQUEST] Received at {req_time:.3f}")

    # [FIX-3] Race condition fix: check _processing TRONG lock
    with _lock:
        if _processing:
            print("[SKIP] Server busy, returning cached result")
            return jsonify(_last_result)
        _processing = True

    try:
        return _do_detect()
    finally:
        with _lock:
            _processing = False

def _do_detect():
    global _last_result

    # ── 1. ĐỌC RAW BYTES ────────────────────────────────
    jpg_bytes = request.data
    content_len = len(jpg_bytes) if jpg_bytes else 0
    print(f"[RECV] Content-Length: {content_len} bytes")

    if not jpg_bytes or content_len < 100:
        print("[ERROR] Empty or too-small payload")
        return jsonify({"traffic_light": "NONE", "error": "empty_payload"})

    # ── 2. DECODE JPEG ───────────────────────────────────
    buf = np.frombuffer(jpg_bytes, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)

    # [FIX-4] Check None TRƯỚC khi imwrite
    if img is None:
        print("[ERROR] imdecode failed — JPEG corrupt or wrong format")
        # Dump raw bytes để debug
        with open(os.path.join(DEBUG_DIR, "failed_raw.bin"), "wb") as f:
            f.write(jpg_bytes)
        return jsonify({"traffic_light": "NONE","error": "decode_failed"})

    # [FIX-10] Đảm bảo 3-channel BGR
    img = ensure_bgr(img)

    print(f"[DECODE] Shape: {img.shape}, dtype: {img.dtype}, "
          f"min/max: {img.min()}/{img.max()}")

    # Debug save ảnh gốc
    if DEBUG_SAVE:
        debug_path = os.path.join(DEBUG_DIR, "original.jpg")
        cv2.imwrite(debug_path, img)
        print(f"[DEBUG] Saved original → {debug_path}")

    # ── 3. BASIC SANITY CHECK ───────────────────────────
    h, w = img.shape[:2]
    if h < 32 or w < 32:
        print(f"[ERROR] Image too small: {w}x{h}")
        return jsonify({"traffic_light": "NONE", "error": "image_too_small"})

    # Check if image is all-black (ESP32 power issue)
    mean_brightness = np.mean(img)
    if mean_brightness < 5:
        print(f"[ERROR] Image too dark (mean={mean_brightness:.1f}) — ESP32 issue?")
        return jsonify({"traffic_light": "NONE", "error": "image_too_dark"})

    # ── 4. YOLO INFERENCE ───────────────────────────────
    # [FIX-1] KHÔNG resize thủ công. YOLO tự letterbox.
    # [FIX-11] Truyền imgsz trực tiếp vào model()
    print(f"[YOLO] Running inference on {w}x{h} image (imgsz={YOLO_IMGSZ})")

    results = model(
        img,
        verbose=False,
        conf=CONFIDENCE_THRESHOLD,
        iou=IOU_THRESHOLD,
        imgsz=YOLO_IMGSZ,
    )
    result = results[0]

    # ── 5. LOG TẤT CẢ DETECTIONS (debug) ───────────────
    all_boxes = result.boxes
    if all_boxes is None or len(all_boxes) == 0:
        print("[YOLO] No boxes detected at all")
        final_result = {"traffic_light": "NONE"}
        _last_result = final_result
        return jsonify(final_result)

    print(f"[YOLO] Total boxes: {len(all_boxes)}")
    for i in range(len(all_boxes)):
        cls_id  = int(all_boxes.cls[i].item())
        conf    = all_boxes.conf[i].item()
        cls_name = result.names.get(cls_id, f"cls_{cls_id}")
        xyxy    = all_boxes.xyxy[i].cpu().numpy().flatten()
        print(f"  Box[{i}]: class={cls_id}({cls_name}) conf={conf:.3f} "
              f"xyxy=[{xyxy[0]:.0f},{xyxy[1]:.0f},{xyxy[2]:.0f},{xyxy[3]:.0f}]")

    # ── 6. FILTER: CHỈ LẤY TRAFFIC LIGHT ───────────────
    valid_lights = []
    for i in range(len(all_boxes)):
        cls_id = int(all_boxes.cls[i].item())
        conf   = all_boxes.conf[i].item()
        if cls_id == TRAFFIC_LIGHT_CLASS_ID and conf >= MIN_CONFIDENCE_TO_SEND:
            xyxy = all_boxes.xyxy[i].cpu().numpy().flatten()  # [FIX-7] safe flatten
            valid_lights.append({"conf": conf, "xyxy": xyxy})

    if not valid_lights:
        print(f"[FILTER] No traffic light (class {TRAFFIC_LIGHT_CLASS_ID}) "
              f"above conf={MIN_CONFIDENCE_TO_SEND}")
        final_result = {"traffic_light": "NONE"}
        _last_result = final_result
        return jsonify(final_result)

    # ── 7. CHỌN BOX TỐT NHẤT ────────────────────────────
    valid_lights.sort(key=lambda x: x["conf"], reverse=True)
    best = valid_lights[0]
    x1, y1, x2, y2 = map(int, best["xyxy"])  # [FIX-7] đã flatten, không cần [0] thêm

    print(f"[BEST] conf={best['conf']:.3f} box=({x1},{y1},{x2},{y2}) "
          f"on image {w}x{h}")

    # Clamp tọa độ
    x1 = max(0, x1); y1 = max(0, y1)
    x2 = min(w, x2); y2 = min(h, y2)

    # Kiểm tra ROI hợp lệ
    if x2 <= x1 or y2 <= y1:
        print(f"[ERROR] Invalid ROI after clamp: ({x1},{y1})-({x2},{y2})")
        final_result = {"traffic_light": "NONE"}
        _last_result = final_result
        return jsonify(final_result)

    # ── 8. EXTRACT ROI & CLASSIFY ────────────────────────
    roi = img[y1:y2, x1:x2].copy()

    if DEBUG_SAVE:
        roi_path = os.path.join(DEBUG_DIR, "roi.jpg")
        cv2.imwrite(roi_path, roi)
        # Vẽ bounding box lên ảnh full để debug
        debug_img = img.copy()
        cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imwrite(os.path.join(DEBUG_DIR, "annotated.jpg"), debug_img)
        print(f"[DEBUG] Saved ROI → {roi_path}")

    light = classify_light_color(roi)

    # final_result = {
    #     "light": light,
    #     "confidence": round(best["conf"], 3),
    #     "box": [x1, y1, x2, y2],
    # }

    # _last_result = {"light": light}  # Cache chỉ light
    # print(f"[RESULT] ✅ {final_result}")
    # return jsonify(final_result)
    final_result = {
    "traffic_light": light}
    _last_result = final_result
    print(f"[RESULT] ✅ {final_result}")
    return jsonify(final_result)


# =========================================================
# DIAGNOSTIC ENDPOINT
# =========================================================
@app.route('/ping', methods=['GET'])
def ping():
    """Quick health check + model info"""
    return jsonify({
        "status": "ok",
        "model": str(model.model_name if hasattr(model, 'model_name') else "yolo11n"),
        "classes": {str(k): v for k, v in model.names.items()},
        "traffic_light_class": TRAFFIC_LIGHT_CLASS_ID,
        "conf_threshold": CONFIDENCE_THRESHOLD,
    })


@app.route('/debug_last', methods=['GET'])
def debug_last():
    """Trả về kết quả detect cuối cùng"""
    return jsonify(_last_result)


# =========================================================
# MAIN
# =========================================================
if __name__ == '__main__':
    print(f"[INFO] Traffic light class ID = {TRAFFIC_LIGHT_CLASS_ID}")
    print(f"[INFO] Model classes: {model.names}")
    print(f"[INFO] Verify class 9 = '{model.names.get(9, 'NOT FOUND')}'")
    print(f"[INFO] Debug frames → ./{DEBUG_DIR}/")
    app.run(
        host='0.0.0.0',
        port=5000,
        threaded=True,
        debug=False,
    )