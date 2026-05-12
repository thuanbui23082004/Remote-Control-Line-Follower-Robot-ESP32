import cv2
import numpy as np
from ultralytics import YOLO
import warnings

# =========================================================
# CONFIG - Giữ nguyên logic tin cậy như đã thảo luận
# =========================================================
CONFIDENCE_THRESHOLD = 0.25   # Ngưỡng để YOLO bắt vật thể
MIN_CONFIDENCE_TO_SEND = 0.30 # Ngưỡng tin cậy để xác nhận là Đèn giao thông
TRAFFIC_LIGHT_CLASS_ID = 9    # ID của đèn giao thông trong bộ COCO
IOU_THRESHOLD = 0.25

ENHANCE_FRAME = True          # Bật/Tắt tăng cường chất lượng ảnh
UPSCALE_FRAME = True          # Bật/Tắt phóng to vùng nhận diện
UPSCALE_FACTOR = 2.0

# =========================================================
# LOAD MODEL
# =========================================================
print("Đang tải model YOLO...")
model = YOLO("yolo11n.pt") 
print("Model đã sẵn sàng!")

# =========================================================
# HÀM HỖ TRỢ (Giữ nguyên logic từ detect.py)
# =========================================================
def classify_light_color(roi):
    if roi is None or roi.size == 0:
        return "NONE"
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Mask Đỏ
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([160, 80, 80]), np.array([179, 255, 255]))
    )
    # Mask Xanh
    green_mask = cv2.inRange(hsv, np.array([35, 60, 60]), np.array([90, 255, 255]))

    red_score = cv2.countNonZero(red_mask)
    green_score = cv2.countNonZero(green_mask)

    if red_score > green_score and red_score > 25: return "RED"
    if green_score > red_score and green_score > 25: return "GREEN"
    return "NONE"

def enhance_frame(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    enhanced = cv2.merge((cl,a,b))
    enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
    return enhanced

# =========================================================
# VÒNG LẶP XỬ LÝ CAMERA
# =========================================================
cap = cv2.VideoCapture(0) # Mở Webcam (số 0 là camera mặc định)

if not cap.isOpened():
    print("Không thể mở Camera!")
    exit()

print("Bắt đầu Test. Nhấn 'q' để thoát.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 1. Tiền xử lý (Tùy chọn)
    display_frame = frame.copy()
    processing_frame = enhance_frame(frame) if ENHANCE_FRAME else frame
    processing_frame = cv2.resize(processing_frame, (640, 480)) # Resize để chạy mượt hơn trên PC

    # 2. Nhận diện YOLO
    results = model(processing_frame, conf=CONFIDENCE_THRESHOLD, iou=IOU_THRESHOLD, verbose=False)
    result = results[0]

    if result.boxes is not None:
        boxes = result.boxes
        for i in range(len(boxes)):
            box_class = int(boxes.cls[i].item())
            box_conf = boxes.conf[i].item()

            # CHỈ LỌC ĐÈN GIAO THÔNG VÀ TIN CẬY CAO
            if box_class == TRAFFIC_LIGHT_CLASS_ID and box_conf >= MIN_CONFIDENCE_TO_SEND:
                x1, y1, x2, y2 = map(int, boxes.xyxy[i])
                
                # Cắt ROI để soi màu
                roi = processing_frame[y1:y2, x1:x2]
                color = classify_light_color(roi)

                # Vẽ lên màn hình để quan sát
                label_color = (0, 255, 0) if color == "GREEN" else (0, 0, 255) if color == "RED" else (255, 255, 255)
                
                # Vẽ khung
                cv2.rectangle(processing_frame, (x1, y1), (x2, y2), label_color, 2)
                # Ghi text (Màu + Độ tin cậy)
                text = f"{color} ({box_conf:.2f})"
                cv2.putText(processing_frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, label_color, 2)

    # Hiển thị kết quả
    cv2.imshow("Test AI Traffic Light - Webcam", processing_frame)

    # Thoát khi nhấn phím 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()