#!/usr/bin/env python3
# encoding: utf-8

import cv2 as cv
import time
import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32



class VisionYawPublisher(Node):
    def __init__(self):
        super().__init__('vision_yaw_pub')
        self.pub = self.create_publisher(Float32, '/vision/yaw_error', 10)


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def main():
    # -----------------------------
    # Parameters you care about
    # -----------------------------
    deadband = 0.2          # |ex| <= 0.2 -> ex = 0
    frame_skip = 2          # 每2幀推論一次
    conf_th = 0.7
    imgsz = 320
    iou_th = 0.5
    max_det = 20            # 允許多個人，再挑 bbox 最大者

    # EMA smoothing
    ema_alpha = 0.90        # 越大越平滑、反應越慢（建議 0.85~0.95）
    reset_ema_on_lost = True  # target lost 時是否把 EMA 歸零（建議 True）

    cam_w = 1280
    cam_h = 720
    # -----------------------------

    rclpy.init()
    node = VisionYawPublisher()

    # 先初始化相機,再載入 YOLO
    capture = cv.VideoCapture(
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, "
        "format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=0 ! "
        "video/x-raw, width=1280, height=720, format=(string)BGRx ! videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink",
        cv.CAP_GSTREAMER
    )

    if not capture.isOpened():
        print("Error: Cannot open camera")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("capture get FPS:", capture.get(cv.CAP_PROP_FPS))

    from ultralytics import YOLO

    pkg_share = get_package_share_directory('image_yaw')
    weights_path = os.path.join(pkg_share, 'weights', 'yolov8n.pt')

    model = YOLO(weights_path)

    # 預熱模型
    print("Warming up model...")
    ret, dummy_frame = capture.read()
    if ret:
        _ = model(dummy_frame, imgsz=416, verbose=False)
    print("Model ready!")

    frame_count = 0
    last_results = None

    # 用於「跳幀時沿用上一筆目標」
    last_best_box = None      # (x1,y1,x2,y2)
    last_person_count = 0

    # EMA state
    ex_ema = 0.0
    ema_inited = False
    last_ex_raw = 0.0
    last_ex_out = 0.0

    try:
        while capture.isOpened():
            start = time.time()
            ret, frame = capture.read()
            if not ret:
                break

            frame_count += 1

            do_infer = (frame_count % frame_skip == 0) or (last_results is None)

            if do_infer:
                results = model(
                    frame,
                    classes=[0],
                    conf=conf_th,
                    imgsz=imgsz,
                    iou=iou_th,
                    max_det=max_det,
                    verbose=False
                )
                last_results = results

                boxes = results[0].boxes
                best_area = -1.0
                best_box = None

                person_count = 0
                if boxes is not None and len(boxes) > 0:
                    person_count = len(boxes)
                    xyxy = boxes.xyxy
                    for i in range(len(boxes)):
                        x1, y1, x2, y2 = xyxy[i].tolist()
                        w = max(0.0, x2 - x1)
                        h = max(0.0, y2 - y1)
                        area = w * h
                        if area > best_area:
                            best_area = area
                            best_box = (x1, y1, x2, y2)

                if best_box is not None:
                    x1, y1, x2, y2 = best_box
                    xc = 0.5 * (x1 + x2)

                    # raw ex in [-1,1]
                    ex_raw = (xc - (cam_w * 0.5)) / (cam_w * 0.5)
                    ex_raw = float(clamp(ex_raw, -1.0, 1.0))
                    last_ex_raw = ex_raw

                    # EMA update
                    if not ema_inited:
                        ex_ema = ex_raw
                        ema_inited = True
                    else:
                        ex_ema = (ema_alpha * ex_ema) + ((1.0 - ema_alpha) * ex_raw)

                    # deadband on EMA output
                    ex_out = float(ex_ema)
                    if abs(ex_out) <= deadband:
                        ex_out = 0.0

                    # publish yaw error
                    msg = Float32()
                    msg.data = ex_out
                    node.pub.publish(msg)

                    last_ex_out = ex_out
                    last_best_box = best_box
                    last_person_count = person_count

                else:
                    # 沒偵測到人：不 publish（讓飛控端用 timeout 判斷 target lost）
                    last_best_box = None
                    last_person_count = 0
                    last_ex_raw = 0.0
                    last_ex_out = 0.0
                    if reset_ema_on_lost:
                        ex_ema = 0.0
                        ema_inited = False

            else:
                # 跳幀：沿用上一筆結果（不重新 publish）
                results = last_results

            annotated_frame = results[0].plot() if last_results is not None else frame

            # 畫面中心線
            cv.line(annotated_frame, (cam_w // 2, 0), (cam_w // 2, cam_h), (255, 255, 255), 1)

            # 額外把「最大 bbox 的 target」標出來（紅框）
            if last_best_box is not None:
                x1, y1, x2, y2 = last_best_box
                x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
                cv.rectangle(annotated_frame, (x1i, y1i), (x2i, y2i), (0, 0, 255), 2)
                xc = int(0.5 * (x1 + x2))
                yc = int(0.5 * (y1 + y2))
                cv.circle(annotated_frame, (xc, yc), 4, (0, 0, 255), -1)

                cv.putText(
                    annotated_frame,
                    f"ex_raw={last_ex_raw:.2f} | ex_ema_out={last_ex_out:.2f} | alpha={ema_alpha:.2f} | deadband=+-{deadband}",
                    (20, 60),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 0, 255),
                    2
                )

            # 計算 FPS
            end = time.time()
            fps = 1.0 / max(1e-6, (end - start))

            person_count_show = last_person_count
            text = f"FPS: {int(fps)} | Persons: {person_count_show} | frame_skip={frame_skip}"
            cv.putText(annotated_frame, text, (20, 30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            cv.imshow('YOLOv8 Detection', annotated_frame)

            rclpy.spin_once(node, timeout_sec=0.0)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        capture.release()
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()