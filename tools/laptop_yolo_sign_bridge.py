#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO


DEFAULT_MODEL = (
    r"C:\Users\George\Desktop\_\RTK_KUBOK_MOSCOW"
    r"\gorod_znaki_01_04_2026\my_yolov8n_run\weights\best.pt"
)
DEFAULT_STREAM = "http://192.168.1.145:8080/stream.mjpg"
DEFAULT_ROBOT_HOST = "192.168.1.145"
DEFAULT_ROBOT_PORT = 5005


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run YOLO on the laptop camera stream and send detections to the robot over UDP."
    )
    parser.add_argument("--stream-url", default=DEFAULT_STREAM)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--robot-host", default=DEFAULT_ROBOT_HOST)
    parser.add_argument("--robot-port", type=int, default=DEFAULT_ROBOT_PORT)
    parser.add_argument("--conf", type=float, default=0.6)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--max-fps", type=float, default=5.0)
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N processed frames; 0 means run forever.")
    parser.add_argument("--show", action="store_true", help="Show annotated preview window on the laptop.")
    parser.add_argument("--print-json", action="store_true", help="Print every JSON payload.")
    return parser.parse_args()


def open_stream(url):
    cap = cv2.VideoCapture(url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open video stream: {url}")
    return cap


def box_to_detection(box, names):
    cls_id = int(box.cls[0])
    cls_name = str(names.get(cls_id, cls_id))
    conf = float(box.conf[0])
    x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    return {
        "class_id": cls_id,
        "class_name": cls_name,
        "confidence": round(conf, 4),
        "bbox_xyxy": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
        "center_xy": [round(x1 + width / 2.0, 1), round(y1 + height / 2.0, 1)],
        "area_px": round(width * height, 1),
    }


def draw_detections(frame, detections):
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det["bbox_xyxy"]]
        cls_name = det["class_name"].lower()
        if cls_name == "blue_robot":
            color = (255, 0, 0)
        elif cls_name == "red_robot":
            color = (0, 0, 255)
        else:
            color = (0, 255, 0)
        label = f"{det['class_name']} {det['confidence']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


def make_payload(seq, frame, detections, inference_ms, stream_url):
    height, width = frame.shape[:2]
    best = max(detections, key=lambda item: item["confidence"], default=None)
    return {
        "schema": "tb3_yolo_detections.v1",
        "seq": seq,
        "stamp_unix": time.time(),
        "source": stream_url,
        "frame": {"width": width, "height": height},
        "inference_ms": round(inference_ms, 1),
        "best_detection": best,
        "detections": detections,
    }


def main():
    args = parse_args()
    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(model_path)

    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Loading YOLO model: {model_path}")
    print(f"Inference device: {device}")
    model = YOLO(str(model_path))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.robot_host, args.robot_port)
    min_period = 1.0 / args.max_fps if args.max_fps > 0 else 0.0

    seq = 0
    last_process = 0.0
    cap = None
    print(f"Reading stream: {args.stream_url}")
    print(f"Sending JSON detections to udp://{args.robot_host}:{args.robot_port}")

    while True:
        try:
            if cap is None:
                cap = open_stream(args.stream_url)

            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("stream returned no frame")

            now = time.monotonic()
            if min_period and now - last_process < min_period:
                continue
            last_process = now

            started = time.perf_counter()
            results = model.predict(
                source=frame,
                conf=args.conf,
                imgsz=args.imgsz,
                save=False,
                verbose=False,
                device=device,
            )
            inference_ms = (time.perf_counter() - started) * 1000.0

            detections = []
            for result in results:
                names = result.names
                for box in result.boxes:
                    detections.append(box_to_detection(box, names))

            payload = make_payload(seq, frame, detections, inference_ms, args.stream_url)
            data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            sock.sendto(data, target)

            if args.print_json:
                print(data.decode("utf-8"), flush=True)
            else:
                names = ", ".join(d["class_name"] for d in detections) if detections else "none"
                print(f"seq={seq} detections={len(detections)} [{names}] inference={inference_ms:.0f}ms", flush=True)

            if args.show:
                draw_detections(frame, detections)
                cv2.imshow("Laptop YOLO detections", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            seq += 1
            if args.max_frames and seq >= args.max_frames:
                break
        except KeyboardInterrupt:
            break
        except Exception as exc:
            print(f"warning: {exc}; reconnecting in 1s", file=sys.stderr, flush=True)
            if cap is not None:
                cap.release()
                cap = None
            time.sleep(1)

    if cap is not None:
        cap.release()
    if args.show:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
