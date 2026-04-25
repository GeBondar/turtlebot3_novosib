#!/usr/bin/env python3
import os
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

CAMERA_DEVICE = os.environ.get("CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
CAMERA_FPS = int(os.environ.get("CAMERA_FPS", "30"))
CAMERA_PORT = int(os.environ.get("CAMERA_PORT", "8080"))
CAMERA_MAX_CLIENTS = int(os.environ.get("CAMERA_MAX_CLIENTS", "8"))
FRAME_WAIT_TIMEOUT = float(os.environ.get("FRAME_WAIT_TIMEOUT", "5"))
DETECTIONS_PATH = Path(os.environ.get("DETECTIONS_PATH", "/home/ubuntu/camera_web/latest_detections.json"))
BOUNDARY = "tb3camera"


def ffmpeg_capture_args():
    return [
        "/usr/bin/ffmpeg",
        "-hide_banner",
        "-loglevel", "warning",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-f", "v4l2",
        "-input_format", "mjpeg",
        "-video_size", f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}",
        "-framerate", str(CAMERA_FPS),
        "-i", CAMERA_DEVICE,
        "-an",
        "-c:v", "copy",
        "-f", "mjpeg",
        "pipe:1",
    ]


class SharedMjpegCamera:
    def __init__(self):
        self.condition = threading.Condition()
        self.client_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame = None
        self.frame_id = 0
        self.last_frame_time = 0.0
        self.error = "camera has not produced a frame yet"
        self.proc = None
        self.active_clients = 0
        self.thread = threading.Thread(target=self._run, name="camera-producer", daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self._stop_process()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=3)

    def try_register_client(self):
        with self.client_lock:
            if self.active_clients >= CAMERA_MAX_CLIENTS:
                return False
            self.active_clients += 1
            return True

    def unregister_client(self):
        with self.client_lock:
            self.active_clients = max(0, self.active_clients - 1)

    def stats(self):
        with self.client_lock:
            clients = self.active_clients
        with self.condition:
            age = time.monotonic() - self.last_frame_time if self.last_frame_time else None
            return {
                "clients": clients,
                "frame_id": self.frame_id,
                "last_frame_age": age,
                "error": self.error,
            }

    def wait_for_frame(self, last_seen_id, timeout=FRAME_WAIT_TIMEOUT):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.stop_event.is_set() and self.frame_id == last_seen_id:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(remaining)
            if self.frame is None or self.frame_id == last_seen_id:
                return None, last_seen_id
            return self.frame, self.frame_id

    def _publish(self, frame):
        with self.condition:
            self.frame = frame
            self.frame_id += 1
            self.last_frame_time = time.monotonic()
            self.error = None
            self.condition.notify_all()

    def _set_error(self, message):
        with self.condition:
            self.error = message
            self.condition.notify_all()

    def _run(self):
        while not self.stop_event.is_set():
            cmd = ffmpeg_capture_args()
            print("Starting shared camera capture: " + " ".join(cmd), flush=True)
            try:
                self.proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )
            except OSError as exc:
                self._set_error(f"failed to start ffmpeg: {exc}")
                time.sleep(2)
                continue

            stderr_thread = threading.Thread(
                target=self._drain_stderr,
                args=(self.proc,),
                name="ffmpeg-stderr",
                daemon=True,
            )
            stderr_thread.start()

            try:
                self._read_jpeg_stream(self.proc.stdout)
            finally:
                return_code = self.proc.poll()
                self._stop_process()
                if not self.stop_event.is_set():
                    self._set_error(f"ffmpeg stopped with code {return_code}; restarting")
                    time.sleep(1)

    def _drain_stderr(self, proc):
        if proc.stderr is None:
            return
        for raw_line in iter(proc.stderr.readline, b""):
            line = raw_line.decode("utf-8", "replace").strip()
            if line:
                print(f"ffmpeg: {line}", flush=True)

    def _read_jpeg_stream(self, stdout):
        buffer = bytearray()
        while not self.stop_event.is_set():
            chunk = stdout.read(16384)
            if not chunk:
                break
            buffer.extend(chunk)

            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    if len(buffer) > 1:
                        del buffer[:-1]
                    break

                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start > 0:
                        del buffer[:start]
                    if len(buffer) > 4 * 1024 * 1024:
                        del buffer[:-2]
                    break

                frame = bytes(buffer[start:end + 2])
                del buffer[:end + 2]
                self._publish(frame)

    def _stop_process(self):
        proc = self.proc
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
        self.proc = None


camera = SharedMjpegCamera()


class CameraHandler(BaseHTTPRequestHandler):
    server_version = "TurtleBotCamera/2.0"

    def log_message(self, fmt, *args):
        print(f"{self.client_address[0]} - {fmt % args}", flush=True)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            self.send_index()
        elif path == "/stream.mjpg":
            self.send_stream()
        elif path == "/snapshot.jpg":
            self.send_snapshot()
        elif path == "/detections.json":
            self.send_detections()
        elif path == "/healthz":
            self.send_health()
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def send_index(self):
        stats = camera.stats()
        html = f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\">
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
  <title>TurtleBot3 Camera</title>
  <style>
    :root {{ color-scheme: dark; font-family: Verdana, sans-serif; }}
    body {{ margin: 0; min-height: 100vh; background: #101820; color: #f4f1de; }}
    main {{ width: min(1320px, calc(100vw - 32px)); margin: 0 auto; padding: 28px 0; }}
    h1 {{ margin: 0 0 12px; font-size: clamp(26px, 4vw, 42px); }}
    p {{ color: #c9d1d9; line-height: 1.45; }}
    .grid {{ display: grid; grid-template-columns: minmax(0, {CAMERA_WIDTH}px) minmax(340px, 1fr); gap: 18px; align-items: start; }}
    .panel {{ padding: 12px; border-radius: 18px; background: #17212b; box-shadow: 0 20px 60px rgba(0,0,0,.35); }}
    .video-wrap {{ position: relative; width: 100%; max-width: {CAMERA_WIDTH}px; aspect-ratio: {CAMERA_WIDTH}/{CAMERA_HEIGHT}; }}
    img {{ display: block; width: 100%; height: 100%; object-fit: contain; background: #000; border-radius: 12px; }}
    canvas {{ position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; border-radius: 12px; }}
    .json-head {{ display: flex; align-items: baseline; justify-content: space-between; gap: 12px; margin: 4px 4px 10px; }}
    .json-head h2 {{ margin: 0; font-size: 20px; }}
    .status {{ color: #93c5fd; font-size: 13px; white-space: nowrap; }}
    pre {{ min-height: 410px; max-height: 70vh; overflow: auto; margin: 0; padding: 14px; border-radius: 12px; background: #08111a; color: #d7f9ff; font: 13px/1.45 Consolas, monospace; white-space: pre-wrap; word-break: break-word; }}
    a {{ color: #7dd3fc; }}
    @media (max-width: 980px) {{ .grid {{ grid-template-columns: 1fr; }} main {{ width: min(720px, calc(100vw - 32px)); }} }}
  </style>
</head>
<body>
  <main>
    <h1>TurtleBot3 Camera</h1>
    <p>{CAMERA_DEVICE} at {CAMERA_WIDTH}x{CAMERA_HEIGHT}, {CAMERA_FPS} fps. Shared stream clients: {stats['clients']}/{CAMERA_MAX_CLIENTS}. Endpoint: <a href=\"/stream.mjpg\">/stream.mjpg</a></p>
    <div class=\"grid\">
      <section class=\"panel\">
        <div class=\"video-wrap\">
          <img id=\"camera-stream\" src=\"/stream.mjpg\" width=\"{CAMERA_WIDTH}\" height=\"{CAMERA_HEIGHT}\" alt=\"Camera stream\">
          <canvas id=\"detection-overlay\"></canvas>
        </div>
      </section>
      <section class=\"panel\">
        <div class=\"json-head\">
          <h2>YOLO JSON</h2>
          <span id=\"json-status\" class=\"status\">waiting...</span>
        </div>
        <pre id=\"detections-json\">Loading /detections.json...</pre>
      </section>
    </div>
  </main>
  <script>
    const output = document.getElementById("detections-json");
    const status = document.getElementById("json-status");
    const image = document.getElementById("camera-stream");
    const canvas = document.getElementById("detection-overlay");
    const ctx = canvas.getContext("2d");
    let lastPayload = null;

    const palette = [
      "#22c55e", "#38bdf8", "#f97316", "#f43f5e",
      "#eab308", "#a78bfa", "#14b8a6", "#f472b6"
    ];

    function formatAge(payload) {{
      if (!payload || typeof payload.stamp_unix !== "number") {{
        return "";
      }}
      const age = Math.max(0, Date.now() / 1000 - payload.stamp_unix);
      return `, age ${{age.toFixed(1)}}s`;
    }}

    function resizeOverlay() {{
      const rect = image.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const nextWidth = Math.max(1, Math.round(rect.width * dpr));
      const nextHeight = Math.max(1, Math.round(rect.height * dpr));
      if (canvas.width !== nextWidth || canvas.height !== nextHeight) {{
        canvas.width = nextWidth;
        canvas.height = nextHeight;
      }}
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      canvas.style.width = `${{rect.width}}px`;
      canvas.style.height = `${{rect.height}}px`;
      return rect;
    }}

    function drawDetections(payload) {{
      const rect = resizeOverlay();
      ctx.clearRect(0, 0, rect.width, rect.height);
      if (!payload || !Array.isArray(payload.detections) || payload.detections.length === 0) {{
        return;
      }}

      const sourceWidth = payload.frame?.width || {CAMERA_WIDTH};
      const sourceHeight = payload.frame?.height || {CAMERA_HEIGHT};
      const scaleX = rect.width / sourceWidth;
      const scaleY = rect.height / sourceHeight;

      payload.detections.forEach((det, index) => {{
        if (!Array.isArray(det.bbox_xyxy) || det.bbox_xyxy.length !== 4) {{
          return;
        }}

        const [x1, y1, x2, y2] = det.bbox_xyxy;
        const x = x1 * scaleX;
        const y = y1 * scaleY;
        const w = Math.max(1, (x2 - x1) * scaleX);
        const h = Math.max(1, (y2 - y1) * scaleY);
        const color = palette[(det.class_id ?? index) % palette.length];
        const confidence = Number(det.confidence ?? 0).toFixed(2);
        const label = `${{det.class_name ?? "object"}} ${{confidence}}`;

        ctx.lineWidth = 3;
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.strokeRect(x, y, w, h);

        ctx.font = "600 14px Consolas, monospace";
        const metrics = ctx.measureText(label);
        const labelWidth = metrics.width + 12;
        const labelHeight = 24;
        const labelY = y > labelHeight + 4 ? y - labelHeight - 4 : y + 4;

        ctx.fillRect(x, labelY, labelWidth, labelHeight);
        ctx.fillStyle = "#06111c";
        ctx.fillText(label, x + 6, labelY + 17);
      }});
    }}

    async function refreshDetections() {{
      try {{
        const response = await fetch(`/detections.json?ts=${{Date.now()}}`, {{ cache: "no-store" }});
        const payload = await response.json();
        lastPayload = payload;
        output.textContent = JSON.stringify(payload, null, 2);
        const count = Array.isArray(payload.detections) ? payload.detections.length : 0;
        const seq = payload.seq !== undefined ? `seq ${{payload.seq}}` : "no seq";
        status.textContent = `${{seq}}, detections ${{count}}${{formatAge(payload)}}`;
        drawDetections(payload);
      }} catch (error) {{
        output.textContent = `Failed to load /detections.json\\n${{error}}`;
        status.textContent = "offline";
        drawDetections(null);
      }}
    }}

    image.addEventListener("load", () => drawDetections(lastPayload));
    window.addEventListener("resize", () => drawDetections(lastPayload));
    refreshDetections();
    setInterval(refreshDetections, 500);
  </script>
</body>
</html>
""".encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(html)

    def send_health(self):
        stats = camera.stats()
        age = stats["last_frame_age"]
        healthy = age is not None and age < FRAME_WAIT_TIMEOUT
        body = [
            "ok" if healthy else "not_ready",
            f"device={CAMERA_DEVICE}",
            f"resolution={CAMERA_WIDTH}x{CAMERA_HEIGHT}",
            f"fps={CAMERA_FPS}",
            f"clients={stats['clients']}/{CAMERA_MAX_CLIENTS}",
            f"frame_id={stats['frame_id']}",
            f"last_frame_age={age:.3f}" if age is not None else "last_frame_age=none",
        ]
        if stats["error"]:
            body.append(f"error={stats['error']}")
        data = ("\n".join(body) + "\n").encode("utf-8")
        self.send_response(HTTPStatus.OK if healthy else HTTPStatus.SERVICE_UNAVAILABLE)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_detections(self):
        try:
            data = DETECTIONS_PATH.read_bytes()
            if not data.strip():
                raise ValueError("empty detections file")
        except Exception:
            data = (
                b'{"schema":"tb3_yolo_detections.v1","status":"no_data",'
                b'"detections":[],"best_detection":null}\n'
            )
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_snapshot(self):
        frame, _ = camera.wait_for_frame(-1)
        if frame is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera frame is not ready")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def send_stream(self):
        if not camera.try_register_client():
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "too many stream clients")
            return

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        last_seen_id = -1
        try:
            while True:
                frame, last_seen_id = camera.wait_for_frame(last_seen_id)
                if frame is None:
                    continue
                header = (
                    f"--{BOUNDARY}\r\n"
                    "Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n\r\n"
                ).encode("ascii")
                self.wfile.write(header)
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            camera.unregister_client()


class CameraServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    camera.start()
    httpd = CameraServer(("0.0.0.0", CAMERA_PORT), CameraHandler)

    def shutdown(signum, frame):
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    print(
        f"Serving shared camera {CAMERA_DEVICE} on http://0.0.0.0:{CAMERA_PORT}/ "
        f"({CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}, max clients {CAMERA_MAX_CLIENTS})",
        flush=True,
    )
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
        camera.stop()


if __name__ == "__main__":
    main()
