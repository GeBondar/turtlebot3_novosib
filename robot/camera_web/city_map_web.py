#!/usr/bin/env python3
import json
import math
import os
import signal
import threading
import time
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HOST = os.environ.get("CITY_MAP_HOST", "0.0.0.0")
PORT = int(os.environ.get("CITY_MAP_PORT", "8090"))

FIELD_SIZE_M = 4.0
GRID_CELLS = 5
CELL_SIZE_M = FIELD_SIZE_M / GRID_CELLS
REFERENCE_START_M = 0.2
REFERENCE_STEP_M = 0.4
MAP_ORIGIN_OFFSET_X_M = float(os.environ.get("CITY_MAP_ORIGIN_OFFSET_X", str(REFERENCE_START_M)))
MAP_ORIGIN_OFFSET_Y_M = float(os.environ.get("CITY_MAP_ORIGIN_OFFSET_Y", str(REFERENCE_START_M)))
SIGN_SLOT_EDGE_OFFSET_M = 0.05
SIGN_SLOT_SIDE_POSITIONS_M = (0.2, 0.6)
SIGN_VISIBILITY_RADIUS_M = 0.32
DETECTIONS_URL = os.environ.get(
    "CITY_DETECTIONS_URL",
    "http://127.0.0.1:8080/detections.json",
)
CAMERA_STREAM_URL = os.environ.get(
    "CITY_CAMERA_STREAM_URL",
    "http://127.0.0.1:8080/stream.mjpg",
)
DETECTIONS_TIMEOUT_S = float(os.environ.get("CITY_DETECTIONS_TIMEOUT", "0.35"))
CAMERA_PROXY_TIMEOUT_S = float(os.environ.get("CITY_CAMERA_PROXY_TIMEOUT", "3.0"))
SCAN_PATH = Path(
    os.environ.get(
        "CITY_SCAN_PATH",
        str(Path(__file__).with_name("latest_scan.json")),
    )
)
POSE_PATH = Path(
    os.environ.get(
        "CITY_POSE_PATH",
        str(Path(__file__).with_name("latest_pose.json")),
    )
)
COSTMAP_PATH = Path(
    os.environ.get(
        "CITY_COSTMAP_PATH",
        str(Path(__file__).with_name("latest_costmaps.json")),
    )
)
TF_PATH = Path(
    os.environ.get(
        "CITY_TF_PATH",
        str(Path(__file__).with_name("latest_tf.json")),
    )
)
INITIAL_POSE_COMMAND_PATH = Path(
    os.environ.get(
        "CITY_INITIAL_POSE_COMMAND_PATH",
        str(Path(__file__).with_name("initial_pose_command.json")),
    )
)
INITIAL_POSE_FRAME = os.environ.get("CITY_INITIAL_POSE_FRAME", "map")
NAV_GOAL_COMMAND_PATH = Path(
    os.environ.get(
        "CITY_NAV_GOAL_COMMAND_PATH",
        str(Path(__file__).with_name("nav_goal_command.json")),
    )
)
NAV_GOAL_STATUS_PATH = Path(
    os.environ.get(
        "CITY_NAV_GOAL_STATUS_PATH",
        str(Path(__file__).with_name("nav_goal_status.json")),
    )
)
NAV_GOAL_ACTION = os.environ.get("CITY_NAV_GOAL_ACTION", "/navigate_to_pose")
RECOGNIZED_SIGNS_PATH = Path(
    os.environ.get(
        "CITY_SIGNS_PATH",
        str(Path(__file__).with_name("recognized_signs.json")),
    )
)

STATE_LOCK = threading.Lock()
SIGN_MEMORY = []
ROBOT_STATE = {
    "reference_point": None,
    "heading": None,
    "visible_point": None,
    "visible_slot_id": None,
    "updated_unix": None,
}
DETECTION_SOURCE_STATE = {
    "url": DETECTIONS_URL,
    "status": "not_polled",
    "last_seq": None,
    "last_stamp_unix": None,
    "last_error": None,
}

# Section coordinates are zero-based and measured from the bottom-left corner.
BLOCKED_SECTIONS = (
    (1, 1),
    (1, 3),
    (3, 1),
    (3, 3),
)

SIGN_TYPES = {
    "straight": {"label": "движение прямо", "short": "S", "color": "#2563eb"},
    "left": {"label": "движение налево", "short": "L", "color": "#2563eb"},
    "right": {"label": "движение направо", "short": "R", "color": "#2563eb"},
    "no_left": {"label": "поворот налево запрещен", "short": "NL", "color": "#dc2626"},
    "no_right": {"label": "поворот направо запрещен", "short": "NR", "color": "#dc2626"},
    "stop": {"label": "место остановки", "short": "BUS", "color": "#16a34a"},
    "parking": {"label": "парковка", "short": "P", "color": "#2563eb"},
    "danger": {"label": "опасный объект", "short": "!", "color": "#f97316"},
    "unknown": {"label": "неизвестный знак", "short": "?", "color": "#6b7280"},
}

YOLO_CLASS_TO_SIGN_TYPE = {
    "forward": "straight",
    "straight": "straight",
    "left": "left",
    "right": "right",
    "no_left": "no_left",
    "no_right": "no_right",
    "bus_stop": "stop",
    "red_bus": "stop",
    "p_sign": "parking",
    "parking": "parking",
    "danger": "danger",
    "danger_object": "danger",
}

HEADING_VECTORS = {
    "north": (0.0, REFERENCE_STEP_M),
    "south": (0.0, -REFERENCE_STEP_M),
    "east": (REFERENCE_STEP_M, 0.0),
    "west": (-REFERENCE_STEP_M, 0.0),
}

HEADING_YAWS = {
    "east": 0.0,
    "north": math.pi / 2.0,
    "west": math.pi,
    "south": -math.pi / 2.0,
}

DEFAULT_START_POINT = {"x": 0.6, "y": 0.6, "heading": "north"}


def course_to_map_xy(x, y):
    return (
        float(x) - MAP_ORIGIN_OFFSET_X_M,
        float(y) - MAP_ORIGIN_OFFSET_Y_M,
    )


def map_to_course_xy(x, y):
    return (
        float(x) + MAP_ORIGIN_OFFSET_X_M,
        float(y) + MAP_ORIGIN_OFFSET_Y_M,
    )


def orientation_from_yaw(yaw):
    return {
        "z": round(math.sin(yaw / 2.0), 6),
        "w": round(math.cos(yaw / 2.0), 6),
    }

INDEX_HTML = r"""<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Карта города TurtleBot3</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Inter, "Segoe UI", Arial, sans-serif;
      --page: #f4f5f1;
      --ink: #1f2933;
      --muted: #607080;
      --line: #cfd7cf;
      --panel: #ffffff;
      --road: #44474c;
      --road-edge: #2e3136;
      --marking: #f7f7f2;
      --grass: #7aa266;
      --building: #c6a15b;
      --point: #ffd24a;
      --point-border: #3d3520;
      --sign-slot: #15a3a3;
      --sign-slot-fill: #e4fbf8;
      --robot: #111827;
      --visible: #ef7b45;
      --accent: #da3c32;
      --goal: #2563eb;
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      min-height: 100vh;
      background: var(--page);
      color: var(--ink);
    }

    header {
      display: flex;
      align-items: flex-end;
      justify-content: space-between;
      gap: 20px;
      padding: 20px clamp(16px, 4vw, 36px) 14px;
      border-bottom: 1px solid var(--line);
      background: rgba(255, 255, 255, 0.78);
    }

    h1 {
      margin: 0;
      font-size: clamp(24px, 3.2vw, 38px);
      line-height: 1.05;
      font-weight: 760;
      letter-spacing: 0;
    }

    .stats {
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 8px;
      color: var(--muted);
      font-size: 14px;
    }

    .stats span {
      min-width: 88px;
      padding: 6px 8px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fff;
      text-align: center;
    }

    main {
      width: min(1280px, calc(100vw - 32px));
      margin: 18px auto 28px;
    }

    .dashboard {
      display: grid;
      grid-template-columns: minmax(520px, 1.05fr) minmax(420px, 0.95fr);
      gap: 14px;
      align-items: start;
    }

    .details-grid {
      display: grid;
      grid-template-columns: minmax(360px, 0.9fr) minmax(420px, 1.1fr);
      gap: 14px;
      margin-top: 16px;
      align-items: start;
    }

    .map-panel,
    .camera-panel,
    .side-panel,
    .json-panel,
    .rviz-panel {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      box-shadow: 0 12px 30px rgba(31, 41, 51, 0.08);
    }

    .map-panel {
      padding: 14px;
    }

    .camera-panel {
      padding: 12px;
    }

    .panel-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      min-height: 28px;
      margin-bottom: 8px;
      font-size: 13px;
      color: var(--muted);
    }

    .panel-head strong {
      color: var(--ink);
      font-size: 15px;
    }

    .inline-toggles {
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 10px;
    }

    .mini-toggle {
      display: inline-flex;
      align-items: center;
      gap: 5px;
      white-space: nowrap;
    }

    .mini-toggle input {
      width: 15px;
      height: 15px;
      accent-color: var(--accent);
    }

    #map {
      width: 100%;
      min-height: min(58vh, 620px);
    }

    svg {
      display: block;
      width: 100%;
      height: auto;
    }

    .side-panel {
      padding: 14px;
    }

    .video-wrap {
      position: relative;
      width: 100%;
      aspect-ratio: 4 / 3;
      min-height: 260px;
      overflow: hidden;
      border-radius: 8px;
      background: #060b10;
    }

    .video-wrap img,
    .video-wrap canvas {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      border-radius: 8px;
    }

    .video-wrap img {
      display: block;
      object-fit: contain;
      background: #000;
    }

    .video-wrap canvas {
      pointer-events: none;
    }

    .json-panel {
      padding: 14px;
    }

    .rviz-panel {
      margin-top: 16px;
      padding: 14px;
    }

    .rviz-tabs {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin: 2px 0 12px;
    }

    .rviz-tab {
      height: 34px;
      padding: 0 12px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #f8faf6;
      color: var(--ink);
      font: inherit;
      font-size: 13px;
      font-weight: 680;
      cursor: pointer;
    }

    .rviz-tab.active {
      border-color: var(--accent);
      background: #fff4ee;
      color: var(--accent);
    }

    .rviz-view {
      display: none;
    }

    .rviz-view.active {
      display: block;
    }

    .rviz-canvas-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      align-items: start;
    }

    .rviz-card {
      display: grid;
      gap: 8px;
      min-width: 0;
    }

    .rviz-card-title {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      color: var(--muted);
      font-size: 12px;
      font-weight: 680;
    }

    .rviz-canvas {
      width: 100%;
      height: 320px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #0d1720;
    }

    .rviz-readout {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      margin-top: 10px;
      font-size: 12px;
    }

    .rviz-readout span {
      min-width: 0;
      padding: 8px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #f8faf6;
      color: var(--muted);
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }

    .tf-grid {
      display: grid;
      grid-template-columns: minmax(320px, 0.9fr) minmax(360px, 1.1fr);
      gap: 12px;
    }

    .tf-list {
      max-height: 420px;
      overflow: auto;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #f8faf6;
    }

    .tf-row {
      display: grid;
      grid-template-columns: minmax(120px, 1fr) minmax(82px, auto);
      gap: 8px;
      padding: 9px 10px;
      border-bottom: 1px solid var(--line);
      font-size: 12px;
    }

    .tf-row:last-child {
      border-bottom: 0;
    }

    .tf-row strong {
      color: var(--ink);
      font-size: 13px;
    }

    .tf-ok { color: #15803d; }
    .tf-missing { color: #b42318; }

    .robot-model-scene {
      position: relative;
      min-height: 360px;
      overflow: hidden;
      border: 1px solid var(--line);
      border-radius: 8px;
      background:
        linear-gradient(180deg, rgba(255,255,255,0.94), rgba(236,241,237,0.96)),
        repeating-linear-gradient(0deg, rgba(84,95,103,0.12) 0 1px, transparent 1px 36px),
        repeating-linear-gradient(90deg, rgba(84,95,103,0.12) 0 1px, transparent 1px 36px);
      perspective: 900px;
    }

    .robot-model {
      position: absolute;
      left: 50%;
      top: 52%;
      width: 156px;
      height: 118px;
      transform-style: preserve-3d;
      transform: translate(-50%, -50%) rotateX(58deg) rotateZ(0deg);
      transition: transform 160ms ease-out;
    }

    .robot-body,
    .robot-sensor,
    .robot-wheel {
      position: absolute;
      border: 2px solid rgba(255,255,255,0.85);
      box-shadow: 0 10px 24px rgba(31, 41, 51, 0.18);
    }

    .robot-body {
      inset: 18px 26px;
      border-radius: 50%;
      background: #2563eb;
    }

    .robot-sensor {
      left: 58px;
      top: 0;
      width: 40px;
      height: 40px;
      border-radius: 50%;
      background: #0f1720;
    }

    .robot-wheel {
      top: 38px;
      width: 22px;
      height: 58px;
      border-radius: 8px;
      background: #202832;
    }

    .robot-wheel.left { left: 10px; }
    .robot-wheel.right { right: 10px; }

    .robot-nose {
      position: absolute;
      left: 72px;
      top: -24px;
      width: 12px;
      height: 52px;
      border-radius: 999px;
      background: var(--accent);
      box-shadow: 0 8px 18px rgba(211, 78, 44, 0.28);
    }

    .json-grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 12px;
    }

    pre {
      min-height: 160px;
      max-height: 360px;
      overflow: auto;
      margin: 0;
      padding: 12px;
      border-radius: 8px;
      background: #0f1720;
      color: #d9f6ff;
      font: 12px/1.45 Consolas, monospace;
      white-space: pre-wrap;
      word-break: break-word;
    }

    .control {
      display: grid;
      gap: 7px;
      margin-bottom: 14px;
      font-size: 13px;
      color: var(--muted);
    }

    select,
    input[type="number"] {
      width: 100%;
      height: 38px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fff;
      color: var(--ink);
      font: inherit;
      padding: 0 10px;
    }

    .control-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }

    .pose-toolbar {
      display: grid;
      grid-template-columns: minmax(150px, 1fr) minmax(120px, 160px) auto minmax(120px, auto);
      gap: 8px;
      align-items: end;
      margin-bottom: 10px;
      padding: 10px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #f8faf6;
    }

    .pose-toolbar .control {
      margin-bottom: 0;
    }

    .action-button {
      height: 38px;
      padding: 0 12px;
      border: 1px solid #b83229;
      border-radius: 8px;
      background: var(--accent);
      color: #fff;
      font: inherit;
      font-weight: 700;
      white-space: nowrap;
      cursor: pointer;
    }

    .action-button:disabled {
      cursor: wait;
      opacity: 0.68;
    }

    .command-status {
      min-width: 120px;
      align-self: center;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.25;
    }

    .nav-toolbar {
      grid-template-columns: minmax(150px, 1fr) minmax(120px, 150px) auto minmax(130px, auto);
      margin-top: 8px;
    }

    .command-preview {
      min-height: 76px;
      max-height: 118px;
      margin: 8px 0 10px;
      padding: 9px 10px;
      font-size: 11px;
    }

    .toggle {
      display: flex;
      align-items: center;
      gap: 9px;
      margin: 12px 0;
      color: var(--ink);
      font-size: 14px;
    }

    input[type="checkbox"] {
      width: 18px;
      height: 18px;
      accent-color: var(--accent);
    }

    dl {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px 12px;
      margin: 16px 0 0;
      padding-top: 14px;
      border-top: 1px solid var(--line);
      font-size: 14px;
    }

    dt { color: var(--muted); }
    dd { margin: 0; font-weight: 650; }

    .point-list {
      margin-top: 14px;
      max-height: 240px;
      overflow: auto;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fafbf8;
    }

    .table-title {
      margin: 16px 0 8px;
      color: var(--muted);
      font-size: 13px;
      font-weight: 680;
    }

    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
    }

    th,
    td {
      padding: 7px 8px;
      border-bottom: 1px solid #e3e8e0;
      text-align: right;
      white-space: nowrap;
    }

    th:first-child,
    td:first-child { text-align: left; }

    th {
      position: sticky;
      top: 0;
      background: #f0f3ec;
      color: var(--muted);
      font-weight: 680;
    }

    @media (max-width: 900px) {
      header {
        align-items: flex-start;
        flex-direction: column;
      }

      .stats { justify-content: flex-start; }

      .dashboard,
      .details-grid {
        grid-template-columns: 1fr;
        width: min(720px, calc(100vw - 24px));
      }

      .rviz-canvas-grid,
      .tf-grid,
      .rviz-readout {
        grid-template-columns: 1fr;
      }

      .map-panel,
      #map { min-height: 360px; }

      .control-row {
        grid-template-columns: 1fr;
      }

      .pose-toolbar {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <header>
    <h1>Карта города TurtleBot3</h1>
    <div class="stats" aria-live="polite">
      <span id="field-size">4.0 м</span>
      <span id="section-count">5 x 5</span>
      <span id="point-count">0 точек</span>
      <span id="slot-count">0 слотов</span>
      <span id="recognized-count">0 знаков</span>
      <span id="source-status">нет данных</span>
    </div>
  </header>

  <main>
    <section class="dashboard">
      <section class="map-panel" aria-label="Карта полигона">
        <div class="panel-head">
          <strong>Город</strong>
          <div class="inline-toggles">
            <label class="mini-toggle"><input id="show-scan" type="checkbox" checked>/scan</label>
            <label class="mini-toggle"><input id="show-base-link" type="checkbox" checked>base_link</label>
          </div>
        </div>
        <div class="pose-toolbar">
          <label class="control">
            Точка робота
            <select id="robot-point">
              <option value="">не задана</option>
            </select>
          </label>

          <label class="control">
            Направление
            <select id="robot-heading">
              <option value="">не задано</option>
              <option value="north">север</option>
              <option value="south">юг</option>
              <option value="east">восток</option>
              <option value="west">запад</option>
            </select>
          </label>

          <button id="initial-pose-button" class="action-button" type="button">2D Pose Estimate</button>
          <span id="initial-pose-status" class="command-status">0.4; 0.4, север</span>
        </div>
        <div class="pose-toolbar nav-toolbar">
          <label class="control">
            Goal-точка
            <select id="nav-goal-point">
              <option value="">выберите на карте</option>
            </select>
          </label>

          <label class="control">
            Финальный угол, град
            <input id="nav-yaw-deg" type="number" min="-180" max="180" step="15" value="0">
          </label>

          <button id="nav-goal-button" class="action-button" type="button">Отправить команду</button>
          <span id="nav-goal-status" class="command-status">Nav2 goal не выбран</span>
        </div>
        <pre id="nav-goal-preview" class="command-preview">Выберите точку на карте или из списка.</pre>
        <div id="map"></div>
      </section>

      <section class="camera-panel" aria-label="Камера и распознавание">
        <div class="panel-head">
          <strong>Камера</strong>
          <span id="camera-status">waiting...</span>
        </div>
        <div class="video-wrap">
          <img id="camera-stream" alt="Camera stream">
          <canvas id="detection-overlay"></canvas>
        </div>
      </section>
    </section>

    <section class="details-grid">
    <aside class="side-panel">
      <label class="control">
        Начало координат
        <select id="origin">
          <option value="bottom-left">нижний левый</option>
          <option value="top-left">верхний левый</option>
          <option value="bottom-right">нижний правый</option>
          <option value="top-right">верхний правый</option>
        </select>
      </label>

      <label class="toggle">
        <input id="show-labels" type="checkbox">
        Подписи координат
      </label>

      <label class="toggle">
        <input id="show-road-marking" type="checkbox" checked>
        Дорожная разметка
      </label>

      <label class="toggle">
        <input id="show-sign-slots" type="checkbox" checked>
        Места установки знаков
      </label>

      <label class="toggle">
        <input id="show-recognized-signs" type="checkbox" checked>
        Распознанные знаки
      </label>

      <dl>
        <dt>Секция</dt>
        <dd id="cell-size">0.8 м</dd>
        <dt>Шаг точек</dt>
        <dd id="point-step">0.4 м</dd>
        <dt>Первая точка</dt>
        <dd id="first-point">0.2; 0.2</dd>
        <dt>Здания</dt>
        <dd id="blocked-count">4</dd>
        <dt>Слоты знаков</dt>
        <dd id="slot-count-details">32</dd>
        <dt>Распознано</dt>
        <dd id="recognized-count-details">0</dd>
        <dt>Видимый слот</dt>
        <dd id="visible-slot">-</dd>
        <dt>Источник YOLO</dt>
        <dd id="detection-source">-</dd>
      </dl>

      <div class="table-title">Реперные точки</div>
      <div class="point-list">
        <table>
          <thead>
            <tr><th>ID</th><th>X, м</th><th>Y, м</th></tr>
          </thead>
          <tbody id="points-table"></tbody>
        </table>
      </div>

      <div class="table-title">Слоты знаков</div>
      <div class="point-list">
        <table>
          <thead>
            <tr><th>ID</th><th>Стор.</th><th>Знак</th></tr>
          </thead>
          <tbody id="signs-table"></tbody>
        </table>
      </div>
    </aside>
    <section class="json-panel">
      <div class="panel-head">
        <strong>Служебные данные</strong>
        <span>JSON и отладка</span>
      </div>
      <div class="json-grid">
        <div>
          <div class="table-title">YOLO detections</div>
          <pre id="detections-json">Loading /detections.json...</pre>
        </div>
        <div>
          <div class="table-title">Карта, /scan и base_link</div>
          <pre id="telemetry-json">Loading telemetry...</pre>
        </div>
      </div>
    </section>
    </section>

    <section class="rviz-panel" aria-label="Web RViz diagnostics">
      <div class="panel-head">
        <strong>Web RViz</strong>
        <span id="rviz-status">waiting...</span>
      </div>
      <div class="rviz-tabs" role="tablist" aria-label="RViz diagnostics">
        <button class="rviz-tab active" type="button" data-rviz-tab="costmaps">Costmaps</button>
        <button class="rviz-tab" type="button" data-rviz-tab="lidar">Lidar</button>
        <button class="rviz-tab" type="button" data-rviz-tab="tf">TF</button>
        <button class="rviz-tab" type="button" data-rviz-tab="robot">Robot</button>
      </div>

      <div id="rviz-costmaps" class="rviz-view active">
        <div class="rviz-canvas-grid">
          <div class="rviz-card">
            <div class="rviz-card-title"><strong>Global costmap</strong><span id="global-costmap-status">no data</span></div>
            <canvas id="global-costmap-canvas" class="rviz-canvas"></canvas>
          </div>
          <div class="rviz-card">
            <div class="rviz-card-title"><strong>Local costmap</strong><span id="local-costmap-status">no data</span></div>
            <canvas id="local-costmap-canvas" class="rviz-canvas"></canvas>
          </div>
        </div>
      </div>

      <div id="rviz-lidar" class="rviz-view">
        <canvas id="lidar-canvas" class="rviz-canvas"></canvas>
        <div class="rviz-readout">
          <span id="lidar-frame">frame -</span>
          <span id="lidar-points">points 0</span>
          <span id="lidar-range">range -</span>
          <span id="lidar-age">age -</span>
        </div>
      </div>

      <div id="rviz-tf" class="rviz-view">
        <div class="tf-grid">
          <div id="tf-list" class="tf-list"></div>
          <pre id="tf-json">Loading /tf.json...</pre>
        </div>
      </div>

      <div id="rviz-robot" class="rviz-view">
        <div class="robot-model-scene">
          <div id="robot-model" class="robot-model">
            <div class="robot-nose"></div>
            <div class="robot-sensor"></div>
            <div class="robot-wheel left"></div>
            <div class="robot-wheel right"></div>
            <div class="robot-body"></div>
          </div>
        </div>
        <div class="rviz-readout">
          <span id="robot-pose-frame">frame -</span>
          <span id="robot-pose-xy">x -, y -</span>
          <span id="robot-pose-yaw">yaw -</span>
          <span id="nav2-goal-readout">nav2 -</span>
        </div>
      </div>
    </section>
  </main>

  <script>
    const mapRoot = document.getElementById("map");
    const originSelect = document.getElementById("origin");
    const labelsToggle = document.getElementById("show-labels");
    const markingToggle = document.getElementById("show-road-marking");
    const signSlotsToggle = document.getElementById("show-sign-slots");
    const recognizedSignsToggle = document.getElementById("show-recognized-signs");
    const scanToggle = document.getElementById("show-scan");
    const baseLinkToggle = document.getElementById("show-base-link");
    const robotPointSelect = document.getElementById("robot-point");
    const robotHeadingSelect = document.getElementById("robot-heading");
    const initialPoseButton = document.getElementById("initial-pose-button");
    const initialPoseStatus = document.getElementById("initial-pose-status");
    const navGoalPointSelect = document.getElementById("nav-goal-point");
    const navYawDegInput = document.getElementById("nav-yaw-deg");
    const navGoalButton = document.getElementById("nav-goal-button");
    const navGoalStatus = document.getElementById("nav-goal-status");
    const navGoalPreview = document.getElementById("nav-goal-preview");
    const pointsTable = document.getElementById("points-table");
    const signsTable = document.getElementById("signs-table");
    const detectionsJson = document.getElementById("detections-json");
    const telemetryJson = document.getElementById("telemetry-json");
    const cameraStatus = document.getElementById("camera-status");
    const cameraImage = document.getElementById("camera-stream");
    const detectionCanvas = document.getElementById("detection-overlay");
    const detectionCtx = detectionCanvas.getContext("2d");
    const rvizStatus = document.getElementById("rviz-status");
    const rvizTabs = Array.from(document.querySelectorAll(".rviz-tab"));
    const rvizViews = {
      costmaps: document.getElementById("rviz-costmaps"),
      lidar: document.getElementById("rviz-lidar"),
      tf: document.getElementById("rviz-tf"),
      robot: document.getElementById("rviz-robot")
    };
    const globalCostmapCanvas = document.getElementById("global-costmap-canvas");
    const localCostmapCanvas = document.getElementById("local-costmap-canvas");
    const globalCostmapStatus = document.getElementById("global-costmap-status");
    const localCostmapStatus = document.getElementById("local-costmap-status");
    const lidarCanvas = document.getElementById("lidar-canvas");
    const lidarFrame = document.getElementById("lidar-frame");
    const lidarPoints = document.getElementById("lidar-points");
    const lidarRange = document.getElementById("lidar-range");
    const lidarAge = document.getElementById("lidar-age");
    const tfList = document.getElementById("tf-list");
    const tfJson = document.getElementById("tf-json");
    const robotModel = document.getElementById("robot-model");
    const robotPoseFrame = document.getElementById("robot-pose-frame");
    const robotPoseXy = document.getElementById("robot-pose-xy");
    const robotPoseYaw = document.getElementById("robot-pose-yaw");
    const nav2GoalReadout = document.getElementById("nav2-goal-readout");
    let mapData = null;
    let lastDetections = null;
    let rvizData = null;
    let activeRvizTab = "costmaps";
    let updatingRobotControls = false;
    const defaultStartPointKey = "0.6,0.6";
    const defaultStartHeading = "north";
    const navActionName = "/navigate_to_pose";

    const detectionPalette = [
      "#22c55e", "#38bdf8", "#f97316", "#f43f5e",
      "#eab308", "#a78bfa", "#14b8a6", "#f472b6"
    ];

    function fixed(value) {
      return Number(value).toFixed(1);
    }

    function coordOffset() {
      return mapData?.coordinate_frame?.origin_offset_m || { x: 0, y: 0 };
    }

    function courseToMap(point) {
      const offset = coordOffset();
      return {
        x: Number(point.x) - Number(offset.x || 0),
        y: Number(point.y) - Number(offset.y || 0)
      };
    }

    function mapToCourse(point) {
      const offset = coordOffset();
      return {
        x: Number(point.x) + Number(offset.x || 0),
        y: Number(point.y) + Number(offset.y || 0)
      };
    }

    function formatAge(payload) {
      if (!payload || typeof payload.stamp_unix !== "number") {
        return "";
      }
      const age = Math.max(0, Date.now() / 1000 - payload.stamp_unix);
      return `, ${age.toFixed(1)}s`;
    }

    function resizeDetectionOverlay() {
      const rect = cameraImage.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const width = Math.max(1, Math.round(rect.width * dpr));
      const height = Math.max(1, Math.round(rect.height * dpr));
      if (detectionCanvas.width !== width || detectionCanvas.height !== height) {
        detectionCanvas.width = width;
        detectionCanvas.height = height;
      }
      detectionCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
      detectionCanvas.style.width = `${rect.width}px`;
      detectionCanvas.style.height = `${rect.height}px`;
      return rect;
    }

    function drawDetections(payload) {
      const rect = resizeDetectionOverlay();
      detectionCtx.clearRect(0, 0, rect.width, rect.height);
      if (!payload || !Array.isArray(payload.detections) || payload.detections.length === 0) {
        return;
      }

      const sourceWidth = payload.frame?.width || 640;
      const sourceHeight = payload.frame?.height || 480;
      const scaleX = rect.width / sourceWidth;
      const scaleY = rect.height / sourceHeight;

      payload.detections.forEach((det, index) => {
        if (!Array.isArray(det.bbox_xyxy) || det.bbox_xyxy.length !== 4) {
          return;
        }

        const [x1, y1, x2, y2] = det.bbox_xyxy;
        const x = x1 * scaleX;
        const y = y1 * scaleY;
        const w = Math.max(1, (x2 - x1) * scaleX);
        const h = Math.max(1, (y2 - y1) * scaleY);
        const color = detectionPalette[(det.class_id ?? index) % detectionPalette.length];
        const confidence = Number(det.confidence ?? 0).toFixed(2);
        const label = `${det.class_name ?? "object"} ${confidence}`;

        detectionCtx.lineWidth = 3;
        detectionCtx.strokeStyle = color;
        detectionCtx.fillStyle = color;
        detectionCtx.strokeRect(x, y, w, h);

        detectionCtx.font = "600 13px Consolas, monospace";
        const metrics = detectionCtx.measureText(label);
        const labelWidth = metrics.width + 12;
        const labelHeight = 22;
        const labelY = y > labelHeight + 4 ? y - labelHeight - 4 : y + 4;

        detectionCtx.fillRect(x, labelY, labelWidth, labelHeight);
        detectionCtx.fillStyle = "#06111c";
        detectionCtx.fillText(label, x + 6, labelY + 16);
      });
    }

    function resizeCanvas(canvas) {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const width = Math.max(1, Math.round(rect.width * dpr));
      const height = Math.max(1, Math.round(rect.height * dpr));
      if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
      }
      const ctx = canvas.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { ctx, width: rect.width, height: rect.height };
    }

    function clearCanvas(canvas, label) {
      const { ctx, width, height } = resizeCanvas(canvas);
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#0d1720";
      ctx.fillRect(0, 0, width, height);
      ctx.fillStyle = "#9fb0bd";
      ctx.font = "600 13px Consolas, monospace";
      ctx.textAlign = "center";
      ctx.fillText(label, width / 2, height / 2);
    }

    function costmapColor(value) {
      const v = Number(value);
      if (!Number.isFinite(v) || v < 0) {
        return [101, 116, 128, 255];
      }
      if (v <= 0) {
        return [248, 250, 252, 255];
      }
      if (v < 35) {
        return [250, 204, 21, 255];
      }
      if (v < 70) {
        return [249, 115, 22, 255];
      }
      return [185, 28, 28, 255];
    }

    function drawCostmap(canvas, costmap, statusNode) {
      if (!costmap || costmap.status !== "ok" || !Array.isArray(costmap.data) || costmap.data.length === 0) {
        clearCanvas(canvas, "no costmap data");
        statusNode.textContent = costmap?.status || "no data";
        return;
      }

      const { ctx, width, height } = resizeCanvas(canvas);
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#0d1720";
      ctx.fillRect(0, 0, width, height);

      const source = document.createElement("canvas");
      source.width = costmap.width;
      source.height = costmap.height;
      const sourceCtx = source.getContext("2d");
      const image = sourceCtx.createImageData(costmap.width, costmap.height);
      for (let y = 0; y < costmap.height; y += 1) {
        for (let x = 0; x < costmap.width; x += 1) {
          const srcIndex = y * costmap.width + x;
          const dstY = costmap.height - 1 - y;
          const dstIndex = (dstY * costmap.width + x) * 4;
          const color = costmapColor(costmap.data[srcIndex]);
          image.data[dstIndex] = color[0];
          image.data[dstIndex + 1] = color[1];
          image.data[dstIndex + 2] = color[2];
          image.data[dstIndex + 3] = color[3];
        }
      }
      sourceCtx.putImageData(image, 0, 0);

      const scale = Math.min(width / costmap.width, height / costmap.height);
      const drawW = costmap.width * scale;
      const drawH = costmap.height * scale;
      const x = (width - drawW) / 2;
      const y = (height - drawH) / 2;
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(source, x, y, drawW, drawH);
      ctx.strokeStyle = "rgba(255,255,255,0.78)";
      ctx.lineWidth = 1.5;
      ctx.strokeRect(x, y, drawW, drawH);

      const age = typeof costmap.stamp_unix === "number" ? `${Math.max(0, Date.now() / 1000 - costmap.stamp_unix).toFixed(1)}s` : "-";
      statusNode.textContent = `${costmap.frame_id || "-"} ${costmap.source_width || costmap.width}x${costmap.source_height || costmap.height} ${Number(costmap.source_resolution || costmap.resolution || 0).toFixed(3)}m ${age}`;
    }

    function drawLidar() {
      const scan = rvizData?.scan || mapData?.scan;
      if (!scan || !Array.isArray(scan.points) || scan.points.length === 0) {
        clearCanvas(lidarCanvas, "no /scan data");
        lidarFrame.textContent = "frame -";
        lidarPoints.textContent = "points 0";
        lidarRange.textContent = "range -";
        lidarAge.textContent = "age -";
        return;
      }

      const { ctx, width, height } = resizeCanvas(lidarCanvas);
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#0d1720";
      ctx.fillRect(0, 0, width, height);

      const maxRange = Math.max(1, Number(scan.range_max || 4.5));
      const scale = Math.min(width, height) * 0.42 / Math.min(maxRange, 6);
      const cx = width / 2;
      const cy = height * 0.66;

      ctx.strokeStyle = "rgba(148,163,184,0.25)";
      ctx.lineWidth = 1;
      for (let r = 0.5; r <= Math.min(maxRange, 6); r += 0.5) {
        ctx.beginPath();
        ctx.arc(cx, cy, r * scale, Math.PI, 2 * Math.PI);
        ctx.stroke();
      }
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx, cy - Math.min(maxRange, 6) * scale);
      ctx.moveTo(cx - Math.min(maxRange, 6) * scale, cy);
      ctx.lineTo(cx + Math.min(maxRange, 6) * scale, cy);
      ctx.stroke();

      ctx.fillStyle = "#38bdf8";
      scan.points.forEach((point) => {
        const px = Number(point.x);
        const py = Number(point.y);
        if (!Number.isFinite(px) || !Number.isFinite(py)) {
          return;
        }
        const sx = cx + py * scale;
        const sy = cy - px * scale;
        if (sx < 0 || sx > width || sy < 0 || sy > height) {
          return;
        }
        ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
      });

      ctx.fillStyle = "#2563eb";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(cx, cy, 12, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.strokeStyle = "#d34e2c";
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx, cy - 26);
      ctx.stroke();

      const age = typeof scan.stamp_unix === "number" ? `${Math.max(0, Date.now() / 1000 - scan.stamp_unix).toFixed(1)}s` : "-";
      lidarFrame.textContent = `frame ${scan.points_frame || scan.frame_id || "-"}`;
      lidarPoints.textContent = `points ${scan.points.length}`;
      lidarRange.textContent = `range ${Number(scan.range_min || 0).toFixed(2)}-${Number(scan.range_max || 0).toFixed(2)}m`;
      lidarAge.textContent = `age ${age}`;
    }

    function renderTf() {
      const tf = rvizData?.tf || mapData?.tf;
      const transforms = Array.isArray(tf?.watched_transforms) ? tf.watched_transforms : [];
      tfList.innerHTML = transforms.length
        ? transforms.map((item) => {
            const ok = item.status === "ok";
            const cls = ok ? "tf-ok" : "tf-missing";
            const pose = ok && item.translation
              ? `${Number(item.translation.x).toFixed(2)}, ${Number(item.translation.y).toFixed(2)}, yaw ${Number(item.yaw || 0).toFixed(2)}`
              : "missing";
            return `<div class="tf-row"><div><strong>${item.target_frame} <- ${item.source_frame}</strong><br>${pose}</div><div class="${cls}">${item.status}</div></div>`;
          }).join("")
        : `<div class="tf-row"><div><strong>TF</strong><br>no watched transforms</div><div class="tf-missing">no data</div></div>`;
      tfJson.textContent = JSON.stringify(tf || { status: "no_data" }, null, 2);
    }

    function renderRobotModel() {
      const pose = currentMapPose();
      if (!pose) {
        robotModel.style.transform = "translate(-50%, -50%) rotateX(58deg) rotateZ(0deg)";
        robotPoseFrame.textContent = "frame -";
        robotPoseXy.textContent = "x -, y -";
        robotPoseYaw.textContent = "yaw -";
      } else {
        const yawDeg = (pose.yaw || 0) * 180 / Math.PI;
        robotModel.style.transform = `translate(-50%, -50%) rotateX(58deg) rotateZ(${yawDeg.toFixed(1)}deg)`;
        robotPoseFrame.textContent = `frame ${mapData.pose?.fixed_frame || pose.source || "-"}`;
        robotPoseXy.textContent = `x ${Number(pose.mapX ?? pose.x).toFixed(2)}, y ${Number(pose.mapY ?? pose.y).toFixed(2)}`;
        robotPoseYaw.textContent = `yaw ${yawDeg.toFixed(1)} deg`;
      }
      const navStatus = rvizData?.nav_goal?.status?.status || mapData?.nav_goal?.status?.status || "-";
      nav2GoalReadout.textContent = `nav2 ${navStatus}`;
    }

    function renderRviz() {
      if (!mapData) {
        return;
      }
      drawCostmap(globalCostmapCanvas, rvizData?.costmaps?.costmaps?.global || mapData.costmaps?.costmaps?.global, globalCostmapStatus);
      drawCostmap(localCostmapCanvas, rvizData?.costmaps?.costmaps?.local || mapData.costmaps?.costmaps?.local, localCostmapStatus);
      drawLidar();
      renderTf();
      renderRobotModel();
      const source = rvizData || mapData;
      rvizStatus.textContent = source?.costmaps?.status || source?.tf?.status || "no data";
    }

    function selectRvizTab(tab) {
      activeRvizTab = tab;
      rvizTabs.forEach((button) => button.classList.toggle("active", button.dataset.rvizTab === tab));
      Object.entries(rvizViews).forEach(([name, node]) => node.classList.toggle("active", name === tab));
      renderRviz();
    }

    async function refreshRviz() {
      if (!mapData) {
        return;
      }
      try {
        const response = await fetch(`/rviz-data.json?ts=${Date.now()}`, { cache: "no-store" });
        rvizData = await response.json();
        mapData.scan = rvizData.scan || mapData.scan;
        mapData.pose = rvizData.pose || mapData.pose;
        mapData.costmaps = rvizData.costmaps || mapData.costmaps;
        mapData.tf = rvizData.tf || mapData.tf;
        mapData.nav_goal = rvizData.nav_goal || mapData.nav_goal;
        renderRviz();
      } catch (error) {
        rvizStatus.textContent = "offline";
        console.warn("failed to refresh rviz data", error);
      }
    }

    function displayPoint(point, origin) {
      const size = mapData.field.size_m;
      const shifted = courseToMap(point);
      const offset = coordOffset();
      const minX = -Number(offset.x || 0);
      const minY = -Number(offset.y || 0);
      const maxX = size - Number(offset.x || 0);
      const maxY = size - Number(offset.y || 0);
      if (origin === "top-left") {
        return { x: shifted.x, y: maxY - shifted.y + minY };
      }
      if (origin === "bottom-right") {
        return { x: maxX - shifted.x + minX, y: shifted.y };
      }
      if (origin === "top-right") {
        return { x: maxX - shifted.x + minX, y: maxY - shifted.y + minY };
      }
      return shifted;
    }

    function currentMapPose() {
      const pose = mapData.pose?.pose;
      if (pose && Number.isFinite(Number(pose.x)) && Number.isFinite(Number(pose.y))) {
        const coursePose = mapToCourse(pose);
        return {
          x: coursePose.x,
          y: coursePose.y,
          mapX: Number(pose.x),
          mapY: Number(pose.y),
          yaw: Number(pose.yaw || 0),
          source: mapData.pose.source || "tf"
        };
      }

      const ref = mapData.robot_state?.reference_point;
      if (ref) {
        const headingYaw = { east: 0, north: Math.PI / 2, west: Math.PI, south: -Math.PI / 2 };
        return {
          x: ref.x,
          y: ref.y,
          yaw: headingYaw[mapData.robot_state?.heading] || 0,
          source: "reference_point"
        };
      }
      return null;
    }

    function scanPointToMap(point, pose, scan) {
      if (!point) {
        return null;
      }
      if (point.frame === "map" || scan?.frame_id === "map" || scan?.points_frame === "map") {
        return mapToCourse(point);
      }
      if (!pose) {
        return null;
      }

      const px = Number(point.x);
      const py = Number(point.y);
      if (!Number.isFinite(px) || !Number.isFinite(py)) {
        return null;
      }
      const cos = Math.cos(pose.yaw || 0);
      const sin = Math.sin(pose.yaw || 0);
      return {
        x: pose.x + px * cos - py * sin,
        y: pose.y + px * sin + py * cos
      };
    }

    function physicalToSvg(x, y, pad, scale, size) {
      return {
        x: pad + x * scale,
        y: pad + (size - y) * scale
      };
    }

    function sectionToRect(section, pad, scale, size, cell) {
      const x = section.col * cell;
      const y = section.row * cell;
      const topLeft = physicalToSvg(x, y + cell, pad, scale, size);
      return {
        x: topLeft.x,
        y: topLeft.y,
        width: cell * scale,
        height: cell * scale
      };
    }

    function axisGeometry(origin, pad, fieldPx, scale, size) {
      const left = pad;
      const right = pad + fieldPx;
      const top = pad;
      const bottom = pad + fieldPx;
      const offset = coordOffset();
      const originCourseX = origin.endsWith("right") ? size - Number(offset.x || 0) : Number(offset.x || 0);
      const originCourseY = origin.startsWith("top") ? size - Number(offset.y || 0) : Number(offset.y || 0);
      const svgOrigin = physicalToSvg(originCourseX, originCourseY, pad, scale, size);
      if (origin === "top-left") {
        return { ox: svgOrigin.x, oy: svgOrigin.y, x2: right, y2: svgOrigin.y, xx: right + 28, xy: svgOrigin.y + 5, xAxis: "right", yx: svgOrigin.x, yy: bottom, yl: "Y" };
      }
      if (origin === "top-right") {
        return { ox: svgOrigin.x, oy: svgOrigin.y, x2: left, y2: svgOrigin.y, xx: left - 28, xy: svgOrigin.y + 5, xAxis: "left", yx: svgOrigin.x, yy: bottom, yl: "Y" };
      }
      if (origin === "bottom-right") {
        return { ox: svgOrigin.x, oy: svgOrigin.y, x2: left, y2: svgOrigin.y, xx: left - 28, xy: svgOrigin.y + 5, xAxis: "left", yx: svgOrigin.x, yy: top, yl: "Y" };
      }
      return { ox: svgOrigin.x, oy: svgOrigin.y, x2: right, y2: svgOrigin.y, xx: right + 28, xy: svgOrigin.y + 5, xAxis: "right", yx: svgOrigin.x, yy: top, yl: "Y" };
    }

    function sectionSet(data) {
      return new Set(data.blocked_sections.map((section) => `${section.col}:${section.row}`));
    }

    function signBySlot() {
      return new Map((mapData.recognized_signs || []).map((sign) => [sign.slot_id, sign]));
    }

    function pointKey(point) {
      if (!point) {
        return "";
      }
      return `${fixed(point.x)},${fixed(point.y)}`;
    }

    function pointLabel(point) {
      if (!point) {
        return "-";
      }
      const shifted = courseToMap(point);
      return `${fixed(shifted.x)}; ${fixed(shifted.y)}`;
    }

    function referencePointByKey(key) {
      if (!key || !mapData) {
        return null;
      }
      return mapData.reference_points.find((point) => pointKey(point) === key) || null;
    }

    function selectedGoalPoint() {
      return referencePointByKey(navGoalPointSelect.value);
    }

    function normalizedYawDeg() {
      const raw = Number(navYawDegInput.value);
      if (!Number.isFinite(raw)) {
        return 0;
      }
      let yaw = raw % 360;
      if (yaw > 180) {
        yaw -= 360;
      } else if (yaw <= -180) {
        yaw += 360;
      }
      return yaw;
    }

    function navYawRad() {
      return normalizedYawDeg() * Math.PI / 180;
    }

    function quaternionFromYaw(yaw) {
      return {
        z: Number(Math.sin(yaw / 2).toFixed(6)),
        w: Number(Math.cos(yaw / 2).toFixed(6))
      };
    }

    function navGoalPreviewPayload() {
      const point = selectedGoalPoint();
      if (!point) {
        return null;
      }
      const yawDeg = normalizedYawDeg();
      const yaw = yawDeg * Math.PI / 180;
      const mapPoint = courseToMap(point);
      return {
        schema: "tb3_nav_goal_command.preview",
        action: navActionName,
        frame_id: "map",
        target_point: {
          id: point.id,
          x: Number(mapPoint.x.toFixed(4)),
          y: Number(mapPoint.y.toFixed(4)),
          course_x: point.x,
          course_y: point.y
        },
        pose: {
          x: Number(mapPoint.x.toFixed(4)),
          y: Number(mapPoint.y.toFixed(4)),
          yaw: Number(yaw.toFixed(6)),
          yaw_deg: Number(yawDeg.toFixed(1)),
          orientation: quaternionFromYaw(yaw)
        }
      };
    }

    function updateNavGoalPreview() {
      const preview = navGoalPreviewPayload();
      if (!preview) {
        navGoalPreview.textContent = "Выберите точку на карте или из списка.";
        navGoalStatus.textContent = mapData?.nav_goal?.status?.status || "Nav2 goal не выбран";
        return;
      }
      navGoalPreview.textContent = JSON.stringify(preview, null, 2);
      const lastStatus = mapData?.nav_goal?.status?.status;
      navGoalStatus.textContent = lastStatus ? `Nav2: ${lastStatus}` : `${fixed(preview.pose.x)}; ${fixed(preview.pose.y)}, ${preview.pose.yaw_deg} deg`;
    }

    function signTypeMeta(type) {
      return mapData.sign_types[type] || mapData.sign_types.unknown;
    }

    function signSlotNormal(side) {
      if (side === "west") {
        return { dx: -1, dy: 0 };
      }
      if (side === "east") {
        return { dx: 1, dy: 0 };
      }
      if (side === "north") {
        return { dx: 0, dy: -1 };
      }
      return { dx: 0, dy: 1 };
    }

    function updateSignCounts() {
      const recognized = (mapData.recognized_signs || []).length;
      document.getElementById("recognized-count").textContent = `${recognized} знаков`;
      document.getElementById("recognized-count-details").textContent = String(recognized);
      document.getElementById("visible-slot").textContent = mapData.robot_state?.visible_slot_id || "-";
      const source = mapData.detection_source || {};
      const sourceText = source.last_seq === null || source.last_seq === undefined
        ? source.status || "-"
        : `${source.status || "ok"} #${source.last_seq}`;
      document.getElementById("source-status").textContent = sourceText;
      document.getElementById("detection-source").textContent = sourceText;
    }

    function populateRobotControls() {
      robotPointSelect.innerHTML = `<option value="">не задана</option>` + mapData.reference_points
        .map((point) => `<option value="${pointKey(point)}">${pointLabel(point)}</option>`)
        .join("");
      syncRobotControls();
    }

    function populateGoalControls() {
      navGoalPointSelect.innerHTML = `<option value="">выберите на карте</option>` + mapData.reference_points
        .map((point) => `<option value="${pointKey(point)}">${point.id}: ${pointLabel(point)}</option>`)
        .join("");
      updateNavGoalPreview();
    }

    function syncRobotControls() {
      updatingRobotControls = true;
      robotPointSelect.value = pointKey(mapData.robot_state?.reference_point) || defaultStartPointKey;
      robotHeadingSelect.value = mapData.robot_state?.heading || defaultStartHeading;
      updatingRobotControls = false;
    }

    function selectedRobotPosePayload() {
      const pointValue = robotPointSelect.value;
      const heading = robotHeadingSelect.value;
      const payload = {};
      if (pointValue) {
        const [x, y] = pointValue.split(",").map(Number);
        payload.x = x;
        payload.y = y;
        payload.coordinate_frame = "course";
      }
      if (heading) {
        payload.heading = heading;
      }
      return payload;
    }

    async function sendRobotState() {
      if (updatingRobotControls) {
        return;
      }

      const payload = selectedRobotPosePayload();

      try {
        const response = await fetch("/robot-state.json", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload)
        });
        const state = await response.json();
        mapData.robot_state = state.robot_state;
        updateSignCounts();
        syncRobotControls();
        renderMap();
      } catch (error) {
        console.warn("failed to update robot state", error);
      }
    }

    async function sendInitialPoseEstimate() {
      const payload = selectedRobotPosePayload();
      if (!Number.isFinite(payload.x) || !Number.isFinite(payload.y)) {
        initialPoseStatus.textContent = "задайте точку";
        return;
      }
      if (!payload.heading) {
        initialPoseStatus.textContent = "задайте направление";
        return;
      }

      initialPoseButton.disabled = true;
      initialPoseStatus.textContent = "отправка...";
      try {
        const response = await fetch("/initial-pose.json", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload)
        });
        const command = await response.json();
        if (!response.ok) {
          throw new Error(command.error || `HTTP ${response.status}`);
        }
        mapData.robot_state = command.robot_state || mapData.robot_state;
        mapData.initial_pose = command.initial_pose || mapData.initial_pose;
        updateSignCounts();
        syncRobotControls();
        renderMap();
        initialPoseStatus.textContent = `отправлено #${command.initial_pose?.seq ?? "-"}`;
      } catch (error) {
        initialPoseStatus.textContent = "ошибка отправки";
        console.warn("failed to send initial pose", error);
      } finally {
        initialPoseButton.disabled = false;
      }
    }

    async function sendNavGoal() {
      const preview = navGoalPreviewPayload();
      if (!preview) {
        navGoalStatus.textContent = "выберите goal-точку";
        return;
      }

      navGoalButton.disabled = true;
      navGoalStatus.textContent = "отправка...";
      try {
        const response = await fetch("/nav-goal.json", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            reference_point: {
              x: preview.target_point.course_x,
              y: preview.target_point.course_y
            },
            coordinate_frame: "course",
            yaw: preview.pose.yaw,
            yaw_deg: preview.pose.yaw_deg,
            frame_id: preview.frame_id,
            target_point: preview.target_point
          })
        });
        const payload = await response.json();
        if (!response.ok) {
          throw new Error(payload.error || `HTTP ${response.status}`);
        }
        mapData.nav_goal = {
          command: payload.nav_goal,
          status: payload.nav_goal_status || mapData.nav_goal?.status
        };
        updateNavGoalPreview();
        renderMap();
        navGoalStatus.textContent = `queued #${payload.nav_goal?.seq ?? "-"}`;
      } catch (error) {
        navGoalStatus.textContent = "ошибка отправки";
        console.warn("failed to send nav goal", error);
      } finally {
        navGoalButton.disabled = false;
      }
    }

    function renderTable() {
      const origin = originSelect.value;
      pointsTable.innerHTML = mapData.reference_points
        .map((point) => {
          const display = displayPoint(point, origin);
          return `<tr><td>${point.id}</td><td>${fixed(display.x)}</td><td>${fixed(display.y)}</td></tr>`;
        })
        .join("");
    }

    function renderSignsTable() {
      const signs = signBySlot();
      signsTable.innerHTML = mapData.sign_slots
        .map((slot) => {
          const sign = signs.get(slot.id);
          const meta = sign ? signTypeMeta(sign.type) : null;
          const label = sign ? `${meta.short} ${Number(sign.confidence ?? 0).toFixed(2)}` : "-";
          return `<tr><td>${slot.id}</td><td>${slot.side_code}${slot.index}</td><td>${label}</td></tr>`;
        })
        .join("");
    }

    function renderMap() {
      if (!mapData) {
        return;
      }

      const origin = originSelect.value;
      const showLabels = labelsToggle.checked;
      const showMarking = markingToggle.checked;
      const showSignSlots = signSlotsToggle.checked;
      const showRecognizedSigns = recognizedSignsToggle.checked;
      const showScan = scanToggle.checked;
      const showBaseLink = baseLinkToggle.checked;
      const size = mapData.field.size_m;
      const cells = mapData.field.grid_cells;
      const cell = mapData.field.cell_size_m;
      const pad = 72;
      const fieldPx = 720;
      const scale = fieldPx / size;
      const viewBox = `0 0 ${fieldPx + pad * 2} ${fieldPx + pad * 2}`;
      const blocked = sectionSet(mapData);
      const recognizedSigns = signBySlot();
      const robotState = mapData.robot_state || {};
      const pose = currentMapPose();
      const goalPoint = selectedGoalPoint();
      const goalKey = pointKey(goalPoint);

      let svg = `<svg viewBox="${viewBox}" role="img" aria-label="Упрощенная карта города">`;
      svg += `<defs>
        <marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
          <path d="M0,0 L8,4 L0,8 Z" fill="var(--accent)"></path>
        </marker>
        <marker id="goal-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
          <path d="M0,0 L8,4 L0,8 Z" fill="var(--goal)"></path>
        </marker>
      </defs>`;
      svg += `<rect x="0" y="0" width="${fieldPx + pad * 2}" height="${fieldPx + pad * 2}" fill="#ffffff"></rect>`;
      svg += `<rect x="${pad}" y="${pad}" width="${fieldPx}" height="${fieldPx}" rx="2" fill="var(--road)" stroke="var(--road-edge)" stroke-width="3"></rect>`;

      for (let row = 0; row < cells; row += 1) {
        for (let col = 0; col < cells; col += 1) {
          const rect = sectionToRect({ col, row }, pad, scale, size, cell);
          const key = `${col}:${row}`;
          if (blocked.has(key)) {
            svg += `<rect x="${rect.x}" y="${rect.y}" width="${rect.width}" height="${rect.height}" fill="var(--grass)" stroke="#527d45" stroke-width="1"></rect>`;
            const inset = rect.width * 0.19;
            svg += `<rect x="${rect.x + inset}" y="${rect.y + inset}" width="${rect.width - inset * 2}" height="${rect.height - inset * 2}" rx="4" fill="var(--building)" stroke="#8f733d" stroke-width="2"></rect>`;
            svg += `<text x="${rect.x + rect.width / 2}" y="${rect.y + rect.height / 2 + 5}" text-anchor="middle" font-size="24" font-weight="760" fill="#332a1b">${col},${row}</text>`;
          }
          svg += `<rect x="${rect.x}" y="${rect.y}" width="${rect.width}" height="${rect.height}" fill="none" stroke="rgba(255,255,255,0.32)" stroke-width="1"></rect>`;
        }
      }

      if (showMarking) {
        for (let row = 0; row < cells; row += 1) {
          for (let col = 0; col < cells; col += 1) {
            if (blocked.has(`${col}:${row}`)) {
              continue;
            }
            const rect = sectionToRect({ col, row }, pad, scale, size, cell);
            const cx = rect.x + rect.width / 2;
            const cy = rect.y + rect.height / 2;
            svg += `<line x1="${rect.x + 12}" y1="${cy}" x2="${rect.x + rect.width - 12}" y2="${cy}" stroke="var(--marking)" stroke-width="5" stroke-linecap="round" stroke-dasharray="24 18" opacity="0.88"></line>`;
            svg += `<line x1="${cx}" y1="${rect.y + 12}" x2="${cx}" y2="${rect.y + rect.height - 12}" stroke="var(--marking)" stroke-width="5" stroke-linecap="round" stroke-dasharray="24 18" opacity="0.88"></line>`;
          }
        }
      }

      for (let i = 0; i <= cells; i += 1) {
        const meter = i * cell;
        const x = pad + meter * scale;
        const y = pad + fieldPx - meter * scale;
        const xLabel = displayPoint({ x: meter, y: coordOffset().y || 0 }, origin).x;
        const yLabel = displayPoint({ x: coordOffset().x || 0, y: meter }, origin).y;
        svg += `<line x1="${x}" y1="${pad + fieldPx}" x2="${x}" y2="${pad + fieldPx + 7}" stroke="#55606b" stroke-width="1"></line>`;
        svg += `<line x1="${pad - 7}" y1="${y}" x2="${pad}" y2="${y}" stroke="#55606b" stroke-width="1"></line>`;
        svg += `<text x="${x}" y="${pad + fieldPx + 27}" text-anchor="middle" font-size="15" fill="#55606b">${fixed(xLabel)}</text>`;
        svg += `<text x="${pad - 14}" y="${y + 5}" text-anchor="end" font-size="15" fill="#55606b">${fixed(yLabel)}</text>`;
      }

      mapData.reference_points.forEach((point) => {
        const pos = physicalToSvg(point.x, point.y, pad, scale, size);
        const display = displayPoint(point, origin);
        const label = `${fixed(display.x)}; ${fixed(display.y)}`;
        const key = pointKey(point);
        const isGoal = key === goalKey;
        const radius = isGoal ? 8.6 : 5.3;
        const fill = isGoal ? "var(--goal)" : "var(--point)";
        const stroke = isGoal ? "#ffffff" : "var(--point-border)";
        const strokeWidth = isGoal ? 2.6 : 1.4;
        svg += `<circle data-reference-point="${key}" cx="${pos.x}" cy="${pos.y}" r="${radius}" fill="${fill}" stroke="${stroke}" stroke-width="${strokeWidth}" style="cursor:pointer"><title>${point.id}: ${label}</title></circle>`;
        if (showLabels) {
          svg += `<text x="${pos.x + 8}" y="${pos.y - 7}" font-size="11" fill="#fff" stroke="#26313a" stroke-width="3" paint-order="stroke">${label}</text>`;
        }
      });

      if (goalPoint) {
        const goalPos = physicalToSvg(goalPoint.x, goalPoint.y, pad, scale, size);
        const yaw = navYawRad();
        const goalNose = physicalToSvg(
          goalPoint.x + Math.cos(yaw) * 0.25,
          goalPoint.y + Math.sin(yaw) * 0.25,
          pad,
          scale,
          size
        );
        svg += `<line x1="${goalPos.x}" y1="${goalPos.y}" x2="${goalNose.x}" y2="${goalNose.y}" stroke="#ffffff" stroke-width="8" stroke-linecap="round"></line>`;
        svg += `<line x1="${goalPos.x}" y1="${goalPos.y}" x2="${goalNose.x}" y2="${goalNose.y}" stroke="var(--goal)" stroke-width="4" stroke-linecap="round" marker-end="url(#goal-arrow)"><title>goal: ${pointLabel(goalPoint)}, ${normalizedYawDeg()} deg</title></line>`;
        svg += `<text x="${goalPos.x}" y="${goalPos.y - 15}" text-anchor="middle" font-size="12" font-weight="760" fill="var(--goal)" stroke="#ffffff" stroke-width="3" paint-order="stroke">GOAL</text>`;
      }

      if (showSignSlots || showRecognizedSigns) {
        mapData.sign_slots.forEach((slot) => {
          const pos = physicalToSvg(slot.x, slot.y, pad, scale, size);
          const normal = signSlotNormal(slot.side);
          const sign = recognizedSigns.get(slot.id);
          const display = displayPoint(slot, origin);
          const coordLabel = `${fixed(display.x)}; ${fixed(display.y)}`;

          if (showSignSlots) {
            svg += `<circle cx="${pos.x}" cy="${pos.y}" r="7" fill="var(--sign-slot-fill)" stroke="var(--sign-slot)" stroke-width="2"><title>${slot.id}: ${coordLabel}</title></circle>`;
            svg += `<line x1="${pos.x}" y1="${pos.y}" x2="${pos.x + normal.dx * 14}" y2="${pos.y + normal.dy * 14}" stroke="var(--sign-slot)" stroke-width="2" stroke-linecap="round"></line>`;
          }

          if (showRecognizedSigns && sign) {
            const meta = signTypeMeta(sign.type);
            const markerX = pos.x + normal.dx * 17;
            const markerY = pos.y + normal.dy * 17;
            const title = `${slot.id}: ${meta.label}, ${coordLabel}, conf ${Number(sign.confidence ?? 0).toFixed(2)}`;
            svg += `<g transform="translate(${markerX},${markerY})">`;
            svg += `<rect x="-13" y="-13" width="26" height="26" rx="5" fill="${meta.color}" stroke="#ffffff" stroke-width="2"><title>${title}</title></rect>`;
            svg += `<text x="0" y="4" text-anchor="middle" font-size="${meta.short.length > 2 ? 8 : 11}" font-weight="760" fill="#ffffff">${meta.short}</text>`;
            svg += `</g>`;
          }
        });
      }

      if (showScan && Array.isArray(mapData.scan?.points) && mapData.scan.points.length > 0) {
        const scanPoints = [];
        mapData.scan.points.forEach((point) => {
          const mapPoint = scanPointToMap(point, pose, mapData.scan);
          if (!mapPoint || mapPoint.x < 0 || mapPoint.x > size || mapPoint.y < 0 || mapPoint.y > size) {
            return;
          }
          const svgPoint = physicalToSvg(mapPoint.x, mapPoint.y, pad, scale, size);
          scanPoints.push(`${svgPoint.x.toFixed(1)},${svgPoint.y.toFixed(1)}`);
        });
        if (scanPoints.length > 1) {
          svg += `<polyline points="${scanPoints.join(" ")}" fill="none" stroke="#38bdf8" stroke-width="2" opacity="0.78"></polyline>`;
        }
        scanPoints.forEach((pair) => {
          const [x, y] = pair.split(",");
          svg += `<circle cx="${x}" cy="${y}" r="2.3" fill="#0891b2" opacity="0.78"></circle>`;
        });
      }

      if (showBaseLink && pose) {
        const robotPos = physicalToSvg(pose.x, pose.y, pad, scale, size);
        const nose = physicalToSvg(
          pose.x + Math.cos(pose.yaw || 0) * 0.22,
          pose.y + Math.sin(pose.yaw || 0) * 0.22,
          pad,
          scale,
          size
        );
        svg += `<line x1="${robotPos.x}" y1="${robotPos.y}" x2="${nose.x}" y2="${nose.y}" stroke="#ffffff" stroke-width="8" stroke-linecap="round"></line>`;
        svg += `<line x1="${robotPos.x}" y1="${robotPos.y}" x2="${nose.x}" y2="${nose.y}" stroke="var(--robot)" stroke-width="4" stroke-linecap="round"></line>`;
        const robotTitleX = Number.isFinite(pose.mapX) ? pose.mapX : courseToMap(pose).x;
        const robotTitleY = Number.isFinite(pose.mapY) ? pose.mapY : courseToMap(pose).y;
        svg += `<circle cx="${robotPos.x}" cy="${robotPos.y}" r="13" fill="var(--robot)" stroke="#ffffff" stroke-width="3"><title>base_link: ${fixed(robotTitleX)}; ${fixed(robotTitleY)}</title></circle>`;
        svg += `<text x="${robotPos.x}" y="${robotPos.y + 4}" text-anchor="middle" font-size="10" font-weight="760" fill="#ffffff">BL</text>`;

        if (robotState.visible_point) {
          const visiblePos = physicalToSvg(robotState.visible_point.x, robotState.visible_point.y, pad, scale, size);
          svg += `<line x1="${robotPos.x}" y1="${robotPos.y}" x2="${visiblePos.x}" y2="${visiblePos.y}" stroke="var(--visible)" stroke-width="4" stroke-linecap="round" stroke-dasharray="12 8"></line>`;
          svg += `<circle cx="${visiblePos.x}" cy="${visiblePos.y}" r="9" fill="var(--visible)" stroke="#ffffff" stroke-width="2"><title>visible: ${pointLabel(robotState.visible_point)}</title></circle>`;
        }
      }

      const axis = axisGeometry(origin, pad, fieldPx, scale, size);
      svg += `<line x1="${axis.ox}" y1="${axis.oy}" x2="${axis.x2}" y2="${axis.y2}" stroke="var(--accent)" stroke-width="3" marker-end="url(#arrow)"></line>`;
      svg += `<line x1="${axis.ox}" y1="${axis.oy}" x2="${axis.yx}" y2="${axis.yy}" stroke="var(--accent)" stroke-width="3" marker-end="url(#arrow)"></line>`;
      svg += `<circle cx="${axis.ox}" cy="${axis.oy}" r="8" fill="var(--accent)" stroke="#fff" stroke-width="3"></circle>`;
      svg += `<text x="${axis.ox}" y="${axis.oy - 14}" text-anchor="middle" font-size="16" font-weight="760" fill="var(--accent)">0,0</text>`;
      svg += `<text x="${axis.xx}" y="${axis.xy}" text-anchor="middle" font-size="18" font-weight="760" fill="var(--accent)">X</text>`;
      svg += `<text x="${axis.yx + (axis.yx === pad ? -24 : 24)}" y="${axis.yy + 5}" text-anchor="middle" font-size="18" font-weight="760" fill="var(--accent)">Y</text>`;
      svg += `</svg>`;

      mapRoot.innerHTML = svg;
      renderTable();
      renderSignsTable();
    }

    async function boot() {
      const response = await fetch("/map-data.json", { cache: "no-store" });
      mapData = await response.json();

      document.getElementById("field-size").textContent = `${fixed(mapData.field.size_m)} м`;
      document.getElementById("section-count").textContent = `${mapData.field.grid_cells} x ${mapData.field.grid_cells}`;
      document.getElementById("point-count").textContent = `${mapData.reference_points.length} точек`;
      document.getElementById("slot-count").textContent = `${mapData.sign_slots.length} слотов`;
      document.getElementById("cell-size").textContent = `${fixed(mapData.field.cell_size_m)} м`;
      document.getElementById("point-step").textContent = `${fixed(mapData.reference_grid.step_m)} м`;
      document.getElementById("first-point").textContent = pointLabel({
        x: mapData.reference_grid.start_m,
        y: mapData.reference_grid.start_m
      });
      document.getElementById("blocked-count").textContent = String(mapData.blocked_sections.length);
      document.getElementById("slot-count-details").textContent = String(mapData.sign_slots.length);
      updateSignCounts();
      telemetryJson.textContent = JSON.stringify({
        robot_state: mapData.robot_state,
        scan: mapData.scan,
        pose: mapData.pose,
        initial_pose: mapData.initial_pose,
        nav_goal: mapData.nav_goal,
        detection_source: mapData.detection_source
      }, null, 2);

      originSelect.value = mapData.origin.default_corner;
      cameraImage.src = mapData.camera?.stream_url || "/camera-stream.mjpg";
      populateRobotControls();
      populateGoalControls();
      renderMap();
      renderRviz();
    }

    async function refreshSigns() {
      if (!mapData) {
        return;
      }

      try {
        const response = await fetch(`/signs.json?ts=${Date.now()}`, { cache: "no-store" });
        const payload = await response.json();
        mapData.recognized_signs = payload.signs || [];
        mapData.robot_state = payload.robot_state || mapData.robot_state;
        mapData.detection_source = payload.detection_source || mapData.detection_source;
        mapData.scan = payload.scan || mapData.scan;
        mapData.pose = payload.pose || mapData.pose;
        mapData.costmaps = payload.costmaps || mapData.costmaps;
        mapData.tf = payload.tf || mapData.tf;
        mapData.initial_pose = payload.initial_pose || mapData.initial_pose;
        mapData.nav_goal = payload.nav_goal || mapData.nav_goal;
        updateSignCounts();
        updateNavGoalPreview();
        telemetryJson.textContent = JSON.stringify({
          robot_state: mapData.robot_state,
          scan: mapData.scan,
          pose: mapData.pose,
          initial_pose: mapData.initial_pose,
          nav_goal: mapData.nav_goal,
          detection_source: mapData.detection_source
        }, null, 2);
        syncRobotControls();
        renderMap();
        renderRviz();
      } catch (error) {
        console.warn("failed to refresh signs", error);
      }
    }

    async function refreshDetections() {
      try {
        const response = await fetch(`/detections.json?ts=${Date.now()}`, { cache: "no-store" });
        const payload = await response.json();
        lastDetections = payload;
        detectionsJson.textContent = JSON.stringify(payload, null, 2);
        const count = Array.isArray(payload.detections) ? payload.detections.length : 0;
        const seq = payload.seq !== undefined ? `seq ${payload.seq}` : (payload.status || "no seq");
        cameraStatus.textContent = `${seq}, ${count} detections${formatAge(payload)}`;
        drawDetections(payload);
      } catch (error) {
        detectionsJson.textContent = `Failed to load /detections.json\n${error}`;
        cameraStatus.textContent = "offline";
        drawDetections(null);
      }
    }

    originSelect.addEventListener("change", renderMap);
    labelsToggle.addEventListener("change", renderMap);
    markingToggle.addEventListener("change", renderMap);
    signSlotsToggle.addEventListener("change", renderMap);
    recognizedSignsToggle.addEventListener("change", renderMap);
    scanToggle.addEventListener("change", renderMap);
    baseLinkToggle.addEventListener("change", renderMap);
    robotPointSelect.addEventListener("change", sendRobotState);
    robotHeadingSelect.addEventListener("change", sendRobotState);
    initialPoseButton.addEventListener("click", sendInitialPoseEstimate);
    navGoalPointSelect.addEventListener("change", () => {
      updateNavGoalPreview();
      renderMap();
    });
    navYawDegInput.addEventListener("input", () => {
      updateNavGoalPreview();
      renderMap();
    });
    navGoalButton.addEventListener("click", sendNavGoal);
    rvizTabs.forEach((button) => {
      button.addEventListener("click", () => selectRvizTab(button.dataset.rvizTab));
    });
    mapRoot.addEventListener("click", (event) => {
      let node = event.target;
      while (node && node !== mapRoot) {
        if (node.dataset && node.dataset.referencePoint) {
          navGoalPointSelect.value = node.dataset.referencePoint;
          updateNavGoalPreview();
          renderMap();
          return;
        }
        node = node.parentNode;
      }
    });
    cameraImage.addEventListener("load", () => drawDetections(lastDetections));
    window.addEventListener("resize", () => {
      renderMap();
      renderRviz();
      drawDetections(lastDetections);
    });
    boot().catch((error) => {
      mapRoot.textContent = `Ошибка загрузки карты: ${error}`;
    });
    setInterval(refreshSigns, 1000);
    refreshRviz();
    setInterval(refreshRviz, 1000);
    refreshDetections();
    setInterval(refreshDetections, 500);
  </script>
</body>
</html>
"""


def reference_values():
    value = REFERENCE_START_M
    while value < FIELD_SIZE_M:
        yield round(value, 3)
        value += REFERENCE_STEP_M


def is_blocked(x, y):
    col = int(x // CELL_SIZE_M)
    row = int(y // CELL_SIZE_M)
    return (col, row) in BLOCKED_SECTIONS


def build_reference_points():
    points = []
    for y in reference_values():
        for x in reference_values():
            if is_blocked(x, y):
                continue
            points.append({
                "id": f"P{len(points) + 1:03d}",
                "x": round(x, 3),
                "y": round(y, 3),
            })
    return points


def point_key(point):
    return f"{point['x']:.1f},{point['y']:.1f}"


def reference_point_lookup():
    return {point_key(point): point for point in build_reference_points()}


def find_reference_point(x, y):
    key = f"{round(float(x), 1):.1f},{round(float(y), 1):.1f}"
    return reference_point_lookup().get(key)


def reference_point_from_payload(payload, coordinate_frame=None):
    if not isinstance(payload, dict):
        return None

    if "course_x" in payload and "course_y" in payload:
        return find_reference_point(payload["course_x"], payload["course_y"])

    if "x" not in payload or "y" not in payload:
        return None

    frame = str(coordinate_frame or payload.get("coordinate_frame", "")).strip().lower()
    if frame == "course":
        return find_reference_point(payload["x"], payload["y"])

    if frame == "map":
        course_x, course_y = map_to_course_xy(payload["x"], payload["y"])
        return find_reference_point(course_x, course_y)

    reference_point = find_reference_point(payload["x"], payload["y"])
    if reference_point is not None:
        return reference_point

    course_x, course_y = map_to_course_xy(payload["x"], payload["y"])
    return find_reference_point(course_x, course_y)


def build_sign_slots():
    slots = []
    side_specs = (
        ("west", "W"),
        ("east", "E"),
        ("south", "S"),
        ("north", "N"),
    )

    for col, row in BLOCKED_SECTIONS:
        base_x = col * CELL_SIZE_M
        base_y = row * CELL_SIZE_M
        for side, side_code in side_specs:
            for index, along in enumerate(SIGN_SLOT_SIDE_POSITIONS_M, start=1):
                if side == "west":
                    local_x = SIGN_SLOT_EDGE_OFFSET_M
                    local_y = along
                    facing_deg = 180
                elif side == "east":
                    local_x = CELL_SIZE_M - SIGN_SLOT_EDGE_OFFSET_M
                    local_y = along
                    facing_deg = 0
                elif side == "south":
                    local_x = along
                    local_y = SIGN_SLOT_EDGE_OFFSET_M
                    facing_deg = 270
                else:
                    local_x = along
                    local_y = CELL_SIZE_M - SIGN_SLOT_EDGE_OFFSET_M
                    facing_deg = 90

                slots.append({
                    "id": f"H{col}{row}-{side_code}{index}",
                    "house": {"col": col, "row": row},
                    "side": side,
                    "side_code": side_code,
                    "index": index,
                    "x": round(base_x + local_x, 3),
                    "y": round(base_y + local_y, 3),
                    "local_x": round(local_x, 3),
                    "local_y": round(local_y, 3),
                    "facing_deg": facing_deg,
                })
    return slots


def nearest_slot_for_reference_point(point):
    if not point:
        return None

    best_slot = None
    best_distance = None
    for slot in build_sign_slots():
        distance = ((slot["x"] - point["x"]) ** 2 + (slot["y"] - point["y"]) ** 2) ** 0.5
        if best_distance is None or distance < best_distance:
            best_slot = slot
            best_distance = distance

    if best_distance is not None and best_distance <= SIGN_VISIBILITY_RADIUS_M:
        return best_slot
    return None


def compute_visible_point(reference_point, heading):
    if not reference_point or heading not in HEADING_VECTORS:
        return None

    dx, dy = HEADING_VECTORS[heading]
    return find_reference_point(reference_point["x"] + dx, reference_point["y"] + dy)


def current_robot_state():
    with STATE_LOCK:
        return json.loads(json.dumps(ROBOT_STATE))


def update_robot_state(payload):
    if not isinstance(payload, dict):
        payload = {}

    point_payload = payload.get("reference_point") or payload.get("point") or payload
    reference_point = None
    if isinstance(point_payload, dict):
        reference_point = reference_point_from_payload(point_payload, payload.get("coordinate_frame"))

    heading = payload.get("heading")
    if heading is not None:
        heading = str(heading).strip().lower()
        if heading not in HEADING_VECTORS:
            heading = None

    visible_payload = payload.get("visible_point") or payload.get("target_point")
    visible_point = None
    if isinstance(visible_payload, dict) and "x" in visible_payload and "y" in visible_payload:
        visible_point = reference_point_from_payload(visible_payload, payload.get("coordinate_frame"))
    elif reference_point and heading:
        visible_point = compute_visible_point(reference_point, heading)

    visible_slot = nearest_slot_for_reference_point(visible_point)
    state = {
        "reference_point": reference_point,
        "heading": heading,
        "visible_point": visible_point,
        "visible_slot_id": visible_slot["id"] if visible_slot else None,
        "updated_unix": time.time(),
    }

    with STATE_LOCK:
        ROBOT_STATE.update(state)
        return json.loads(json.dumps(ROBOT_STATE))


def known_slot_ids():
    return {slot["id"] for slot in build_sign_slots()}


def normalize_signs(payload):
    raw_signs = payload.get("signs", payload) if isinstance(payload, dict) else payload
    if not isinstance(raw_signs, list):
        return []

    slot_ids = known_slot_ids()
    normalized = []
    seen_slots = set()
    for raw in raw_signs:
        if not isinstance(raw, dict):
            continue

        slot_id = str(raw.get("slot_id", "")).strip()
        if slot_id not in slot_ids or slot_id in seen_slots:
            continue

        sign_type = str(raw.get("type", "unknown")).strip()
        if sign_type not in SIGN_TYPES:
            sign_type = "unknown"

        try:
            confidence = float(raw.get("confidence", 0))
        except (TypeError, ValueError):
            confidence = 0.0

        normalized.append({
            "slot_id": slot_id,
            "type": sign_type,
            "confidence": max(0.0, min(1.0, confidence)),
            "source": str(raw.get("source", "manual")).strip() or "manual",
            "class_name": str(raw.get("class_name", "")).strip(),
            "seen_from": raw.get("seen_from"),
            "visible_point": raw.get("visible_point"),
            "stamp_unix": raw.get("stamp_unix"),
        })
        seen_slots.add(slot_id)

    return normalized


def read_recognized_signs():
    with STATE_LOCK:
        return json.loads(json.dumps(SIGN_MEMORY))


def write_recognized_signs(signs):
    normalized = normalize_signs({"signs": signs})
    with STATE_LOCK:
        SIGN_MEMORY[:] = normalized
        return json.loads(json.dumps(SIGN_MEMORY))


def update_detection_source(**kwargs):
    with STATE_LOCK:
        DETECTION_SOURCE_STATE.update(kwargs)
        return json.loads(json.dumps(DETECTION_SOURCE_STATE))


def current_detection_source():
    with STATE_LOCK:
        return json.loads(json.dumps(DETECTION_SOURCE_STATE))


def read_detection_payload():
    try:
        with urllib.request.urlopen(DETECTIONS_URL, timeout=DETECTIONS_TIMEOUT_S) as response:
            data = response.read(256 * 1024)
        payload = json.loads(data.decode("utf-8", "replace"))
        update_detection_source(
            status="ok",
            last_seq=payload.get("seq"),
            last_stamp_unix=payload.get("stamp_unix"),
            last_error=None,
        )
        return payload
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        update_detection_source(status="offline", last_error=str(exc))
        return None


def detection_to_sign_type(detection):
    class_name = str(detection.get("class_name", "")).strip().lower()
    return YOLO_CLASS_TO_SIGN_TYPE.get(class_name)


def best_sign_detection(payload):
    if not isinstance(payload, dict):
        return None

    detections = payload.get("detections")
    if not isinstance(detections, list):
        return None

    candidates = [
        detection for detection in detections
        if isinstance(detection, dict) and detection_to_sign_type(detection)
    ]
    def confidence(item):
        try:
            return float(item.get("confidence", 0) or 0)
        except (TypeError, ValueError):
            return 0.0

    return max(candidates, key=confidence, default=None)


def remember_detection_for_current_slot(payload):
    state = current_robot_state()
    slot_id = state.get("visible_slot_id")
    if not slot_id:
        return False

    detection = best_sign_detection(payload)
    if not detection:
        return False

    sign_type = detection_to_sign_type(detection)
    seq = payload.get("seq")
    source_state = current_detection_source()
    if seq is not None and source_state.get("ingested_seq") == seq:
        return False

    try:
        confidence = float(detection.get("confidence", 0))
    except (TypeError, ValueError):
        confidence = 0.0

    new_sign = {
        "slot_id": slot_id,
        "type": sign_type,
        "confidence": max(0.0, min(1.0, confidence)),
        "source": "yolo",
        "class_name": str(detection.get("class_name", "")),
        "seen_from": state.get("reference_point"),
        "visible_point": state.get("visible_point"),
        "stamp_unix": payload.get("stamp_unix", time.time()),
    }

    with STATE_LOCK:
        by_slot = {sign["slot_id"]: sign for sign in SIGN_MEMORY}
        previous = by_slot.get(slot_id)
        if previous is None or new_sign["confidence"] >= float(previous.get("confidence", 0) or 0):
            by_slot[slot_id] = new_sign
            SIGN_MEMORY[:] = list(by_slot.values())
        DETECTION_SOURCE_STATE["ingested_seq"] = seq
    return True


def refresh_signs_from_detection_source():
    payload = read_detection_payload()
    if payload is None:
        return False
    return remember_detection_for_current_slot(payload)


def read_json_file(path, fallback):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return fallback
    except Exception as exc:
        data = dict(fallback)
        data["status"] = "error"
        data["error"] = str(exc)
        return data


def write_json_file(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(path.name + ".tmp")
    temp_path.write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    os.replace(temp_path, path)


def read_scan_data():
    return read_json_file(SCAN_PATH, {
        "schema": "tb3_web_scan.v1",
        "status": "no_data",
        "path": str(SCAN_PATH),
        "frame_id": None,
        "points_frame": "base_link",
        "points": [],
    })


def read_pose_data():
    return read_json_file(POSE_PATH, {
        "schema": "tb3_web_pose.v1",
        "status": "no_data",
        "path": str(POSE_PATH),
        "pose": None,
    })


def read_costmap_data():
    return read_json_file(COSTMAP_PATH, {
        "schema": "tb3_web_costmaps.v1",
        "status": "no_data",
        "path": str(COSTMAP_PATH),
        "costmaps": {
            "global": {"status": "no_data", "data": []},
            "local": {"status": "no_data", "data": []},
        },
    })


def read_tf_data():
    return read_json_file(TF_PATH, {
        "schema": "tb3_web_tf.v1",
        "status": "no_data",
        "path": str(TF_PATH),
        "watched_transforms": [],
        "frames_text": "",
    })


def read_initial_pose_command():
    return read_json_file(INITIAL_POSE_COMMAND_PATH, {
        "schema": "tb3_initial_pose_command.v1",
        "status": "no_command",
        "path": str(INITIAL_POSE_COMMAND_PATH),
        "pose": None,
    })


def read_nav_goal_command():
    return read_json_file(NAV_GOAL_COMMAND_PATH, {
        "schema": "tb3_nav_goal_command.v1",
        "status": "no_command",
        "path": str(NAV_GOAL_COMMAND_PATH),
        "pose": None,
    })


def read_nav_goal_status():
    return read_json_file(NAV_GOAL_STATUS_PATH, {
        "schema": "tb3_nav_goal_status.v1",
        "status": "no_status",
        "path": str(NAV_GOAL_STATUS_PATH),
    })


def normalize_initial_pose_payload(payload):
    if not isinstance(payload, dict):
        raise ValueError("payload must be a JSON object")

    point_payload = payload.get("reference_point") or payload.get("point") or payload
    if not isinstance(point_payload, dict):
        point_payload = {}

    reference_point = reference_point_from_payload(point_payload, payload.get("coordinate_frame"))
    if reference_point is not None:
        map_x, map_y = course_to_map_xy(reference_point["x"], reference_point["y"])
    else:
        try:
            map_x = float(point_payload["x"])
            map_y = float(point_payload["y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("x and y are required numbers") from exc

    course_x, course_y = map_to_course_xy(map_x, map_y)
    if not (0.0 <= course_x <= FIELD_SIZE_M and 0.0 <= course_y <= FIELD_SIZE_M):
        raise ValueError(f"x and y must be inside shifted map bounds for the {FIELD_SIZE_M:.1f} m field")

    heading = payload.get("heading", DEFAULT_START_POINT["heading"])
    if heading is not None:
        heading = str(heading).strip().lower()
    if heading not in HEADING_YAWS:
        heading = None

    if heading:
        yaw = HEADING_YAWS[heading]
    else:
        try:
            yaw = float(payload["yaw"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("heading or yaw is required") from exc

    frame_id = str(payload.get("frame_id", INITIAL_POSE_FRAME)).strip() or INITIAL_POSE_FRAME
    stamp_unix = time.time()
    return {
        "schema": "tb3_initial_pose_command.v1",
        "status": "queued",
        "seq": time.time_ns(),
        "stamp_unix": stamp_unix,
        "frame_id": frame_id,
        "heading": heading,
        "pose": {
            "x": round(map_x, 4),
            "y": round(map_y, 4),
            "yaw": round(yaw, 6),
        },
    }


def write_initial_pose_command(payload):
    command = normalize_initial_pose_payload(payload)
    write_json_file(INITIAL_POSE_COMMAND_PATH, command)
    return command


def normalize_nav_goal_payload(payload):
    if not isinstance(payload, dict):
        raise ValueError("payload must be a JSON object")

    point_payload = payload.get("target_point") or payload.get("reference_point") or payload.get("point") or payload
    if not isinstance(point_payload, dict):
        point_payload = {}

    reference_point = reference_point_from_payload(point_payload, payload.get("coordinate_frame"))
    if reference_point is None and isinstance(payload.get("reference_point"), dict):
        reference_point = reference_point_from_payload(payload["reference_point"], "course")
    if reference_point is None:
        raise ValueError("goal must be one of the allowed reference points")

    heading = payload.get("heading")
    if heading is not None:
        heading = str(heading).strip().lower()
    if heading in HEADING_YAWS:
        yaw = HEADING_YAWS[heading]
    else:
        try:
            yaw = float(payload["yaw"])
        except (KeyError, TypeError, ValueError):
            try:
                yaw = math.radians(float(payload["yaw_deg"]))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("goal yaw, yaw_deg or heading is required") from exc

    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    map_x, map_y = course_to_map_xy(reference_point["x"], reference_point["y"])
    frame_id = str(payload.get("frame_id", INITIAL_POSE_FRAME)).strip() or INITIAL_POSE_FRAME
    stamp_unix = time.time()
    return {
        "schema": "tb3_nav_goal_command.v1",
        "status": "queued",
        "seq": time.time_ns(),
        "stamp_unix": stamp_unix,
        "action": NAV_GOAL_ACTION,
        "frame_id": frame_id,
        "target_point": reference_point,
        "pose": {
            "x": round(map_x, 4),
            "y": round(map_y, 4),
            "yaw": round(yaw, 6),
            "yaw_deg": round(math.degrees(yaw), 3),
            "orientation": orientation_from_yaw(yaw),
        },
    }


def write_nav_goal_command(payload):
    command = normalize_nav_goal_payload(payload)
    write_json_file(NAV_GOAL_COMMAND_PATH, command)
    return command


def build_map_data():
    return {
        "schema": "tb3_city_map.v1",
        "field": {
            "size_m": FIELD_SIZE_M,
            "grid_cells": GRID_CELLS,
            "cell_size_m": CELL_SIZE_M,
        },
        "origin": {
            "default_corner": "bottom-left",
            "section_index_base": 0,
            "section_index_origin": "bottom-left",
        },
        "coordinate_frame": {
            "nav_frame": INITIAL_POSE_FRAME,
            "origin_offset_m": {
                "x": MAP_ORIGIN_OFFSET_X_M,
                "y": MAP_ORIGIN_OFFSET_Y_M,
            },
            "description": "map/Nav2 coordinates are course coordinates minus this offset",
        },
        "reference_grid": {
            "start_m": REFERENCE_START_M,
            "step_m": REFERENCE_STEP_M,
        },
        "blocked_sections": [
            {"col": col, "row": row} for col, row in BLOCKED_SECTIONS
        ],
        "reference_points": build_reference_points(),
        "sign_slot_layout": {
            "edge_offset_m": SIGN_SLOT_EDGE_OFFSET_M,
            "side_positions_m": list(SIGN_SLOT_SIDE_POSITIONS_M),
            "slots_per_house": 8,
        },
        "sign_types": SIGN_TYPES,
        "sign_slots": build_sign_slots(),
        "recognized_signs": read_recognized_signs(),
        "robot_state": current_robot_state(),
        "detection_source": current_detection_source(),
        "scan": read_scan_data(),
        "pose": read_pose_data(),
        "costmaps": read_costmap_data(),
        "tf": read_tf_data(),
        "initial_pose": read_initial_pose_command(),
        "nav_goal": {
            "command": read_nav_goal_command(),
            "status": read_nav_goal_status(),
        },
        "camera": {
            "stream_url": "/camera-stream.mjpg",
            "source_stream_url": CAMERA_STREAM_URL,
            "detections_url": "/detections.json",
            "source_detections_url": DETECTIONS_URL,
        },
    }


class CityMapHandler(BaseHTTPRequestHandler):
    server_version = "TurtleBotCityMap/0.1"

    def log_message(self, fmt, *args):
        print(f"{self.client_address[0]} - {fmt % args}", flush=True)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            self.send_index()
        elif path == "/camera-stream.mjpg":
            self.send_camera_stream()
        elif path == "/detections.json":
            self.send_detections()
        elif path == "/map-data.json":
            self.send_map_data()
        elif path == "/signs.json":
            self.send_signs()
        elif path == "/scan.json":
            self.send_scan()
        elif path == "/pose.json":
            self.send_pose()
        elif path == "/costmaps.json":
            self.send_costmaps()
        elif path == "/tf.json":
            self.send_tf()
        elif path == "/rviz-data.json":
            self.send_rviz_data()
        elif path == "/robot-state.json":
            self.send_robot_state()
        elif path == "/initial-pose.json":
            self.send_initial_pose()
        elif path == "/nav-goal.json":
            self.send_nav_goal()
        elif path == "/healthz":
            self.send_text("ok\n")
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/signs.json":
            self.receive_signs()
        elif path == "/robot-state.json":
            self.receive_robot_state()
        elif path == "/initial-pose.json":
            self.receive_initial_pose()
        elif path == "/nav-goal.json":
            self.receive_nav_goal()
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def send_index(self):
        data = INDEX_HTML.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_map_data(self):
        self.send_json(build_map_data())

    def send_detections(self):
        payload = read_detection_payload()
        if payload is None:
            payload = {
                "schema": "tb3_yolo_detections.v1",
                "status": "no_data",
                "source": DETECTIONS_URL,
                "detections": [],
                "best_detection": None,
            }
        self.send_json(payload)

    def send_scan(self):
        self.send_json(read_scan_data())

    def send_pose(self):
        self.send_json(read_pose_data())

    def send_costmaps(self):
        self.send_json(read_costmap_data())

    def send_tf(self):
        self.send_json(read_tf_data())

    def send_rviz_data(self):
        self.send_json({
            "schema": "tb3_web_rviz_data.v1",
            "scan": read_scan_data(),
            "pose": read_pose_data(),
            "costmaps": read_costmap_data(),
            "tf": read_tf_data(),
            "nav_goal": {
                "command": read_nav_goal_command(),
                "status": read_nav_goal_status(),
            },
        })

    def send_initial_pose(self):
        self.send_json(read_initial_pose_command())

    def send_nav_goal(self):
        self.send_json({
            "schema": "tb3_nav_goal.v1",
            "command": read_nav_goal_command(),
            "status": read_nav_goal_status(),
        })

    def send_json(self, payload, status=HTTPStatus.OK):
        data = (json.dumps(payload, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_camera_stream(self):
        try:
            upstream = urllib.request.urlopen(CAMERA_STREAM_URL, timeout=CAMERA_PROXY_TIMEOUT_S)
        except Exception as exc:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, f"camera stream unavailable: {exc}")
            return

        self.send_response(HTTPStatus.OK)
        content_type = upstream.headers.get("Content-Type", "multipart/x-mixed-replace")
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        try:
            while True:
                chunk = upstream.read(16384)
                if not chunk:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            upstream.close()

    def send_signs(self):
        refresh_signs_from_detection_source()
        payload = {
            "schema": "tb3_city_signs.v1",
            "storage": "memory",
            "detections_url": DETECTIONS_URL,
            "signs": read_recognized_signs(),
            "robot_state": current_robot_state(),
            "detection_source": current_detection_source(),
            "scan": read_scan_data(),
            "pose": read_pose_data(),
            "costmaps": read_costmap_data(),
            "tf": read_tf_data(),
            "initial_pose": read_initial_pose_command(),
            "nav_goal": {
                "command": read_nav_goal_command(),
                "status": read_nav_goal_status(),
            },
        }
        self.send_json(payload)

    def send_robot_state(self):
        payload = {
            "schema": "tb3_city_robot_state.v1",
            "robot_state": current_robot_state(),
        }
        self.send_json(payload)

    def receive_signs(self):
        payload = self.read_json_body()
        if payload is None:
            return

        signs = write_recognized_signs(payload.get("signs", payload) if isinstance(payload, dict) else payload)
        response = {
            "schema": "tb3_city_signs.v1",
            "saved": len(signs),
            "storage": "memory",
            "signs": signs,
        }
        self.send_json(response)

    def receive_robot_state(self):
        payload = self.read_json_body()
        if payload is None:
            return

        state = update_robot_state(payload)
        response = {
            "schema": "tb3_city_robot_state.v1",
            "robot_state": state,
        }
        self.send_json(response)

    def receive_initial_pose(self):
        payload = self.read_json_body()
        if payload is None:
            return

        try:
            command = write_initial_pose_command(payload)
        except ValueError as exc:
            self.send_json(
                {
                    "schema": "tb3_initial_pose_command.v1",
                    "status": "error",
                    "error": str(exc),
                },
                HTTPStatus.BAD_REQUEST,
            )
            return

        state_payload = {
            "x": command["pose"]["x"],
            "y": command["pose"]["y"],
            "coordinate_frame": "map",
        }
        if command.get("heading"):
            state_payload["heading"] = command["heading"]
        state = update_robot_state(state_payload)
        response = {
            "schema": "tb3_initial_pose_command.v1",
            "status": "queued",
            "initial_pose": command,
            "robot_state": state,
        }
        self.send_json(response)

    def receive_nav_goal(self):
        payload = self.read_json_body()
        if payload is None:
            return

        try:
            command = write_nav_goal_command(payload)
        except ValueError as exc:
            self.send_json(
                {
                    "schema": "tb3_nav_goal_command.v1",
                    "status": "error",
                    "error": str(exc),
                },
                HTTPStatus.BAD_REQUEST,
            )
            return

        response = {
            "schema": "tb3_nav_goal_command.v1",
            "status": "queued",
            "nav_goal": command,
            "nav_goal_status": read_nav_goal_status(),
        }
        self.send_json(response)

    def read_json_body(self):
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            content_length = 0

        if content_length <= 0 or content_length > 128 * 1024:
            self.send_error(HTTPStatus.BAD_REQUEST, "invalid body length")
            return

        try:
            payload = json.loads(self.rfile.read(content_length).decode("utf-8"))
        except Exception:
            self.send_error(HTTPStatus.BAD_REQUEST, "invalid JSON")
            return None
        return payload

    def send_text(self, text):
        data = text.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


class CityMapServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    httpd = CityMapServer((HOST, PORT), CityMapHandler)

    def shutdown(signum, frame):
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    print(f"Serving TurtleBot3 city map on http://{HOST}:{PORT}/", flush=True)
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
