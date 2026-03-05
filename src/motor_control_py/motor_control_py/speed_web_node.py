import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


INDEX_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>速度实时曲线</title>
  <style>
    :root {
      --bg: #f3f7fb;
      --card: #ffffff;
      --grid: #d7dee8;
      --target: #1b84f2;
      --feedback: #f25c1b;
      --text: #243041;
    }
    body {
      margin: 0;
      font-family: "Noto Sans SC", "Microsoft YaHei", sans-serif;
      background: radial-gradient(circle at 10% 10%, #e9f3ff, var(--bg));
      color: var(--text);
    }
    .wrap {
      max-width: 1100px;
      margin: 24px auto;
      padding: 0 16px;
    }
    .card {
      background: var(--card);
      border-radius: 14px;
      box-shadow: 0 8px 20px rgba(10, 40, 80, 0.08);
      padding: 16px;
    }
    h1 {
      margin: 4px 0 12px 0;
      font-size: 22px;
    }
    .status {
      margin-bottom: 10px;
      font-size: 14px;
    }
    .dot {
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 999px;
      margin-right: 8px;
      vertical-align: baseline;
      background: #9aa4b1;
    }
    canvas {
      width: 100%;
      height: 420px;
      border-radius: 10px;
      background: #fbfdff;
      border: 1px solid #e8edf4;
    }
    .legend {
      margin-top: 10px;
      font-size: 14px;
    }
    .legend span {
      display: inline-block;
      margin-right: 14px;
    }
    .line-box {
      display: inline-block;
      width: 22px;
      height: 3px;
      margin-right: 6px;
      vertical-align: middle;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>速度实时曲线</h1>
      <div class="status"><span id="dot" class="dot"></span><span id="txt">连接中...</span></div>
      <canvas id="chart" width="1200" height="420"></canvas>
      <div class="legend">
        <span><i class="line-box" style="background: var(--target)"></i>目标速度 target</span>
        <span><i class="line-box" style="background: var(--feedback)"></i>反馈速度 feedback</span>
        <span id="last">--</span>
      </div>
    </div>
  </div>
<script>
const windowSec = 12.0;
const points = [];
const canvas = document.getElementById('chart');
const ctx = canvas.getContext('2d');
const dot = document.getElementById('dot');
const txt = document.getElementById('txt');
const last = document.getElementById('last');

function setStatus(ok, msg) {
  dot.style.background = ok ? '#23a453' : '#c75b23';
  txt.textContent = msg;
}

function pushPoint(p) {
  points.push(p);
  const now = p.t;
  while (points.length && points[0].t < now - windowSec) {
    points.shift();
  }
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!points.length) {
    requestAnimationFrame(draw);
    return;
  }

  const now = points[points.length - 1].t;
  const x0 = now - windowSec;
  let yMin = Infinity;
  let yMax = -Infinity;
  for (const p of points) {
    yMin = Math.min(yMin, p.target, p.feedback);
    yMax = Math.max(yMax, p.target, p.feedback);
  }
  const m = Math.max(0.05, (yMax - yMin) * 0.2);
  yMin -= m;
  yMax += m;
  if (Math.abs(yMax - yMin) < 0.01) {
    yMax += 0.1;
    yMin -= 0.1;
  }

  const w = canvas.width;
  const h = canvas.height;
  const padL = 50;
  const padR = 20;
  const padT = 20;
  const padB = 34;
  const pw = w - padL - padR;
  const ph = h - padT - padB;

  const sx = (t) => padL + ((t - x0) / windowSec) * pw;
  const sy = (v) => padT + (1 - (v - yMin) / (yMax - yMin)) * ph;

  ctx.strokeStyle = '#d7dee8';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 5; i++) {
    const y = padT + (i / 5) * ph;
    ctx.beginPath();
    ctx.moveTo(padL, y);
    ctx.lineTo(w - padR, y);
    ctx.stroke();
  }

  function drawLine(key, color) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    let started = false;
    for (const p of points) {
      const x = sx(p.t);
      const y = sy(p[key]);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  }

  drawLine('target', '#1b84f2');
  drawLine('feedback', '#f25c1b');

  ctx.fillStyle = '#516173';
  ctx.font = '14px sans-serif';
  ctx.fillText(`${yMax.toFixed(2)} m/s`, 8, padT + 5);
  ctx.fillText(`${yMin.toFixed(2)} m/s`, 8, padT + ph);
  ctx.fillText(`-${windowSec.toFixed(0)}s`, padL, h - 10);
  ctx.fillText('0s', w - padR - 20, h - 10);

  const p = points[points.length - 1];
  last.textContent = `target=${p.target.toFixed(3)}  feedback=${p.feedback.toFixed(3)}  err=${(p.target - p.feedback).toFixed(3)} m/s`;

  requestAnimationFrame(draw);
}

const es = new EventSource('/stream');
es.onopen = () => setStatus(true, '已连接');
es.onerror = () => setStatus(false, '连接中断，等待重连...');
es.onmessage = (ev) => {
  try {
    const p = JSON.parse(ev.data);
    pushPoint(p);
  } catch (_) {}
};

requestAnimationFrame(draw);
</script>
</body>
</html>
"""


class SpeedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {'t': time.time(), 'target': 0.0, 'feedback': 0.0}
        self.version = 0

    def update(self, target: float, feedback: float):
        with self.lock:
            self.latest = {'t': time.time(), 'target': target, 'feedback': feedback}
            self.version += 1

    def snapshot(self):
        with self.lock:
            return self.latest.copy(), self.version


class SpeedWebNode(Node):
    def __init__(self, state: SpeedState):
        super().__init__('speed_web_node')
        self.state = state

        self.cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel').value
        self.feedback_topic = self.declare_parameter('feedback_topic', 'linear_velocity').value
        self.cmd_vel_axis = str(self.declare_parameter('cmd_vel_axis', 'x').value).lower()
        self.cmd_vel_scale = float(self.declare_parameter('cmd_vel_scale', 1.0).value)
        self.web_host = str(self.declare_parameter('web_host', '0.0.0.0').value)
        self.web_port = int(self.declare_parameter('web_port', 8080).value)
        self.sample_hz = float(self.declare_parameter('sample_hz', 20.0).value)

        if self.cmd_vel_axis not in ('x', 'y', 'z'):
            raise RuntimeError("cmd_vel_axis must be one of: 'x', 'y', 'z'")
        if self.sample_hz <= 0.0:
            raise RuntimeError('sample_hz must be > 0')

        self.current_target = 0.0
        self.current_feedback = 0.0
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_subscription(Float64, self.feedback_topic, self.on_feedback, 10)
        self.create_timer(1.0 / self.sample_hz, self.on_sample_timer)

    def on_cmd(self, msg: Twist):
        axis_value = {
            'x': float(msg.linear.x),
            'y': float(msg.linear.y),
            'z': float(msg.linear.z),
        }[self.cmd_vel_axis]
        self.current_target = axis_value * self.cmd_vel_scale

    def on_feedback(self, msg: Float64):
        self.current_feedback = float(msg.data)

    def on_sample_timer(self):
        self.state.update(self.current_target, self.current_feedback)


def make_handler(state: SpeedState):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                body = INDEX_HTML.encode('utf-8')
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == '/stream':
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                last_version = -1
                try:
                    while True:
                        payload, version = state.snapshot()
                        if version != last_version:
                            message = f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
                            self.wfile.write(message.encode('utf-8'))
                            self.wfile.flush()
                            last_version = version
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    return

            self.send_error(HTTPStatus.NOT_FOUND, 'Not Found')

        def log_message(self, format, *args):
            return

    return Handler


def main():
    rclpy.init()
    state = SpeedState()
    node = SpeedWebNode(state)

    handler_cls = make_handler(state)
    server = ThreadingHTTPServer((node.web_host, node.web_port), handler_cls)
    server.daemon_threads = True
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    node.get_logger().info(
        f'speed_web started at http://{node.web_host}:{node.web_port} '
        f'cmd_topic={node.cmd_topic} feedback_topic={node.feedback_topic}'
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
