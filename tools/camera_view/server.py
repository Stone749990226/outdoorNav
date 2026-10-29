#!/usr/bin/env python3
"""One JPEG per tick from a robot camera, served to a browser over a tunnel.

Only the selected camera runs ffmpeg. Resolution, quality and update interval
are adjustable from the page (or the config file) to trade picture for
bandwidth; the default 640 px / 1 Hz costs roughly 20-40 kB/s.
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:
    from .battery import BatteryMonitor
except ImportError:
    from battery import BatteryMonitor

ROOT = Path(__file__).resolve().parent
# One slider trades picture quality for bandwidth. Each anchor is
# (level, width_px, ffmpeg -q:v); values between anchors are interpolated.
QUALITY_ANCHORS = [(0, 240, 28), (25, 320, 20), (50, 480, 14), (75, 800, 9), (100, 1280, 5)]
LEVEL_RANGE = (0, 100)
INTERVAL_RANGE = (0.1, 5.0)


def level_settings(level):
    level = max(LEVEL_RANGE[0], min(LEVEL_RANGE[1], float(level)))
    for (l0, w0, q0), (l1, w1, q1) in zip(QUALITY_ANCHORS, QUALITY_ANCHORS[1:]):
        if level <= l1:
            t = (level - l0) / (l1 - l0)
            return int(round(w0 + (w1 - w0) * t)), int(round(q0 + (q1 - q0) * t))
    return QUALITY_ANCHORS[-1][1], QUALITY_ANCHORS[-1][2]


class Camera:
    def __init__(self, name, url, settings):
        self.name, self.url = name, url
        self.settings = dict(settings, width=settings.get('width', 480),
                             jpeg_quality=settings.get('jpeg_quality', 14),
                             fps=settings.get('fps', 1.0))
        self.lock = threading.Lock()
        self.frame = None
        self.frame_at = 0.0
        self.frame_wall = 0.0
        self.last_view = 0.0
        self.error = None
        self.process = None
        self.available = None
        self.restart = False
        self.thread = None

    def probe(self):
        import subprocess
        command = ['ffprobe', '-v', 'error', '-rtsp_transport', 'tcp',
                   '-select_streams', 'v:0', '-show_entries', 'stream=width,height',
                   '-of', 'csv=p=0', self.url]
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=8)
            self.available = result.returncode == 0 and bool(result.stdout.strip())
        except (subprocess.TimeoutExpired, OSError):
            self.available = False
        return self.available

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return
        if self.process is not None and self.process.poll() is None:
            self.stop()          # never run two pipelines for one camera
        self.restart = False
        self.thread = threading.Thread(target=self._run, name='camera-' + self.name, daemon=True)
        self.thread.start()

    def touch(self):
        self.last_view = time.monotonic()

    def idle(self):
        return time.monotonic() - self.last_view > self.settings['idle_stop_s']

    def snapshot(self):
        with self.lock:
            return self.frame, self.frame_at, self.frame_wall

    def stop(self):
        process, self.process = self.process, None
        if process is not None:
            try:
                process.terminate()
                process.wait(timeout=3)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass

    def restart_pipeline(self):
        """Apply new settings by ending the current ffmpeg; the next view respawns it."""
        self.restart = True
        self.stop()
        thread = self.thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        self.thread = None
        with self.lock:
            self.frame = None
            self.frame_at = 0.0

    def _run(self):
        import subprocess
        while not (self.idle() or self.restart):
            self.error = None
            command = ['ffmpeg', '-nostdin', '-loglevel', 'error', '-rtsp_transport', 'tcp',
                       '-i', self.url, '-an',
                       '-vf', 'fps=%g,scale=%d:-2' % (self.settings['fps'], self.settings['width']),
                       '-q:v', str(self.settings['jpeg_quality']), '-f', 'mjpeg', 'pipe:1']
            try:
                self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                                stderr=subprocess.PIPE, bufsize=0)
            except OSError as error:
                self.error = str(error)
                time.sleep(2)
                continue
            buffer = b''
            try:
                stream = self.process.stdout
                while not self.restart:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buffer += chunk
                    while True:
                        start = buffer.find(b'\xff\xd8')
                        end = buffer.find(b'\xff\xd9', start + 2) if start >= 0 else -1
                        if start < 0 or end < 0:
                            buffer = buffer[-65536:]
                            break
                        frame = buffer[start:end + 2]
                        buffer = buffer[end + 2:]
                        with self.lock:
                            self.frame, self.frame_at = frame, time.monotonic()
                            self.frame_wall = time.time()
            except Exception as error:
                self.error = str(error)
            finally:
                self.stop()
            if self.restart or self.idle():
                break
            self.error = self.error or 'stream ended; retrying'
            time.sleep(2)
        with self.lock:
            self.frame = None
        self.thread = None


class Service:
    def __init__(self, config):
        self.config = config
        self.battery = BatteryMonitor(config.get('battery_host', '10.21.33.103'),
                                      config.get('battery_port', 30004))
        self.cameras = {name: Camera(name, url, config)
                        for name, url in config['cameras'].items()}
        current = self.settings()
        for camera in self.cameras.values():
            camera.settings.update({'width': current['width'],
                                    'jpeg_quality': current['jpeg_quality'],
                                    'fps': current['fps']})
        threading.Thread(target=self._probe_loop, daemon=True).start()

    def _probe_loop(self):
        while True:
            for camera in self.cameras.values():
                if not camera.thread or not camera.thread.is_alive():
                    camera.probe()
            time.sleep(self.config['probe_interval_s'])

    def camera(self, name):
        return self.cameras.get(name)

    def settings(self):
        width, quality = level_settings(self.config.get('level', 55))
        interval = max(INTERVAL_RANGE[0], min(INTERVAL_RANGE[1], self.config.get('interval_s', 1.0)))
        return {'level': round(float(self.config.get('level', 55)), 1),
                'interval_s': round(float(interval), 3),
                'width': width, 'jpeg_quality': quality,
                'fps': round(1.0 / interval, 3)}

    def update_settings(self, values):
        changes = {}
        if 'level' in values:
            value = values['level']
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('level must be a number')
            if not LEVEL_RANGE[0] <= value <= LEVEL_RANGE[1]:
                raise ValueError('level must be %g..%g' % LEVEL_RANGE)
            changes['level'] = float(value)
        if 'interval_s' in values:
            value = values['interval_s']
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('interval_s must be a number')
            if not INTERVAL_RANGE[0] <= value <= INTERVAL_RANGE[1]:
                raise ValueError('interval_s must be %g..%g s' % INTERVAL_RANGE)
            changes['interval_s'] = float(value)
        if not changes:
            raise ValueError('expected level and/or interval_s')
        # Cameras read width/quality/fps from this live mapping.
        self.config.update(changes)
        width, quality = level_settings(self.config['level'])
        interval = self.config['interval_s']
        for camera in self.cameras.values():
            camera.settings.update({'width': width, 'jpeg_quality': quality, 'fps': 1.0 / interval})
            camera.restart_pipeline()
        return {'settings': self.settings()}

    def status(self):
        return {
            'settings': self.settings(),
            'cameras': [
                {'name': camera.name, 'url': camera.url, 'available': camera.available,
                 'running': bool(camera.thread and camera.thread.is_alive()),
                 'age_s': round(time.monotonic() - camera.frame_at, 2) if camera.frame_at else None,
                 'frame_time': round(camera.frame_wall, 3) if camera.frame_wall else None,
                 'error': camera.error}
                for camera in self.cameras.values()
            ],
        }


HTML = """<!doctype html><meta charset="utf-8"><title>山猫相机</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}
 header{padding:10px 14px;display:flex;gap:8px;align-items:center;flex-wrap:wrap;background:#1b1b1b}
 .controls{padding:8px 14px;display:flex;gap:18px;align-items:center;flex-wrap:wrap;background:#161616;font-size:13px}
 #battery{padding:5px 10px;border:1px solid #526259;border-radius:6px;color:#b9ec9f}
 #battery.low{color:#ffbf69;border-color:#ffbf69}
 #battery.unavailable{color:#a8b1b6;border-color:#526259}
 .controls label{display:flex;gap:8px;align-items:center}
 input[type=range]{width:180px;accent-color:#2b6cb0}
 button{padding:6px 14px;border:1px solid #444;background:#222;color:#eee;border-radius:6px;cursor:pointer}
 button.active{background:#2b6cb0;border-color:#2b6cb0}
 button:disabled{opacity:.35;cursor:not-allowed}
 img{display:block;width:100%;max-width:1280px;margin:10px auto;background:#000;min-height:200px}
 small{opacity:.75}
</style>
<header><strong>山猫相机</strong><span id="cams"></span><strong id="battery" role="status" aria-live="polite">电量读取中…</strong><small id="info">连接中…</small></header>
<div class="controls">
  <label>画质/带宽 <input id="level" type="range" min="0" max="100" step="1" value="55"><small id="levelText"></small></label>
  <label>更新间隔 <input id="interval" type="range" min="0" max="1000" step="1" value="500"><small id="intervalText"></small></label>
  <small id="rate"></small>
</div>
<img id="frame" alt="camera">
<script>
let current = null, timer = null, status = {};
let settings = {level:55, interval_s:1, width:480, jpeg_quality:14, fps:1};
const I_MIN = 0.1, I_MAX = 5.0, I_SPAN = Math.log(I_MAX / I_MIN);
const intervalFromSlider = v => I_MIN * Math.exp(I_SPAN * v / 1000);
const sliderFromInterval = s => Math.round(1000 * Math.log(s / I_MIN) / I_SPAN);
function fmtInterval(s){ return s >= 1 ? s.toFixed(2)+' 秒' : Math.round(s*1000)+' 毫秒'; }
async function fetchWithTimeout(url, ms, init){
  const control = new AbortController();
  const t = setTimeout(()=>control.abort(), ms);
  try { return await fetch(url, Object.assign({cache:'no-store', signal: control.signal}, init||{})); }
  finally { clearTimeout(t); }
}
function syncControls(){
  document.getElementById('level').value = String(Math.round(settings.level));
  document.getElementById('interval').value = String(sliderFromInterval(settings.interval_s));
  labels();
}
function labels(){
  const level = Number(document.getElementById('level').value);
  const interval = intervalFromSlider(Number(document.getElementById('interval').value));
  document.getElementById('levelText').textContent = level + ' / 100';
  document.getElementById('intervalText').textContent = fmtInterval(interval);
}
async function loadStatus(){
  const info = document.getElementById('info');
  try{
    const s = await (await fetchWithTimeout('api/cameras', 5000)).json();
    status = s; settings = s.settings || settings;
    renderButtons(s.cameras);
    if(!current) syncControls(), connectDefault(s);
    else labels();
  }catch(e){
    info.textContent = '后台不可达：请确认 SSH 隧道已加 -L 8080:127.0.0.1:8080（' + location.host + '）';
    document.getElementById('cams').innerHTML = '';
  }
}
function connectDefault(s){
  const first = s.cameras.find(c => c.available);
  if(first){ select(first.name); }
  else { document.getElementById('info').textContent = '当前没有可用的相机流（推流端未上线）'; }
}
function renderButtons(cams){
  const box = document.getElementById('cams'); box.innerHTML='';
  for(const c of cams){
    const b=document.createElement('button');
    b.textContent = c.name;
    b.disabled = !c.available;
    b.title = c.available ? c.url : '未接入';
    if(c.name===current) b.className='active';
    b.onclick = ()=>{ select(c.name); };
    box.appendChild(b);
  }
}
let applyTimer = null;
async function applySettings(immediate){
  const level = Number(document.getElementById('level').value);
  const interval_s = intervalFromSlider(Number(document.getElementById('interval').value));
  const send = async ()=>{
    try{
      const r = await fetchWithTimeout('api/settings', 8000,
        {method:'POST', headers:{'Content-Type':'application/json'},
         body: JSON.stringify({level:level, interval_s:interval_s})});
      if(!r.ok){ document.getElementById('info').textContent = '设置失败：'+ (await r.text()); return; }
      settings = (await r.json()).settings;
      rate();
      if(immediate) scheduleGrab();
    }catch(e){ document.getElementById('info').textContent = '设置失败：'+e; }
  };
  if(applyTimer) clearTimeout(applyTimer);
  if(immediate) return send();
  applyTimer = setTimeout(send, 350);
}
function rate(){
  const kb = window.lastFrameKb;
  if(kb){
    document.getElementById('rate').textContent =
      settings.width+' px · q'+settings.jpeg_quality+' · 每帧 '+kb+' kB · ≈'+
      Math.round(kb / settings.interval_s)+' kB/s';
  } else {
    document.getElementById('rate').textContent = settings.width+' px · q'+settings.jpeg_quality;
  }
}
async function grab(){
  if(!current) return;
  const started = performance.now();
  try{
    const r = await fetchWithTimeout('frame?cam='+encodeURIComponent(current)+'&t='+Date.now(), 9000);
    if(!r.ok){ document.getElementById('info').textContent = current+'：'+r.status+' '+(await r.text()); return; }
    const blob = await r.blob();
    const captured = Number(r.headers.get('X-Frame-Time') || 0) * 1000;
    const ageMs = Number(r.headers.get('X-Frame-Age-Ms') || 0);
    const url = URL.createObjectURL(blob);
    const img = document.getElementById('frame');
    const old = img.src; img.src = url;
    if(old.startsWith('blob:')) setTimeout(()=>URL.revokeObjectURL(old), 2000);
    window.lastFrameKb = Math.round(blob.size/1024*10)/10;
    rate();
    const capturedText = captured
      ? new Date(captured).toLocaleTimeString('zh-CN', {hour12:false}) : '未知';
    const e2e = captured ? Date.now() - captured : null;
    document.getElementById('info').textContent =
      current+' · 采集 '+capturedText+' · 山猫侧帧龄 '+ageMs+' ms'+
      (e2e !== null && e2e >= 0 && e2e < 60000 ? ' · 端到端 '+e2e+' ms'
        : (e2e !== null ? ' · 端到端不可用（两机时钟不同步）' : ''))+
      ' · 请求 '+Math.round(performance.now()-started)+' ms · '+
      window.lastFrameKb+' kB';
  }catch(e){ document.getElementById('info').textContent = current+'：'+e; }
}
function scheduleGrab(){
  if(timer) clearTimeout(timer);
  const interval = settings.interval_s >= 0.1 ? settings.interval_s : 1;
  const tick = async ()=>{ await grab(); if(current) timer = setTimeout(tick, interval*1000); };
  timer = setTimeout(tick, 0);
}
function select(name){
  current = name;
  document.getElementById('info').textContent = name+' 加载中…';
  renderButtons(status.cameras||[]);
  scheduleGrab();
}
document.getElementById('level').oninput = ()=>{ labels();
  document.getElementById('rate').textContent = '应用新画质…'; };
document.getElementById('level').onchange = ()=>applySettings(true);
document.getElementById('interval').oninput = labels;
document.getElementById('interval').onchange = ()=>applySettings(true);
async function loadBattery(){
  const badge = document.getElementById('battery');
  try {
    const r = await fetchWithTimeout('api/battery', 3000);
    if(!r.ok) throw new Error('HTTP '+r.status);
    const b = await r.json();
    if(!b.available){
      badge.textContent = b.percent != null ? '电量已过期（上次 '+b.percent+'%）' : '电量暂不可用';
      badge.className = 'unavailable';
    } else {
      badge.textContent = '电量 '+b.percent+'%'+(b.charging === true ? ' · 充电中' : '');
      badge.className = b.percent <= 20 ? 'low' : '';
    }
    badge.title = '本体实际状态'+(b.age_s != null ? ' · '+b.age_s.toFixed(1)+' 秒前更新' : '')+
      (b.voltage_v != null ? ' · '+b.voltage_v.toFixed(1)+' V' : '');
  } catch(e) { badge.textContent = '电量连接中断'; badge.className = 'unavailable'; }
}
loadBattery(); setInterval(loadBattery, 5000);
loadStatus(); setInterval(loadStatus, 10000);
</script>"""


class Handler(BaseHTTPRequestHandler):
    server_version = 'lynx-camera/1.1'
    service = None

    def log_message(self, *_):
        pass

    def _send(self, code, body, content_type='application/json', extra=()):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        for name, value in extra:
            self.send_header(name, str(value))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ('/', '/index.html'):
            self._send(200, HTML.encode(), 'text/html; charset=utf-8')
            return
        if parsed.path == '/api/battery':
            self._send(200, json.dumps(self.service.battery.snapshot()).encode())
            return
        if parsed.path == '/api/cameras':
            self._send(200, json.dumps(self.service.status()).encode())
            return
        if parsed.path == '/api/settings':
            self._send(200, json.dumps({'settings': self.service.settings()}).encode())
            return
        if parsed.path == '/frame':
            name = parse_qs(parsed.query).get('cam', [''])[0]
            camera = self.service.camera(name)
            if camera is None:
                self._send(404, b'unknown camera', 'text/plain'); return
            if not camera.available:
                self._send(503, b'camera not publishing', 'text/plain'); return
            camera.touch()
            camera.start()
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                frame, stamp, wall = camera.snapshot()
                # A 5 s cadence produces frames that are legitimately seconds old.
                if frame and time.monotonic() - stamp < max(5.0, 2.5 / max(0.05, camera.settings.get('fps', 1.0))):
                    age_ms = int((time.monotonic() - stamp) * 1000)
                    self._send(200, frame, 'image/jpeg', extra=(
                        ('X-Frame-Time', '%.3f' % wall),
                        ('X-Frame-Age-Ms', age_ms),
                        ('X-Camera', camera.name)))
                    return
                time.sleep(0.1)
            self._send(504, (camera.error or 'no frame yet').encode(), 'text/plain'); return
        self._send(404, b'not found', 'text/plain')

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path != '/api/settings':
            self._send(404, b'not found', 'text/plain'); return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            values = json.loads(self.rfile.read(length) or b'{}')
            result = self.service.update_settings(values)
        except (ValueError, TypeError) as error:
            self._send(400, str(error).encode(), 'text/plain'); return
        self._send(200, json.dumps(result).encode())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'config.json'))
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    config.setdefault('jpeg_quality', 8)
    service = Service(config)
    Handler.service = service
    server = ThreadingHTTPServer((config['listen_host'], config['listen_port']), Handler)
    print('camera view on http://%s:%d' % (config['listen_host'], config['listen_port']), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        service.battery.stop()
        for camera in service.cameras.values():
            camera.stop()
        server.server_close()


if __name__ == '__main__':
    main()
