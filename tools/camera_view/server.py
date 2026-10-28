#!/usr/bin/env python3
"""One JPEG per second from a robot camera, served to a browser over a tunnel.

Only the selected camera runs ffmpeg, at 1 fps and 640 px wide, so the tunnel
carries roughly 20-40 kB/s. Streams are probed at startup and every
`probe_interval_s`; cameras that are not publishing are shown as unavailable.
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent


class Camera:
    """Runs one ffmpeg MJPEG pipeline per active camera; keeps only the last frame."""

    def __init__(self, name, url, settings):
        self.name, self.url, self.settings = name, url, settings
        self.lock = threading.Lock()
        self.frame = None
        self.frame_at = 0.0
        self.last_view = 0.0
        self.error = None
        self.process = None
        self.available = None
        self.probed_at = 0.0
        self.thread = None

    def probe(self):
        """Cheap availability check: does the stream answer DESCRIBE?"""
        import subprocess
        command = ['ffprobe', '-v', 'error', '-rtsp_transport', 'tcp',
                   '-select_streams', 'v:0', '-show_entries', 'stream=width,height',
                   '-of', 'csv=p=0', self.url]
        self.probed_at = time.monotonic()
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=8)
            self.available = result.returncode == 0 and bool(result.stdout.strip())
        except (subprocess.TimeoutExpired, OSError):
            self.available = False
        return self.available

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._run, name='camera-' + self.name, daemon=True)
        self.thread.start()

    def touch(self):
        self.last_view = time.monotonic()

    def idle(self):
        return time.monotonic() - self.last_view > self.settings['idle_stop_s']

    def snapshot(self):
        with self.lock:
            return self.frame, self.frame_at

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

    def _run(self):
        import subprocess
        command = ['ffmpeg', '-nostdin', '-loglevel', 'error', '-rtsp_transport', 'tcp',
                   '-i', self.url, '-an',
                   '-vf', 'fps=%d,scale=%d:-2' % (self.settings['fps'], self.settings['width']),
                   '-q:v', str(self.settings['jpeg_quality']), '-f', 'mjpeg', 'pipe:1']
        while True:
            if self.idle():
                break
            self.error = None
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
                while True:
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
            except Exception as error:  # keep the service alive; the page shows the error
                self.error = str(error)
            finally:
                self.stop()
            if self.idle():
                break
            self.error = self.error or 'stream ended; retrying'
            time.sleep(2)
        with self.lock:
            self.frame = None
        self.thread = None


class Service:
    def __init__(self, config):
        self.config = config
        self.cameras = {
            name: Camera(name, url, config) for name, url in config['cameras'].items()
        }
        self.lock = threading.Lock()
        threading.Thread(target=self._probe_loop, daemon=True).start()

    def _probe_loop(self):
        while True:
            for camera in self.cameras.values():
                if not camera.thread or not camera.thread.is_alive():
                    camera.probe()
            time.sleep(self.config['probe_interval_s'])

    def camera(self, name):
        return self.cameras.get(name)

    def status(self):
        return {
            'cameras': [
                {'name': camera.name, 'url': camera.url, 'available': camera.available,
                 'running': bool(camera.thread and camera.thread.is_alive()),
                 'age_s': round(time.monotonic() - camera.frame_at, 2) if camera.frame_at else None,
                 'error': camera.error}
                for camera in self.cameras.values()
            ]
        }


HTML = """<!doctype html><meta charset="utf-8"><title>山猫相机</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}
 header{padding:10px 14px;display:flex;gap:8px;align-items:center;flex-wrap:wrap;background:#1b1b1b}
 button{padding:6px 14px;border:1px solid #444;background:#222;color:#eee;border-radius:6px;cursor:pointer}
 button.active{background:#2b6cb0;border-color:#2b6cb0}
 button:disabled{opacity:.35;cursor:not-allowed}
 img{display:block;width:100%;max-width:1024px;margin:10px auto;background:#000;min-height:200px}
 small{opacity:.75}
</style>
<header><strong>山猫相机</strong><span id="cams"></span><small id="info">连接中…</small></header>
<img id="frame" alt="camera">
<script>
let current = null, timer = null, status = {};
async function fetchWithTimeout(url, ms){
  const control = new AbortController();
  const timer = setTimeout(()=>control.abort(), ms);
  try { return await fetch(url, {cache:'no-store', signal: control.signal}); }
  finally { clearTimeout(timer); }
}
async function loadStatus(){
  const info = document.getElementById('info');
  try{
    const r = await fetchWithTimeout('api/cameras', 5000);
    const s = await r.json();
    status = s; renderButtons(s.cameras);
    if(!current){
      const first = s.cameras.find(c => c.available);
      if(first){ select(first.name); }
      else { info.textContent = '当前没有可用的相机流（推流端未上线）'; }
    }
  }catch(e){
    info.textContent = '后台不可达：请确认 SSH 隧道已加 -L 8080:127.0.0.1:8080（' + location.host + '）';
    document.getElementById('cams').innerHTML = '';
  }
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
async function grab(){
  if(!current) return;
  const started = performance.now();
  try{
    const r = await fetchWithTimeout('frame?cam='+encodeURIComponent(current)+'&t='+Date.now(), 8000);
    if(!r.ok){ document.getElementById('info').textContent = current+'：'+r.status+' '+(await r.text()); return; }
    const blob = await r.blob();
    const url = URL.createObjectURL(blob);
    const img = document.getElementById('frame');
    const old = img.src; img.src = url;
    if(old.startsWith('blob:')) setTimeout(()=>URL.revokeObjectURL(old), 2000);
    document.getElementById('info').textContent =
      current+' · '+Math.round(blob.size/1024)+' kB · '+
      Math.round(performance.now()-started)+' ms · '+new Date().toLocaleTimeString();
  }catch(e){ document.getElementById('info').textContent = current+'：'+e; }
}
function select(name){
  current = name;
  document.getElementById('info').textContent = name+' 加载中…';
  renderButtons(status.cameras||[]);
  if(timer) clearInterval(timer);
  grab(); timer = setInterval(grab, 1000);
}
loadStatus(); setInterval(loadStatus, 10000);
</script>"""


class Handler(BaseHTTPRequestHandler):
    server_version = 'lynx-camera/1.0'
    service = None

    def log_message(self, *_):
        pass

    def _send(self, code, body, content_type='application/json'):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ('/', '/index.html'):
            self._send(200, HTML.encode(), 'text/html; charset=utf-8')
            return
        if parsed.path == '/api/cameras':
            self._send(200, json.dumps(self.service.status()).encode())
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
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                frame, stamp = camera.snapshot()
                if frame and time.monotonic() - stamp < 5:
                    self._send(200, frame, 'image/jpeg'); return
                time.sleep(0.1)
            self._send(504, (camera.error or 'no frame yet').encode(), 'text/plain'); return
        self._send(404, b'not found', 'text/plain')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'config.json'))
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    service = Service(config)
    Handler.service = service
    server = ThreadingHTTPServer((config['listen_host'], config['listen_port']), Handler)
    print('camera view on http://%s:%d (tunnel 8080 -> this port)' % (
        config['listen_host'], config['listen_port']), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for camera in service.cameras.values():
            camera.stop()
        server.server_close()


if __name__ == '__main__':
    main()
