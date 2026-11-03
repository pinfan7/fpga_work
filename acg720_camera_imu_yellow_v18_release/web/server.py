"""FPGA FPG1 UDP receiver and localhost-only MJPEG web viewer."""
import argparse
import array
import collections
import io
import json
import socket
import struct
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from PIL import Image

HEADER = struct.Struct('!4sIHHHH')

def newer(a, b):
    return 0 < ((a - b) & 0xffffffff) < 0x80000000

def decode565(raw, width, height):
    words = array.array('H')
    words.frombytes(raw)
    if sys.byteorder == 'little':
        words.byteswap()
    # Pillow's BGR;16 decoder consumes native little-endian RGB565 words.
    if sys.byteorder == 'big':
        words.byteswap()
    return Image.frombytes('RGB', (width, height), words.tobytes(), 'raw', 'BGR;16')

class Assembler:
    def __init__(self):
        self.frames = collections.OrderedDict()
        self.last_id = None
        self.packets = self.invalid = self.incomplete = self.complete = 0
        self.last_packet = 0
        self.dimensions = (0, 0)

    def feed(self, packet):
        self.packets += 1
        now = time.monotonic()
        if len(packet) < HEADER.size:
            self.invalid += 1
            return
        magic, fid, row, w, h, fmt = HEADER.unpack_from(packet)
        if magic != b'FPG1' or (w, h) != (640, 480) or fmt != 1 or row >= h or len(packet) != 16 + w*2:
            self.invalid += 1
            return
        # A gap also permits the FPGA to restart its frame counter after download.
        if self.last_packet and now-self.last_packet>2:
            self.last_id = None
            self.frames.clear()
        self.last_packet = now
        self.dimensions = (w, h)
        for old in list(self.frames):
            if now - self.frames[old][3] > 1:
                del self.frames[old]
                self.incomplete += 1
        if self.last_id is not None and not newer(fid, self.last_id):
            return
        if fid not in self.frames:
            while len(self.frames) >= 4:
                self.frames.popitem(last=False)
                self.incomplete += 1
            self.frames[fid] = [bytearray(w*h*2), bytearray(h), 0, now]
        f = self.frames[fid]
        if not f[1][row]:
            f[0][row*w*2:(row+1)*w*2] = packet[16:]
            f[1][row] = 1
            f[2] += 1
        if f[2] == h:
            self.complete += 1
            self.last_id = fid
            raw = bytes(f[0])
            for old in list(self.frames):
                if old == fid or not newer(old, fid):
                    if old != fid:
                        self.incomplete += 1
                    del self.frames[old]
            return fid, w, h, raw

class Receiver:
    def __init__(self, port=6102, fps=15):
        self.assembler = Assembler()
        self.condition = threading.Condition()
        self.pending = None
        self.jpeg = None
        self.version = 0
        self.error = ''
        self.source = ''
        self.last_image = 0
        self.times = collections.deque(maxlen=60)
        self.fps_limit = fps
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8*1024*1024)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.5)
        self.running = True

    def start(self):
        for target in (self.receive, self.encode):
            threading.Thread(target=target, daemon=True).start()

    def receive(self):
        while self.running:
            try:
                packet, addr = self.sock.recvfrom(65535)
                frame = self.assembler.feed(packet)
                self.source = addr[0]
                if frame:
                    with self.condition:
                        self.pending = frame
                        self.condition.notify_all()
            except socket.timeout:
                pass
            except OSError as exc:
                if self.running:
                    self.error = str(exc)
                return

    def encode(self):
        previous = 0
        while self.running:
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or not self.running, timeout=1)
                if not self.running:
                    return
                if self.pending is None:
                    continue
            time.sleep(max(0, 1/self.fps_limit - (time.monotonic()-previous)))
            with self.condition:
                frame, self.pending = self.pending, None
            try:
                fid, w, h, raw = frame
                image = decode565(raw, w, h)
                buf = io.BytesIO()
                image.save(buf, 'JPEG', quality=85)
                with self.condition:
                    self.jpeg = buf.getvalue()
                    self.version += 1
                    self.last_image = previous = time.monotonic()
                    self.times.append(previous)
                    self.condition.notify_all()
            except Exception as exc:
                self.error = str(exc)

    def status(self):
        a = self.assembler
        now = time.monotonic()
        age = now-self.last_image if self.last_image else None
        recent = [t for t in self.times if now-t < 3]
        fps = (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0
        return dict(live=age is not None and age<2, age=age, fps=round(fps,1),
                    packets=a.packets, invalid=a.invalid, complete=a.complete,
                    incomplete=a.incomplete, source=self.source, error=self.error,
                    width=a.dimensions[0], height=a.dimensions[1], frame_id=a.last_id)

def make_handler(receiver):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/':
                body = Path(__file__).with_name('index.html').read_bytes()
                mime = 'text/html; charset=utf-8'
            elif path == '/api/status':
                body = json.dumps(receiver.status()).encode()
                mime = 'application/json'
            elif path == '/snapshot.jpg':
                body = receiver.jpeg
                mime = 'image/jpeg'
                if not body:
                    self.send_error(503, 'No complete frame received')
                    return
            elif path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                version = -1
                try:
                    while receiver.running:
                        with receiver.condition:
                            receiver.condition.wait_for(lambda: receiver.version != version and receiver.jpeg is not None, timeout=2)
                            if receiver.jpeg is None or receiver.version == version:
                                continue
                            jpeg, version = receiver.jpeg, receiver.version
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                        self.wfile.flush()
                except (ConnectionError, OSError):
                    pass
                return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    return Handler

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--udp-port', type=int, default=6102)
    parser.add_argument('--fps', type=int, default=15, choices=range(1,31))
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    receiver = Receiver(args.udp_port, args.fps)
    http = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(receiver))
    receiver.start()
    url = f'http://127.0.0.1:{args.port}'
    print(f'Web: {url}\nUDP: 0.0.0.0:{args.udp_port}\nPC Ethernet: 192.168.0.3 / 255.255.255.0\nPress Ctrl+C to stop.', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        http.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        receiver.running = False
        receiver.sock.close()
        with receiver.condition:
            receiver.condition.notify_all()
        http.server_close()

if __name__ == '__main__':
    main()
