import io
import pathlib
import socket
import struct
import sys
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'web'))
from server import Assembler, Receiver, decode565, make_handler, newer
from PIL import Image

def packet(fid,row,color=b'\xf8\x00'):
    return struct.pack('!4sI4H',b'FPG1',fid,row,640,480,1)+color*640

class Tests(unittest.TestCase):
    def test_colors(self):
        im=decode565(bytes.fromhex('f80007e0001f'),3,1)
        self.assertEqual([im.getpixel((x,0)) for x in range(3)],[(255,0,0),(0,255,0),(0,0,255)])

    def test_reassembly(self):
        a=Assembler()
        self.assertIsNone(a.feed(b'invalid'))
        for row in range(479,-1,-1):
            result=a.feed(packet(0xffffffff,row))
            if row: self.assertIsNone(result)
            if row==100: self.assertIsNone(a.feed(packet(0xffffffff,row)))
        self.assertEqual(result[0],0xffffffff)
        self.assertEqual(len(result[3]),614400)
        self.assertIsNone(a.feed(packet(0xfffffffe,0)))
        for row in range(480): result=a.feed(packet(0,row,b'\x07\xe0'))
        self.assertEqual(result[0],0)
        self.assertEqual(a.complete,2)
        self.assertTrue(newer(0,0xffffffff))
        # Missing row must not produce a frame, and memory is bounded.
        for fid in range(1,8):
            for row in range(479): self.assertIsNone(a.feed(packet(fid,row)))
        self.assertLessEqual(len(a.frames),4)
        self.assertGreater(a.incomplete,0)

    def test_udp_http_integration(self):
        r=Receiver(0,15)
        http=ThreadingHTTPServer(('127.0.0.1',0),make_handler(r))
        threading.Thread(target=http.serve_forever,daemon=True).start()
        r.start()
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            for row in range(480):
                s.sendto(packet(12,row),('127.0.0.1',r.sock.getsockname()[1]))
                if row%8==0: time.sleep(.001)
            deadline=time.monotonic()+4
            while not r.jpeg and time.monotonic()<deadline: time.sleep(.01)
            self.assertIsNotNone(r.jpeg)
            url='http://127.0.0.1:'+str(http.server_address[1])
            self.assertIn('text/html',urllib.request.urlopen(url).headers['Content-Type'])
            self.assertIn(b'"live": true',urllib.request.urlopen(url+'/api/status').read())
            im=Image.open(io.BytesIO(urllib.request.urlopen(url+'/snapshot.jpg').read()))
            self.assertEqual(im.size,(640,480))
            self.assertGreater(im.getpixel((100,100))[0],245)
            with urllib.request.urlopen(url+'/stream.mjpg',timeout=3) as response:
                self.assertEqual(response.readline(),b'--frame\r\n')
                self.assertEqual(response.readline(),b'Content-Type: image/jpeg\r\n')
                size=int(response.readline().split(b':')[1])
                response.readline()
                self.assertEqual(response.read(size),r.jpeg)
        finally:
            s.close();r.running=False;r.sock.close()
            with r.condition:r.condition.notify_all()
            http.shutdown();http.server_close()

if __name__=='__main__':unittest.main(verbosity=2)
