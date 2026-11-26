"""Run the browser GUI on the Mac, forwarding live data via SSH."""
import argparse
import base64
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shlex
import socket
import subprocess
from threading import Condition, Event, Lock, Thread
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, build_opener, ProxyHandler
import webbrowser

from wall_preview import WallPreview
from field_observations import FieldObservations
from course_clearance import world

HERE = Path(__file__).resolve().parent


def offline_state():
    map_path = HERE.parent/'official_reference/field.pgm'
    if not map_path.exists():
        map_path = HERE.parent/'pi_ws/maps/official_reference/field.pgm'
    magic, size, depth, image = map_path.read_bytes().split(b'\n', 3)
    if magic != b'P5' or size != b'155 104' or depth != b'255' or len(image) != 155*104:
        raise ValueError('官方底图格式不匹配')
    rows = [image[i:i+155] for i in range(0, len(image), 155)]
    pixels = b''.join(reversed(rows)).translate(bytes(0 if i == 254 else 100 if i == 0 else 255 for i in range(256)))
    return {'map': {'width':155,'height':104,'resolution':.02,'origin':[-.04,-.04,0.],
                    'pixels':base64.b64encode(pixels).decode()},
            'points':[], 'pose':None, 'status':'离线 · 官方底图'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='pi@pi')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--pi-root', default='~/DASE7503_GP/code/pi_match',
                        help='Pi 上包含 pi_hardware/ 的目录，仅用于 --radar')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--offline',action='store_true')
    modes.add_argument('--radar',action='store_true',help='SSH 只读订阅 /scan，本地绘图')
    args = parser.parse_args()
    local = f'http://127.0.0.1:{args.port}'
    with socket.socket() as port:
        port.bind(('127.0.0.1',0))
        forward = port.getsockname()[1]
    remote = f'http://127.0.0.1:{forward}'
    opener = build_opener(ProxyHandler({}))
    offline = offline_state()
    page = (HERE/'mapping_web.html').read_bytes()
    tunnel, retry_at = None, 0.
    lock = Lock()

    camera_changed=Condition(lock)
    camera=None
    camera_time=0.
    camera_arrivals=deque(maxlen=26)
    stop_stream=Event()
    last_scan={}
    received=0.
    last_fix=None
    streams={}
    wall_preview=WallPreview()
    observations=FieldObservations()
    def read_stream(filename):
        nonlocal received,last_scan,camera,camera_time,last_fix
        code=(HERE/filename).read_bytes()
        remote_script = (
            'source /opt/ros/jazzy/setup.bash && '
            'source "${DASE_CHASSIS_SETUP:-$HOME/microros_ws/install/setup.bash}" && '
            'export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && '
            f'export DASE_PI_MATCH_ROOT={shlex.quote(args.pi_root)} && exec python3 -u -'
        )
        command = 'bash -c ' + shlex.quote(remote_script)
        while not stop_stream.is_set():
            process=subprocess.Popen(['ssh','-CT','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
                '-o','ConnectTimeout=4','-o','ServerAliveInterval=3','-o','ServerAliveCountMax=2',
                '--',args.host,command],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
            with lock:
                streams[filename]=process
                if stop_stream.is_set():process.terminate()
            try:
                process.stdin.write(code)
                process.stdin.close()
                for line in process.stdout:
                    message=json.loads(line)
                    if filename=='scan_stream.py':
                        if message.get('pose') is not None and message.get('grid_hash')==map_hash:
                            wall_preview.prior=message['pose']
                            wall_preview.hits=3
                            wall_preview.last_stamp=wall_preview.last_good_stamp=message.get('scan_stamp')
                            wall_preview.last_result=message['pose']
                        elif message.get('grid_hash') in (None,map_hash):
                            message['wall_preview']=wall_preview.estimate(message)
                        pose=message.get('wall_preview') or (message.get('pose') if message.get('grid_hash')==map_hash else None)
                        if pose:
                            points=[world(p,pose) for p in message['filtered']]
                            message['observations']=observations.update(points,message['scan_stamp'],
                                'preview' if message.get('wall_preview') else 'tf',message.get('rack_layout',[]))
                    with camera_changed:
                        if filename=='camera_stream.py':
                            camera=(message['stamp'],base64.b64decode(message['jpeg']))
                            camera_time=time.monotonic()
                            camera_arrivals.append(camera_time)
                            camera_changed.notify_all()
                        else:
                            last_scan=message
                            received=time.monotonic()
                            if pose and (last_fix is None or last_fix[0]!=message['scan_stamp']):
                                last_fix=(message['scan_stamp'],pose,received)
                            elif message.get('grid_hash') not in (None,map_hash):
                                last_fix=None
            except (BrokenPipeError,ValueError):
                pass
            finally:
                process.terminate()
                process.wait()
            stop_stream.wait(2)

    def preview_state():
        live=camera is not None and time.monotonic()-camera_time<1.5
        rate=None
        if live and len(camera_arrivals)>=3:
            rate=round((len(camera_arrivals)-1)/max(.001,camera_arrivals[-1]-camera_arrivals[0]),1)
        history=None if last_fix is None else dict(pose=last_fix[1],age_s=round(time.monotonic()-last_fix[2],1))
        return dict(camera_live=live,camera_hz=rate,last_map_fix=history)

    map_hash=hashlib.sha256(base64.b64decode(offline['map']['pixels'])).hexdigest()

    def connect():
        nonlocal tunnel, retry_at
        if args.offline:
            return False
        with lock:
            if tunnel is not None and tunnel.poll() is None:
                return True
            if time.monotonic() < retry_at:
                return False
            retry_at = time.monotonic()+5
            tunnel = subprocess.Popen(['ssh','-NT','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
                '-o','ConnectTimeout=5','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=5',
                '-L',f'127.0.0.1:{forward}:127.0.0.1:8765','--',args.host])
            return True

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, data=b'', mime='application/json'):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/':
                self.reply(200, page, 'text/html; charset=utf-8')
            elif self.path == '/camera.mjpg' and not args.offline:
                self.send_response(200)
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control','no-store')
                self.end_headers()
                stamp=None
                self.connection.settimeout(3)
                try:
                    while not stop_stream.is_set():
                        with camera_changed:
                            if not camera_changed.wait_for(lambda:camera and camera[0]!=stamp and time.monotonic()-camera_time<1.5,timeout=4):
                                break
                            stamp,jpeg=camera
                        self.wfile.write(f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\nX-Frame-Stamp: {stamp}\r\n\r\n'.encode()+jpeg+b'\r\n')
                        self.wfile.flush()
                except (OSError,TimeoutError):
                    pass
            elif self.path == '/state':
                if args.radar:
                    with lock:
                        state=dict(offline,monitor=True,status='等待实时雷达连接',raw=[])
                        if time.monotonic()-received<1.5:
                            state.update(last_scan)
                        elif received:
                            state['status']=f'雷达监看链路 {time.monotonic()-received:.1f} 秒未更新（SSH）'
                        state.update(preview_state())
                    if state.get('pose') and state.get('grid_hash')!=map_hash:
                        state.update(pose=None,points=[],status='雷达在线 · 等待对应底图')
                    if state.get('route_grid_hash')!=map_hash:
                        state.pop('route',None)
                        state.pop('pose_trail',None)
                    state['scan_pose']=state.get('pose') or state.get('wall_preview')
                    if state.get('wall_preview'):
                        state['status']='实时雷达 · 围墙估计预览'
                    self.reply(200,json.dumps(state).encode())
                    return
                try:
                    if not connect():
                        raise URLError('offline')
                    with opener.open(remote+'/state', timeout=.8) as response:
                        state = json.load(response)
                    if (state.get('map') or {}).get('pixels') != offline['map']['pixels']:
                        state['map'] = offline['map']
                        state.update(points=[], pose=None, racks=[], status='等待树莓派官方图定位')
                except (URLError, TimeoutError, ValueError, OSError):
                    state = dict(offline, status='离线预览 · 官方底图' if args.offline else
                                 '树莓派网页未连接 · 官方底图')
                if not args.offline:
                    with lock:
                        sample=last_scan if time.monotonic()-received<1.5 else {}
                        state.update(**preview_state(),
                                     route=sample.get('route') if sample.get('route_grid_hash')==map_hash else None,
                                     pose_trail=sample.get('pose_trail',[]) if sample.get('route_grid_hash')==map_hash else [],
                                     sources=sample.get('sources',{}),base_status=sample.get('base_status',''),
                                     localization=sample.get('localization',''),monitor=True,raw=sample.get('raw',[]),
                                     filtered=sample.get('filtered',[]),body_hits=sample.get('body_hits'),
                                     mask_box=sample.get('mask_box'),body_size=sample.get('body_size'),
                                     body_padding=sample.get('body_padding',0.),
                                     mask_outlines=sample.get('mask_outlines'),rack_layout=sample.get('rack_layout'),
                                     observations=sample.get('observations'),
                                     cluster_hits=sample.get('cluster_hits'),isolated_hits=sample.get('isolated_hits'),
                                     wall_preview=sample.get('wall_preview'),
                                     scan_pose=sample.get('pose') if sample.get('grid_hash')==map_hash and sample.get('pose') else sample.get('wall_preview'))
                self.reply(200, json.dumps(state).encode())
            else:
                self.reply(404)

        def do_POST(self):
            if args.radar or args.offline:
                self.reply(403)
                return
            if (self.path != '/command' or self.headers.get('Origin') != local
                    or self.headers.get('X-DASE-GUI') != '1'):
                self.reply(403)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 256:
                    self.reply(400)
                    return
                request = Request(remote+'/command', self.rfile.read(size),
                                  headers={'X-DASE-GUI':'1','Content-Type':'application/json'})
                with opener.open(request, timeout=.5) as response:
                    self.reply(response.status)
            except HTTPError as error:
                self.reply(error.code)
            except (URLError, TimeoutError, ValueError):
                self.reply(503)

        def log_message(self, *_):
            pass

    with ThreadingHTTPServer(('127.0.0.1',args.port), Handler) as server:
        try:
            if not args.offline:
                for filename in ('scan_stream.py','camera_stream.py'):
                    Thread(target=read_stream,args=(filename,),daemon=True).start()
            print(f'本地界面 {local}', flush=True)
            webbrowser.open(local)
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            stop_stream.set()
            with lock:
                for stream in streams.values():
                    if stream.poll() is None:stream.terminate()
            if tunnel is not None and tunnel.poll() is None:
                tunnel.terminate()
                tunnel.wait()


if __name__=='__main__':
    main()
