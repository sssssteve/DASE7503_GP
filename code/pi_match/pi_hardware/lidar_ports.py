"""Follow one MS200p across USB re-enumerations on the Pi."""
import fcntl
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time


def find_lidar(dev=Path('/dev'), tty_class=Path('/sys/class/tty')):
    configured=os.environ.get('LIDAR_PORT')
    acm=[dev/'ttyACM0',dev/'ttyACM1']
    if configured and Path(configured) not in acm:
        paths=[Path(configured)]
    else:
        paths=sorted((dev/'serial/by-id').glob('usb-1a86_USB_Single_Serial_*-if00'))
        for port in acm:
            usb=tty_class/port.name/'device'/'..'
            try:
                if ((usb/'idVendor').read_text().strip(),(usb/'idProduct').read_text().strip())==('1a86','55d4'):
                    paths.append(port)
            except FileNotFoundError:
                continue
    for path in paths:
        try:
            real=path.resolve(strict=True)
            info=real.stat()
        except FileNotFoundError:
            continue
        if stat.S_ISCHR(info.st_mode):
            return real,info.st_dev,info.st_ino,info.st_rdev
    return None


def stop_driver(child):
    child.terminate()
    try:
        child.wait(timeout=3)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()


def monitor(binary):
    stopping=False
    def stop(signum,frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    current=child=None
    try:
        while not stopping:
            device=find_lidar()
            if child is not None and (device!=current or child.poll() is not None):
                stop_driver(child)
                child=None
                print('旧雷达驱动已退出，重新识别串口',flush=True)
            if device is not None and child is None:
                current=device
                print(f'连接 MS200p {device[0]} → /scan',flush=True)
                child=subprocess.Popen([binary,'--ros-args','-p',f'port_name:={device[0]}',
                    '-p','baudrate:=230400','-p','device_model:=MS200','-p','motor_speed:=10',
                    '-p','clockwise:=false','-p','frame_id:=laser_frame','-p','scan_topic:=/scan',
                    '-p','angle_min:=0.0','-p','angle_max:=360.0','-p','range_min:=0.03','-p','range_max:=12.0'])
            time.sleep(.5)
    finally:
        if child is not None:
            stop_driver(child)


if __name__=='__main__':
    lock_path=Path(os.environ.get('XDG_RUNTIME_DIR') or '/tmp')/f'dase7503-lidar-{os.getuid()}.lock'
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            sys.exit('单雷达监看已在运行')
        monitor(sys.argv[1])
