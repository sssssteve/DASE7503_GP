"""ESP32 Agent connection status without writing to the serial device."""
import errno
import os
from pathlib import Path
import shutil
import subprocess
import time


class ChassisLink:
    def __init__(self):
        self.process = None
        self.port = None
        self.wait_since = None
        self.problem = ''
        self.start_error = ''
        self.observed_port = None
        self.running = False
        self.last_probe = 0.
        self.log = Path(__file__).resolve().with_name('gui-agent.log')

    def probe(self, now):
        if now - self.last_probe < 1.:
            return
        self.last_probe = now
        bridges=sorted(Path('/dev/serial/by-id').glob('*CP2102*'))
        configured=os.environ.get('ESP_PORT')
        self.port = Path(configured) if configured else bridges[0] if len(bridges)==1 else None
        self.problem = ''
        self.observed_port = None
        pids=subprocess.run(['pgrep','-x','micro_ros_agent'],capture_output=True,text=True).stdout.split()
        self.running=bool(pids)
        for pid in pids:
            try:
                args=Path('/proc',pid,'cmdline').read_bytes().decode().strip('\0').split('\0')
                if '--dev' in args:
                    self.observed_port=Path(args[args.index('--dev')+1])
                    if self.port is None and not configured:
                        self.port=self.observed_port
                    break
            except (OSError, UnicodeError, IndexError):
                continue
        if self.port is None:
            self.problem = '检测到多个 CP2102，请指定 ESP_PORT' if len(bridges)>1 else '未检测到 ESP32 串口'
        elif not self.port.is_char_device():
            self.problem = 'ESP32 串口不存在'
        elif not os.access(self.port,os.R_OK | os.W_OK):
            self.problem = '当前用户没有串口读写权限，请检查 dialout'
        elif self.observed_port is not None and self.port.resolve()!=self.observed_port.resolve():
            self.problem = '已运行的 Agent 占用另一串口'
        if self.running:
            if self.wait_since is None:
                self.wait_since=now
        elif self.process and self.process.poll() is not None:
            self.problem=self.problem or f'Agent 已退出（{self.process.returncode}），请查看 gui-agent.log'

    def start(self, now):
        self.last_probe=0.
        self.start_error=''
        self.probe(now)
        if self.running or (self.process and self.process.poll() is None):
            return
        if self.port is None or not self.port.is_char_device() or not os.access(self.port,os.R_OK | os.W_OK):
            return
        if not shutil.which('ros2'):
            self.start_error='找不到 ROS 2 环境'
            return
        try:
            with self.log.open('ab') as stream:
                self.process=subprocess.Popen(['ros2','run','micro_ros_agent','micro_ros_agent',
                    'serial','--dev',str(self.port),'-b','115200','-v','4'],
                    stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        except OSError as error:
            self.start_error='启动 Agent 失败：'+(error.strerror or errno.errorcode.get(error.errno,'未知错误'))
            return
        self.wait_since=now
        self.problem=''

    def report(self, now, feedback, ready):
        self.probe(now)
        port=str(self.port) if self.port else None
        if feedback and now-feedback<.5:
            return dict(port=port,state='ready' if ready else 'fault',
                        text='ESP32 反馈正常' if ready else 'ESP32 有反馈，底盘报告故障或急停')
        if self.problem or self.start_error:
            return dict(port=port,state='problem',text=self.problem or self.start_error)
        if self.running or (self.process and self.process.poll() is None):
            if self.wait_since is not None and now-self.wait_since>8:
                return dict(port=port,state='no_feedback',text='Agent 已运行，ESP32 无反馈。停车后按板上 EN/RESET')
            return dict(port=port,state='waiting',text='Agent 启动中，等待 ESP32 反馈')
        return dict(port=port,state='idle',text='串口已识别，等待启动 Agent')
