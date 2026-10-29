"""PC连接管理器：显式SSH准备与固定回环转发；不导入串口，不重发设备动作。"""

import argparse
import json
from pathlib import Path
import select
import shlex
import socketserver
import threading
from urllib.request import build_opener, ProxyHandler

from fastapi import FastAPI, HTTPException, Request
from pydantic import BaseModel, ConfigDict, SecretStr

ROOT = Path(__file__).resolve().parents[2]


class ConnectCommand(BaseModel):
    model_config = ConfigDict(extra='forbid')
    component: str = 'all'
    password: SecretStr = SecretStr('')
    confirmed_reopen: bool = False


class ForwardServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class ForwardHandler(socketserver.BaseRequestHandler):
    def handle(self):
        from paramiko import SSHException
        channel = None
        try:
            channel = self.server.transport.open_channel('direct-tcpip',
                ('127.0.0.1', self.server.remote_port), self.request.getpeername(), timeout=3)
            while self.server.transport.is_active():
                ready, _, _ = select.select([self.request, channel], [], [], 1)
                for source, target in ((self.request, channel), (channel, self.request)):
                    if source in ready:
                        data = source.recv(65536)
                        if not data:
                            return
                        target.sendall(data)
        except (OSError, EOFError, SSHException):
            pass
        finally:
            if channel:
                channel.close()


class SSHBackend:
    def __init__(self, profile):
        self.profile = profile
        self.client = None
        self.servers = {}
        self.password = ''  # 仅本进程内存；不写配置、日志或响应。

    @property
    def active(self):
        return bool(self.client and self.client.get_transport() and self.client.get_transport().is_active())

    def connect(self, password):
        import paramiko
        if password:
            self.password = password
        if self.active:
            return
        self.close(forget=False)
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.load_host_keys(str(Path.home() / '.ssh/known_hosts'))
        try:
            client.connect(self.profile['host'], username=self.profile['username'],
                           password=self.password or None, look_for_keys=not bool(self.password),
                           allow_agent=not bool(self.password), timeout=6, banner_timeout=8, auth_timeout=6)
        except Exception:
            client.close()
            raise
        self.client = client
        client.get_transport().set_keepalive(10)

    def forward(self, component):
        port = 8091 if component == 'device' else 8092
        if port in self.servers:
            return
        server = ForwardServer(('127.0.0.1', port), ForwardHandler)
        server.transport = self.client.get_transport()
        server.remote_port = port
        self.servers[port] = server
        threading.Thread(target=server.serve_forever, daemon=True, name=f'ssh-{port}').start()

    def prepare(self, component, confirmed):
        command = [self.profile['python'], '-B', self.profile['helper_root'] + '/connection_host.py',
                   '--component', component]
        if confirmed:
            command.append('--confirmed-reopen')
        _, stdout, _ = self.client.exec_command(' '.join(shlex.quote(v) for v in command), timeout=25)
        raw = stdout.read(2 * 1024 * 1024)
        try:
            result = json.loads(raw)
        except (ValueError, TypeError):
            raise ValueError('树莓派连接准备器未返回有效结果；请检查部署，未自动重试') from None
        if not result.get('ok'):
            raise ValueError(result.get('reason', '连接准备未通过'))
        return {key: result[key] for key in ('action', 'component', 'pid', 'backup') if key in result}

    def health(self, component):
        port = 8091 if component == 'device' else 8092
        if not self.active or port not in self.servers:
            return None
        try:
            with build_opener(ProxyHandler({})).open(f'http://127.0.0.1:{port}/state', timeout=.6) as reply:
                return json.load(reply)
        except (OSError, ValueError):
            return None

    def close(self, forget=True):
        for server in self.servers.values():
            server.shutdown()
            server.server_close()
        self.servers.clear()
        if self.client:
            self.client.close()
        self.client = None
        if forget:
            self.password = ''


def device_summary(state):
    state = state or {}
    m = state.get('mechanism') or {}
    base = state.get('base') or {}
    return dict(connected=state.get('connected') is True, firmware=state.get('firmware'),
        state_age_ms=state.get('state_age_ms'), base_fault=base.get('fault'),
        mechanism_fault=m.get('fault'), stopped=base.get('stopped'), attached=m.get('attached'),
        zero_valid=m.get('zero_valid'), pwm_mask=m.get('pwm_mask'),
        boot_id=state.get('boot_id'), session_id=state.get('session_id'))


class Connections:
    def __init__(self, profile, backend=None):
        self.profile = profile
        self.backend = backend or SSHBackend(profile)
        self.lock = threading.Lock()
        self.busy = False
        self.phase = 'IDLE'
        self.message = '尚未连接；点击按钮后才访问树莓派'
        self.results = {}

    def state(self):
        device = device_summary(self.backend.health('device'))
        vision = self.backend.health('vision') or {}
        return dict(service='gc_connections', available=True, busy=self.busy, phase=self.phase,
            message=self.message, host=self.profile['host'], username=self.profile['username'],
            ssh_connected=self.backend.active, credential_cached=bool(self.backend.password),
            device=device, cameras=vision.get('cameras', {}), vision_connected=vision.get('available', False),
            results=self.results, auto_reconnect=False, motion_authority=False)

    def start(self, value):
        if value.component not in ('all', 'device', 'vision'):
            raise ValueError('只能连接底盘/机械、视觉或全部设备')
        with self.lock:
            if self.busy:
                raise ValueError('连接准备正在进行，禁止重复点击')
            self.busy = True
            self.phase = 'CONNECTING'
            self.message = '正在核对SSH、设备身份和已有服务'
        threading.Thread(target=self.run, args=(value,), daemon=True, name='connection-prepare').start()
        return dict(status='CONNECTING', motion_authority=False)

    def run(self, value):
        try:
            self.backend.connect(value.password.get_secret_value())
            components = ('device', 'vision') if value.component == 'all' else (value.component,)
            failures = []
            for component in components:
                try:
                    self.backend.forward(component)
                    result = self.backend.prepare(component, value.confirmed_reopen)
                    self.results[component] = dict(ok=True, **result)
                except Exception as error:
                    reason = str(error).replace(self.backend.password, '[redacted]') if self.backend.password else str(error)
                    self.results[component] = dict(ok=False, reason=reason[:300])
                    failures.append(component)
            self.phase = 'PARTIAL' if failures else 'READY'
            self.message = '部分组件未就绪，请查看对应原因' if failures else '连接准备完成；不会取得控制权、使能、建零或启动动作'
        except Exception as error:
            # 不把认证异常原文或请求对象写到日志/响应。
            self.phase = 'FAILED'
            self.message = f'SSH连接失败（{type(error).__name__}）；核对网络、密码及已记录的主机身份'
        finally:
            self.busy = False


def create_app(manager):
    app = FastAPI(docs_url=None, redoc_url=None)

    @app.get('/state')
    def state():
        return manager.state()

    @app.post('/connect')
    def connect(value: ConnectCommand, request: Request):
        if request.headers.get('x-workbench') != '1' or request.headers.get('origin'):
            raise HTTPException(403, '只接受本机工作台网关请求')
        try:
            return manager.start(value)
        except ValueError as error:
            raise HTTPException(409, str(error)) from error
    return app


def main():
    import uvicorn
    from .runtime import ProcessLock
    profile = json.loads((ROOT / 'config/connections.json').read_text(encoding='utf8'))
    lock = ProcessLock(ROOT / 'output/connections/core.lock')
    manager = Connections(profile)
    try:
        uvicorn.run(create_app(manager), host='127.0.0.1', port=8094, log_level='warning', access_log=False)
    finally:
        manager.backend.close()
        lock.close()


if __name__ == '__main__':
    main()
