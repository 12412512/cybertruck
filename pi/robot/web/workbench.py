"""工作台网关只转发固定本机接口；不导入串口或创建控制核心。"""

import asyncio
import json
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request as HttpRequest, build_opener, ProxyHandler

from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect

from ..workbench.contracts import Command


class CoreClient:
    def __init__(self, url='http://127.0.0.1:8090', headers=None, timeout=.6):
        parsed = urlsplit(url)
        if parsed.scheme != 'http' or parsed.hostname != '127.0.0.1' or parsed.path not in ('', '/'):
            raise ValueError('控制核心必须是本机回环HTTP服务')
        self.url = url.rstrip('/')
        self.opener = build_opener(ProxyHandler({}))
        self.headers = {'Content-Type': 'application/json', **(headers or {})}
        self.timeout = timeout

    def request(self, path, body=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        request = HttpRequest(self.url + path, data=data,
                              headers=self.headers)
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                return json.load(response)
        except HTTPError as exc:
            try:
                detail = json.load(exc).get('detail', '离线核心拒绝请求')
            except (ValueError, OSError):
                detail = '离线核心拒绝请求'
            raise HTTPException(exc.code, detail) from exc
        except (URLError, OSError, ValueError) as exc:
            raise HTTPException(503, '离线核心未连接；本页不访问真实设备') from exc


def router(client):
    api = APIRouter(prefix='/api/workbench')

    @api.get('/state')
    def state():
        try:
            return client.request('/state')
        except HTTPException:
            return dict(available=False, domain='simulation', real_connected=False,
                        reason='离线核心未连接；当前无有效模拟位姿')

    @api.post('/command')
    def command(value: Command, request: Request):
        # 浏览器必须同源；自定义头使跨站简单表单不能发动作。
        origin = request.headers.get('origin')
        expected = str(request.base_url).rstrip('/')
        if request.headers.get('x-workbench') != '1' or (origin is not None and origin != expected):
            raise HTTPException(403, '仅接受工作台同源控制请求')
        return client.request('/command', value.model_dump())

    @api.websocket('/stream')
    async def stream(socket: WebSocket):
        origin = socket.headers.get('origin')
        expected = f'{"https" if socket.url.scheme == "wss" else "http"}://{socket.headers.get("host")}'
        if origin is not None and origin != expected:
            await socket.close(code=1008)
            return
        await socket.accept()
        try:
            while True:
                value = await asyncio.to_thread(state)
                await asyncio.wait_for(socket.send_json(value), timeout=1)
                await asyncio.sleep(.1)
        except (WebSocketDisconnect, OSError, RuntimeError, TimeoutError):
            pass

    return api


def device_router(client=None):
    """同一工作台的实车通道；只转发固定回环核心，网页不打开设备。"""
    from ..mech_core import DeviceCommand
    client = client or CoreClient('http://127.0.0.1:8091')
    api = APIRouter(prefix='/api/device')

    @api.get('/state')
    def state():
        try:
            return client.request('/state')
        except HTTPException:
            return dict(available=False, connected=False, domain='physical', reason='实车核心未连接；没有打开串口')

    @api.post('/command')
    def command(value: DeviceCommand, request: Request):
        origin = request.headers.get('origin')
        if request.headers.get('x-workbench') != '1' or (origin and origin != str(request.base_url).rstrip('/')):
            raise HTTPException(403, '仅接受同源实车调试请求')
        return client.request('/command', value.model_dump())

    return api


def lidar_router(client=None):
    """固定雷达代理；网页不持有串口，校准提交仍受核心的默认关闭和票据约束。"""
    from ..lidar.contracts import CalibrationPrepare, CalibrationCommit
    client = client or CoreClient('http://127.0.0.1:8093', headers={'X-Lidar-Workbench':'1'})
    api = APIRouter(prefix='/api/lidar')

    @api.get('/state')
    def state():
        try:
            value = client.request('/state')
            if value.get('domain') != 'physical_lidar':
                raise HTTPException(503, '雷达状态来源不符')
            return dict(value, authorizes_motion=False, scan_allowed=False, real_map_registered=False,
                gateway_calibration=True)
        except HTTPException:
            return dict(available=False, connected=False, domain='physical_lidar',
                authorizes_motion=False, scan_allowed=False, real_map_registered=False,
                reason='雷达只读核心未连接；网页没有打开设备')

    @api.get('/calibration/capabilities')
    def calibration_capabilities():
        return dict(gateway=True, authorizes_motion=False)

    def calibration_header(request):
        origin = request.headers.get('origin')
        if request.headers.get('x-workbench') != '1' or (origin and origin != str(request.base_url).rstrip('/')):
            raise HTTPException(403, '只接受同源雷达校准工作台请求')

    @api.post('/calibration/prepare')
    def calibration_prepare(value:CalibrationPrepare, request:Request):
        calibration_header(request)
        return client.request('/calibration/prepare', value.model_dump())

    @api.post('/calibration/commit')
    def calibration_commit(value:CalibrationCommit, request:Request):
        calibration_header(request)
        return client.request('/calibration/commit', value.model_dump())

    @api.post('/calibration/{identity}/cancel')
    def calibration_cancel(identity:str, request:Request):
        calibration_header(request)
        if len(identity) != 32 or any(char not in '0123456789abcdef' for char in identity):
            raise HTTPException(422, '校准票据格式不符')
        return client.request('/calibration/'+identity+'/cancel', {})

    return api
