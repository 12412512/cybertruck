"""工作台网关只转发固定本机接口；不导入串口或创建控制核心。"""

import asyncio
import json
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request as HttpRequest, build_opener, ProxyHandler

from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect

from ..workbench.contracts import Command


class CoreClient:
    def __init__(self, url='http://127.0.0.1:8090'):
        parsed = urlsplit(url)
        if parsed.scheme != 'http' or parsed.hostname != '127.0.0.1' or parsed.path not in ('', '/'):
            raise ValueError('控制核心必须是本机回环HTTP服务')
        self.url = url.rstrip('/')
        self.opener = build_opener(ProxyHandler({}))

    def request(self, path, body=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        request = HttpRequest(self.url + path, data=data,
                              headers={'Content-Type': 'application/json'})
        try:
            with self.opener.open(request, timeout=.6) as response:
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
