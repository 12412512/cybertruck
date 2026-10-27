"""浏览器同源连接入口；密码仅转交本机连接管理器，不写文件。"""

from fastapi import APIRouter, HTTPException, Request
from ..connection_core import ConnectCommand
from .workbench import CoreClient


def router(client=None):
    client = client or CoreClient('http://127.0.0.1:8094', headers={'X-Workbench':'1'}, timeout=2)
    api = APIRouter(prefix='/api/connections')

    @api.get('/state')
    def state():
        try:
            return client.request('/state')
        except HTTPException:
            return dict(service='gc_connections', available=False, busy=False,
                        message='连接管理器未启动；请双击工程内“打开工作台.cmd”', motion_authority=False)

    @api.post('/connect')
    def connect(value: ConnectCommand, request: Request):
        if (request.headers.get('x-workbench') != '1' or
                (request.headers.get('origin') and request.headers['origin'] != str(request.base_url).rstrip('/'))):
            raise HTTPException(403, '只接受工作台同源连接请求')
        try:
            return client.request('/connect', dict(component=value.component,
                password=value.password.get_secret_value(), confirmed_reopen=value.confirmed_reopen))
        except HTTPException as error:
            if error.status_code == 503:
                raise HTTPException(503, '连接管理器未启动或未响应；请双击“打开工作台.cmd”，不要重复发送设备动作') from error
            raise

    return api
