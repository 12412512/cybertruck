"""FastAPI只读总览。控制核心和网页为不同进程，通过快照交换状态。"""

import argparse
import asyncio
import json
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from ..runtime import load_snapshot

DEFAULT_RUNTIME = Path(__file__).resolve().parents[3] / 'output/runtime'
STATIC = Path(__file__).with_name('static')


def create_app(runtime=DEFAULT_RUNTIME):
    app = FastAPI(title='智能搬运小车只读工作台', docs_url=None, redoc_url=None)
    snapshot = Path(runtime) / 'state.json'
    app.mount('/static', StaticFiles(directory=STATIC), name='static')

    @app.get('/')
    def index():
        return FileResponse(STATIC / 'index.html')

    @app.get('/api/state')
    def state():
        return load_snapshot(snapshot)

    @app.get('/api/base/trials')
    def base_trials():
        from ..device.execution import trials
        return trials()

    @app.get('/api/routes/short')
    def short_route():
        from ..nav.route import validate
        route = json.loads((DEFAULT_RUNTIME.parent.parent / 'config/short_route.json').read_text(encoding='utf-8'))
        validate(route)
        return route

    @app.get('/api/routes/short/replay')
    def route_replay():
        # 只提供显式标记的模型记录；缺文件或版本不符不拿真实日志冒充模拟。
        try:
            report = json.loads((DEFAULT_RUNTIME.parent / 'route-link/closed-loop.json').read_text(encoding='utf-8'))
            view = report['snapshot']['route_execution']
            if (report['snapshot']['mode'] != 'sim' or view['domain'] != 'simulation'
                    or view['plan_sha256'] != short_route()['sha256']):
                raise ValueError('模型记录与当前路线不匹配')
            frames = [v for v in report['records'] if v['kind'] == 'state' and v.get('execution')]
            return dict(available=True, domain='simulation_replay', plan_sha256=view['plan_sha256'],
                        frames=frames, result=report['result'])
        except (OSError, ValueError, KeyError, TypeError):
            return dict(available=False, domain='simulation_replay')

    @app.get('/api/routes/field')
    def field_route():
        state = load_snapshot(snapshot)
        return (state.get('navigation') or {}).get('execution_package') or dict(
            available=False, domain='simulation', executable=False)

    @app.websocket('/ws')
    async def stream(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                await asyncio.wait_for(socket.send_json(load_snapshot(snapshot)), timeout=1)
                await asyncio.sleep(.1)
        except (WebSocketDisconnect, OSError, RuntimeError, TimeoutError):
            pass

    return app


def main():
    import uvicorn
    parser = argparse.ArgumentParser(description='只读Web，不占用串口')
    parser.add_argument('--runtime', type=Path, default=DEFAULT_RUNTIME)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    uvicorn.run(create_app(args.runtime), host=args.host, port=args.port, log_level='warning')


if __name__ == '__main__':
    main()
