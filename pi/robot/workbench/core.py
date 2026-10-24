"""独立离线控制服务，监听本机回环；网页进程不拥有执行器。"""

import argparse
from contextlib import asynccontextmanager
from pathlib import Path
import threading
import time

from fastapi import FastAPI, HTTPException
from pydantic import ValidationError

from ..runtime import ProcessLock
from .contracts import Command
from .service import Workbench, Rejected


def create_core(library, runtime, *, background=True):
    core = Workbench(library, runtime)
    stop = threading.Event()

    def loop():
        previous = time.monotonic()
        deadline = previous + .01
        while not stop.wait(max(0, deadline - time.monotonic())):
            now = time.monotonic()
            try:
                core.tick(now - previous)
            except Exception as exc:
                with core.lock:
                    core.sim.halt(f'离线核心异常：{exc}')
                    core.record('控制异常', str(exc))
                stop.set()
            previous = now
            deadline = max(deadline + .01, now)

    @asynccontextmanager
    async def lifespan(app):
        lock = ProcessLock(Path(runtime) / 'workbench.lock')
        thread = threading.Thread(target=loop, daemon=True, name='offline-control')
        if background:
            thread.start()
        try:
            yield
        finally:
            stop.set()
            if background:
                thread.join(timeout=1)
            core.close()
            lock.close()

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)
    app.state.core = core

    @app.get('/state')
    def state():
        value = core.snapshot()
        if stop.is_set():
            value.update(available=False, reason='控制循环已退出，需人工重启离线核心')
        return value

    @app.post('/command')
    def command(value: Command):
        if stop.is_set():
            raise HTTPException(503, '离线控制循环未运行')
        try:
            return core.command(value.model_dump())
        except (Rejected, ValueError, ValidationError) as exc:
            raise HTTPException(409, str(exc)) from exc

    return app


def main():
    import uvicorn
    parser = argparse.ArgumentParser(description='只运行离线模型，不提供串口连接选项')
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, default=Path('output/workbench'))
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args()
    uvicorn.run(create_core(args.library, args.runtime), host='127.0.0.1', port=args.port,
                log_level='warning')


if __name__ == '__main__':
    main()
