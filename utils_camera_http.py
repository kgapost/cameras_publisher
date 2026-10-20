"""HTTP/OpenAPI transport; camera work is dispatched to the ROS executor."""

import asyncio
from concurrent.futures import Future
import math
from queue import Empty, Full, Queue
import socket
import threading
import time


class APIUnavailable(RuntimeError):
    pass


class CommandQueue:
    """Bounded mailbox. Only drain() executes commands or touches camera state."""

    def __init__(self, execute, capacity=64):
        self.execute = execute
        self.pending = Queue(maxsize=capacity)
        self.lock = threading.Lock()
        self.closed = False
        self.wake = lambda: None

    def submit(self, action, **parameters):
        future = Future()
        with self.lock:
            if self.closed:
                raise APIUnavailable('Camera API is shutting down')
            try:
                self.pending.put_nowait((action, parameters, future))
            except Full:
                raise APIUnavailable('Camera command queue is full') from None
            self.wake()
        return future

    def drain(self):
        # Bound each batch so a burst of HTTP requests cannot starve recording.
        for _ in range(8):
            with self.lock:
                if self.closed:
                    return
                try:
                    action, parameters, future = self.pending.get_nowait()
                except Empty:
                    return
                running = future.set_running_or_notify_cancel()
            if not running:
                continue
            try:
                future.set_result(self.execute(action, **parameters))
            except Exception as exc:
                future.set_exception(exc)
        with self.lock:
            if not self.closed and not self.pending.empty():
                self.wake()

    def close(self):
        with self.lock:
            self.closed = True
            while True:
                try:
                    _, _, future = self.pending.get_nowait()
                except Empty:
                    break
                if future.set_running_or_notify_cancel():
                    future.set_exception(APIUnavailable('Camera API is shutting down'))


def create_app(commands, timeout=10.0):
    from fastapi import FastAPI, Response
    from pydantic import BaseModel, Field

    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('HTTP_API_TIMEOUT must be finite and positive')

    class CameraHealth(BaseModel):
        camera: str
        available: bool
        healthy: bool
        message: str
        recording: bool
        recording_error: str | None = None
        recordings: dict[str, str]
        recording_errors: dict[str, str]

    class HealthReport(BaseModel):
        healthy: bool
        timestamp: str
        cameras: dict[str, CameraHealth]

    class CaptureName(BaseModel):
        model_config = {'extra': 'forbid'}
        name: str = Field(min_length=1, max_length=64,
                          pattern=r'^[A-Za-z0-9][A-Za-z0-9_.-]*$',
                          description='Recording ID / snapshot filename prefix. Use the same name '
                                      'to stop a recording; names are scoped to each camera.')

    class CaptureResponse(BaseModel):
        success: bool
        message: str
        health: HealthReport | None = None

    app = FastAPI(title='Camera Capture API', version='2.0.0',
                  description='Shared ROS 2/HTTP camera snapshots, recordings and health. '
                              'The endpoint selects camera 1 (belly or left) or camera 2 (right). '
                              'Requests contain only name; pass default for a default capture. '
                              'Multiple names can record the same camera independently. '
                              'Health accepts the same request and ignores the name.')

    async def run(action, response, parameters):
        future = None
        try:
            future = commands.submit(action, **parameters)
            result, status = await asyncio.wait_for(
                asyncio.shield(asyncio.wrap_future(future)), timeout=timeout)
        except APIUnavailable as exc:
            result, status = dict(success=False, message=str(exc)), 503
        except TimeoutError:
            cancelled = future.cancel()
            message = ('Command timed out before execution; it was cancelled' if cancelled else
                       'Command timed out during execution; outcome unknown. Check health before retrying')
            result, status = dict(success=False, message=message), 504
        except asyncio.CancelledError:
            if future is not None:
                future.cancel()
            raise
        except Exception:
            result, status = dict(success=False, message='Camera command failed'), 500
        response.status_code = status
        return result

    def endpoint(action):
        async def named(response: Response, capture: CaptureName):
            return await run(action, response, capture.model_dump())
        return named

    descriptions = {
        'health': 'Publish and return camera health; the supplied name is ignored.',
        'image1': 'Save a JPEG from camera 1 (belly or left).',
        'image2': 'Save a JPEG from camera 2 (right); warn when no second camera exists.',
        'video1_start': 'Start camera 1 MP4 with the supplied recording name.',
        'video1_stop': 'Finalize only camera 1 MP4 with the supplied recording name.',
        'video2_start': 'Start camera 2 MP4 with the supplied recording name.',
        'video2_stop': 'Finalize only camera 2 MP4 with the supplied recording name.',
    }
    errors = {code: dict(model=CaptureResponse, description=description) for code, description in (
        (404, 'Camera unavailable'), (409, 'Recording state conflict or no fresh frame'),
        (500, 'Capture or storage failure'), (503, 'Queue full or API stopping'),
        (504, 'ROS command execution timed out'))}
    for name, description in descriptions.items():
        app.add_api_route(f'/api/{name}', endpoint(name), methods=['POST'],
                          operation_id=name, summary=description, response_model=CaptureResponse,
                          response_model_exclude_none=True, responses=errors)
    return app


class CameraHTTPServer:
    """One in-process server; ROS retains signal handling and owns shutdown."""

    def __init__(self, commands, host, port, timeout):
        import uvicorn

        if not 1 <= port <= 65535:
            raise ValueError('HTTP_API_PORT must be between 1 and 65535')
        self.host, self.port = host, port
        self.socket = None
        self.thread = None
        self.server = uvicorn.Server(uvicorn.Config(
            create_app(commands, timeout), host=host, port=port,
            loop='asyncio', http='h11', ws='none', workers=1, access_log=False,
            timeout_graceful_shutdown=2))

    def start(self):
        # Bind synchronously so a port conflict fails startup visibly.
        self.socket = socket.create_server((self.host, self.port),
                                           family=socket.AF_INET6 if ':' in self.host else socket.AF_INET)
        self.thread = threading.Thread(target=self.server.run,
                                       kwargs={'sockets': [self.socket]},
                                       name='camera-http', daemon=True)
        self.thread.start()
        deadline = time.monotonic() + 5.0
        while not self.server.started:
            if not self.thread.is_alive() or time.monotonic() >= deadline:
                self.close()
                raise RuntimeError('HTTP capture server failed to start')
            time.sleep(0.01)

    def close(self):
        self.server.should_exit = True
        if self.thread is not None:
            self.thread.join(timeout=4.0)
        if self.socket is not None:
            self.socket.close()
