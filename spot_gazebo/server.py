"""Local HTTP bridge between SDK adapter clients and Gazebo Transport."""
import argparse
import json
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from .core import Controller


def make_server(controller, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, value, status=200):
            data = json.dumps(value, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlparse(self.path)
            try:
                if path.path in ('/state', '/health'):
                    self.reply(controller.state())
                elif path.path == '/feedback':
                    self.reply(controller.feedback(int(parse_qs(path.query)['id'][0])))
                else:
                    self.reply({'error': 'Unknown route'}, 404)
            except (ValueError, KeyError, IndexError) as error:
                self.reply({'error': str(error)}, 400)

        def do_POST(self):
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length <= 8192:
                    raise ValueError('Expected JSON body of at most 8192 bytes')
                body = json.loads(self.rfile.read(length))
                if self.path == '/command':
                    self.reply({'id': controller.command(**body)})
                elif self.path == '/power':
                    if type(body.get('on')) is not bool:
                        raise ValueError('on must be boolean')
                    controller.power(body['on'])
                    self.reply({'ok': True})
                else:
                    self.reply({'error': 'Unknown route'}, 404)
            except (ValueError, TypeError) as error:
                self.reply({'error': str(error)}, 400)
    # The simulation endpoint intentionally has no credentials; bind only to loopback.
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    return server


def main():
    from .gazebo import GazeboBackend
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    backend = GazeboBackend()
    controller = Controller(backend)
    backend.attach(controller)
    server = make_server(controller, args.port)
    stopping = threading.Event()
    def stop(*_):
        stopping.set()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'Spot bridge listening at http://127.0.0.1:{server.server_port}', flush=True)
    try:
        while not stopping.wait(.02):
            controller.tick()
    finally:
        controller.stop()
        server.shutdown()
        server.server_close()
        thread.join()


if __name__ == '__main__':
    main()
