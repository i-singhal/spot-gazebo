"""Dependency-free client for diagnostics and the normalized bridge protocol."""
import json
from urllib.error import HTTPError
from urllib.request import Request, urlopen


class BridgeClient:
    def __init__(self, url='http://127.0.0.1:8080'):
        self.url = url.rstrip('/')

    def request(self, path, body=None, timeout=5.):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        req = Request(self.url+path, data=data, headers={'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=timeout) as response:
                return json.load(response)
        except HTTPError as error:
            raise ValueError(json.load(error).get('error', str(error))) from error

    def state(self):
        return self.request('/state')

    def power(self, on):
        return self.request('/power', {'on': on})

    def command(self, kind, **kwargs):
        return self.request('/command', dict(kind=kind, **kwargs))['id']

    def feedback(self, command_id):
        return self.request(f'/feedback?id={command_id}')
