"""Shared test helpers: a fake transport and a fake auth/relay server.

No test in this suite uses a real credential, a real key, or the network.
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from orcarouter.transport import HttpResult

FAKE_KEY = "sk-orca-test-0000000000000000000000000000"
FAKE_KEY_2 = "sk-orca-test-1111111111111111111111111111"


@dataclass
class RecordedRequest:
    method: str
    url: str
    json_body: dict | None
    headers: dict


@dataclass
class FakeTransport:
    """Transport double that records every request (so tests can assert
    origins and bodies) and answers from a scripted table."""

    responses: dict = field(default_factory=dict)
    calls: list = field(default_factory=list)
    default: HttpResult | None = None

    def request(
        self,
        method: str,
        url: str,
        *,
        json_body=None,
        form_body=None,
        headers=None,
        timeout=30.0,
    ) -> HttpResult:
        self.calls.append(
            RecordedRequest(method, url, json_body, dict(headers or {}))
        )
        key = (method, urlsplit(url).path)
        if key in self.responses:
            value = self.responses[key]
            if callable(value):
                return value(len([c for c in self.calls if (c.method, urlsplit(c.url).path) == key]))
            return value
        if self.default is not None:
            return self.default
        raise AssertionError(f"unexpected request {method} {url}")

    # -- assertions helpers -------------------------------------------------

    @property
    def urls(self) -> list[str]:
        return [c.url for c in self.calls]


def json_response(payload: object, status: int = 200) -> HttpResult:
    return HttpResult(status, json.dumps(payload).encode(), {})


def models_payload(records: list) -> dict:
    return {"object": "list", "data": records}


def chat_model(model_id: str, modalities=("text",), endpoints=("openai",)) -> dict:
    return {
        "id": model_id,
        "object": "model",
        "name": model_id,
        "owned_by": "test",
        "supported_endpoint_types": list(endpoints),
        "architecture": {
            "input_modalities": list(modalities),
            "output_modalities": ["text"],
        },
    }


# ---------------------------------------------------------------------------
# A real local fake auth server, used for the end-to-end connect test.
# ---------------------------------------------------------------------------


class FakeOrcaServer:
    """Minimal stand-in for the OrcaRouter auth + relay origins.

    Used to exercise the *real* PKCE adapter end to end: authorize URL ->
    loopback callback (or OOB code) -> exchange -> persist.
    """

    def __init__(self) -> None:
        self.exchange_requests: list[dict] = []
        self.models_requests: list[dict] = []
        self.chat_requests: list[dict] = []
        self.expected_challenge: str | None = None
        self.expected_state: str | None = None
        self.exchange_status = 200
        self.exchange_body: dict | None = None
        self.scope = "api"
        self._server: ThreadingHTTPServer | None = None
        self.port = 0
        self._thread: threading.Thread | None = None

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> str:
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *_args):
                return

            def _read(self) -> dict:
                length = int(self.headers.get("Content-Length") or 0)
                if not length:
                    return {}
                try:
                    return json.loads(self.rfile.read(length).decode())
                except Exception:
                    return {}

            def _send(self, status: int, payload: dict) -> None:
                body = json.dumps(payload).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            # auth origin
            def do_POST(self):  # noqa: N802
                path = urlsplit(self.path).path
                body = self._read()
                if path == "/api/v1/auth/keys":
                    outer.exchange_requests.append(body)
                    if outer.exchange_status != 200:
                        return self._send(outer.exchange_status,
                                          outer.exchange_body or {"error": "invalid_grant"})
                    return self._send(200, outer.exchange_body or {
                        "key": FAKE_KEY, "user_id": "42", "scope": outer.scope,
                    })
                return self._send(404, {"error": "not_found"})

            def do_GET(self):  # noqa: N802
                parsed = urlsplit(self.path)
                query = parse_qs(parsed.query)
                if parsed.path == "/v1/models":
                    outer.models_requests.append(dict(self.headers))
                    return self._send(200, models_payload([
                        chat_model("openai/gpt-5.5", ("text", "image")),
                        chat_model("text/only-model", ("text",)),
                    ]))
                if parsed.path == "/cb":
                    # A real consent screen would bounce here; the test drives
                    # the loopback listener directly instead.
                    return self._send(200, {"ok": True})
                del query
                return self._send(404, {"error": "not_found"})

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.port = int(self._server.server_address[1])
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        return self.base_url

    @property
    def base_url(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
