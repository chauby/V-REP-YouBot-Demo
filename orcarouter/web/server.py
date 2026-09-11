"""A small local HTTP server that hosts the OrcaRouter settings UI.

It binds ``127.0.0.1`` only.  The browser is a *view*: it never receives a
stored credential, and every catalog fetch and inference call happens
server-side with the key read from the project's credential store.
"""

from __future__ import annotations

import json
import mimetypes
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlsplit

from ..errors import CredentialError, OrcaRouterError, TerminalReauthRequired
from .service import OrcaRouterService

STATIC_DIR = Path(__file__).parent / "static"
#: The authorization hint shown while a login is in flight.  Kept small so a
#: ``pagehide`` cancellation can clear it synchronously.
PAGE_HIDE_CANCEL_PATH = "/api/connect/cancel"


class OrcaRouterRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "OrcaRouterUI/1.0"
    service: OrcaRouterService  # injected by the server

    # -- plumbing -----------------------------------------------------------

    def log_message(self, fmt: str, *args) -> None:
        # Never log request bodies: they can carry a pasted API key.
        if self.path.startswith("/static"):
            return
        super().log_message("%s", fmt % args)

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload: Any, status: int = 200) -> None:
        self._send(
            status,
            json.dumps(payload).encode("utf-8"),
            "application/json; charset=utf-8",
        )

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise CredentialError("request body was not valid JSON") from None
        return data if isinstance(data, dict) else {}

    # -- routes -------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlsplit(self.path)
        path = parsed.path
        if path in ("/", "/index.html"):
            return self._serve_static("index.html")
        if path.startswith("/static/"):
            return self._serve_static(path[len("/static/") :])
        if path == "/api/status":
            return self._json(self.service.status())
        if path == "/api/providers":
            return self._json({"providers": self.service.providers()})
        if path == "/api/models":
            query = parse_qs(parsed.query)
            capability = (query.get("capability") or ["chat"])[0]
            modalities = (query.get("modalities") or [""])[0]
            try:
                body = self.service.models(capability, modalities)
            except TerminalReauthRequired as exc:
                return self._json(
                    {"ok": False, "error": "needs_reauth", "detail": str(exc)}, 401
                )
            except OrcaRouterError as exc:
                return self._json({"ok": False, "error": str(exc)}, 502)
            self._send(200, body.encode("utf-8"), "application/json; charset=utf-8")
            return
        if path == "/api/connect/poll":
            query = parse_qs(parsed.query)
            generation = int((query.get("generation") or ["0"])[0])
            return self._json(self.service.connect_poll(generation))
        return self._json({"ok": False, "error": "not found"}, 404)

    def do_POST(self) -> None:  # noqa: N802
        path = urlsplit(self.path).path
        try:
            payload = self._read_json()
        except CredentialError as exc:
            return self._json({"ok": False, "error": str(exc)}, 400)

        try:
            if path == "/api/credentials/api-key":
                result = self.service.save_api_key(payload.get("api_key") or "")
                # Never echo the key back: the response carries only a mask.
                return self._json(result)
            if path == "/api/credentials/clear":
                return self._json(
                    self.service.clear_credential(payload.get("source_id") or "")
                )
            if path == "/api/connect/start":
                return self._json(
                    self.service.connect_start(payload.get("flow") or "A")
                )
            if path == "/api/connect/complete":
                return self._json(
                    self.service.connect_complete(
                        int(payload.get("generation") or 0), payload.get("code") or ""
                    )
                )
            if path == PAGE_HIDE_CANCEL_PATH:
                generation = payload.get("generation")
                return self._json(
                    self.service.connect_cancel(
                        int(generation) if generation is not None else None
                    )
                )
            if path == "/api/chat":
                return self._json(
                    self.service.chat(
                        payload.get("model") or "",
                        payload.get("messages") or [],
                    )
                )
        except TerminalReauthRequired as exc:
            return self._json(
                {"ok": False, "error": "needs_reauth", "detail": str(exc)}, 401
            )
        except OrcaRouterError as exc:
            return self._json({"ok": False, "error": str(exc)}, 400)
        return self._json({"ok": False, "error": "not found"}, 404)

    def _serve_static(self, relative: str) -> None:
        candidate = (STATIC_DIR / relative).resolve()
        if not str(candidate).startswith(str(STATIC_DIR.resolve())) or not candidate.is_file():
            return self._json({"ok": False, "error": "not found"}, 404)
        content_type = mimetypes.guess_type(str(candidate))[0] or "application/octet-stream"
        if candidate.suffix in (".js", ".css", ".html", ".svg"):
            content_type += "; charset=utf-8"
        self._send(200, candidate.read_bytes(), content_type)


class OrcaRouterUIServer(ThreadingHTTPServer):
    """Loopback-only server hosting the settings UI."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, service: OrcaRouterService, host: str = "127.0.0.1", port: int = 0):
        handler = type(
            "BoundHandler", (OrcaRouterRequestHandler,), {"service": service}
        )
        super().__init__((host, port), handler)
        self.service = service

    @property
    def port(self) -> int:
        return int(self.server_address[1])

    def start_background(self) -> threading.Thread:
        thread = threading.Thread(target=self.serve_forever, daemon=True)
        thread.start()
        return thread
