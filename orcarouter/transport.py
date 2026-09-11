"""A very small HTTP transport with hard bounds.

Only the standard library is used -- SHA-256 and base64url come from ``hashlib``
and ``base64``, so no dependency is added for the PKCE flow.  Responses are
bounded in time and size so a misbehaving catalog or auth endpoint cannot
exhaust memory.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Mapping, Protocol

from .errors import NetworkError

DEFAULT_TIMEOUT = 30.0
#: Catalog responses are allowed to be larger than auth responses; both are
#: still capped so a hostile or broken server cannot stream unbounded data.
MAX_RESPONSE_BYTES = 8 * 1024 * 1024
USER_AGENT = "v-rep-youbot-demo-orcarouter/1.0"


@dataclass(frozen=True)
class HttpResult:
    status: int
    body: bytes
    headers: Mapping[str, str]

    def json(self) -> object:
        try:
            return json.loads(self.body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise NetworkError(f"response was not valid JSON: {exc}") from exc


class Transport(Protocol):
    """Injectable seam so tests never touch the network."""

    def request(
        self,
        method: str,
        url: str,
        *,
        json_body: dict | None = None,
        form_body: Mapping[str, str] | None = None,
        headers: Mapping[str, str] | None = None,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> HttpResult: ...


class UrllibTransport:
    """Default transport.  Never raises for HTTP status codes; only for I/O."""

    def __init__(self, max_bytes: int = MAX_RESPONSE_BYTES) -> None:
        self.max_bytes = max_bytes

    def request(
        self,
        method: str,
        url: str,
        *,
        json_body: dict | None = None,
        form_body: Mapping[str, str] | None = None,
        headers: Mapping[str, str] | None = None,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> HttpResult:
        data: bytes | None = None
        hdrs = {"User-Agent": USER_AGENT, "Accept": "application/json"}
        if headers:
            hdrs.update(headers)
        if json_body is not None:
            data = json.dumps(json_body).encode("utf-8")
            hdrs["Content-Type"] = "application/json"
        elif form_body is not None:
            data = urllib.parse.urlencode(dict(form_body)).encode("utf-8")
            hdrs["Content-Type"] = "application/x-www-form-urlencoded"

        req = urllib.request.Request(url, data=data, headers=hdrs, method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                body = resp.read(self.max_bytes + 1)
                if len(body) > self.max_bytes:
                    raise NetworkError("response exceeded the configured size cap")
                return HttpResult(resp.status, body, dict(resp.headers))
        except urllib.error.HTTPError as exc:
            body = exc.read(self.max_bytes + 1)
            return HttpResult(exc.code, body, dict(exc.headers or {}))
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise NetworkError(f"request to {url} failed: {exc}") from exc
