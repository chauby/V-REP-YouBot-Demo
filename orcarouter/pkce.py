"""OAuth 2.0 + PKCE against the OrcaRouter consent screen (RFC 7636).

Two flows are implemented, both producing a normal OrcaRouter API key:

* **Flow A -- loopback redirect** (``callback_url=http://127.0.0.1:<port>/cb``):
  used by the local web UI and the CLI, which can bind a loopback port.
* **Flow B -- out-of-band code** (``callback_url=oob``): used when the client
  cannot receive a redirect -- a remote/hosted deployment, a locked-down
  machine, or a display-only surface.

Flow C (device grant) is not implemented; these clients always have a browser.

Security properties enforced here:

* a fresh verifier and ``state`` per attempt, from ``secrets.token_bytes``;
* ``code_challenge = base64url(sha256(verifier))`` with **no padding**, and
  ``code_challenge_method=S256`` always -- never ``plain``, because the user may
  choose "show me a code" even on Flow A;
* the verifier never leaves this process until the exchange, and is never put in
  a URL, a log line, an error message or telemetry;
* ``state`` is compared with :func:`hmac.compare_digest` *before* the code is
  used.
"""

from __future__ import annotations

import base64
import hashlib
import hmac
import http.server
import secrets
import threading
import time
import urllib.parse
import webbrowser
from dataclasses import dataclass, field
from typing import Callable, Mapping

from .config import OriginConfig
from .credentials import CredentialResult, SOURCE_PKCE, redact
from .errors import (
    AccessDenied,
    AuthorizationCancelled,
    AuthorizationTimeout,
    ChallengeRejected,
    CodeRejected,
    CredentialError,
    NetworkError,
    RateLimited,
    ScopeDowngrade,
    StateMismatch,
)
from .transport import DEFAULT_TIMEOUT, Transport, UrllibTransport

#: Scope this client needs.  OrcaRouter grants ``api`` or ``connector``;
#: anything else is refused before a code is minted.
REQUIRED_SCOPE = "api"
AUTHORIZATION_TTL_SECONDS = 600.0
LOOPBACK_CALLBACK_PATH = "/cb"


def b64url(raw: bytes) -> str:
    """base64url without padding, as PKCE requires."""
    return base64.urlsafe_b64encode(raw).decode("ascii").rstrip("=")


def generate_verifier() -> str:
    """A fresh 256-bit verifier from a cryptographic RNG."""
    return b64url(secrets.token_bytes(32))


def challenge_for(verifier: str) -> str:
    """``base64url(sha256(verifier))``, no padding."""
    return b64url(hashlib.sha256(verifier.encode("ascii")).digest())


def generate_state() -> str:
    """A fresh opaque CSRF token."""
    return b64url(secrets.token_bytes(16))


@dataclass
class PkceAttempt:
    """One authorization attempt.  Never logged or serialised."""

    verifier: str
    state: str
    challenge: str
    started_at: float
    scope: str = REQUIRED_SCOPE
    #: Set by the host UI so a stale attempt cannot overwrite a newer login.
    generation: int = 0

    def expired(self, ttl: float = AUTHORIZATION_TTL_SECONDS) -> bool:
        return (time.monotonic() - self.started_at) > ttl

    def __repr__(self) -> str:  # pragma: no cover - defensive
        # Deliberately omits verifier and state.
        return f"PkceAttempt(challenge={self.challenge[:8]}…, generation={self.generation})"


def new_attempt(generation: int = 0, scope: str = REQUIRED_SCOPE) -> PkceAttempt:
    verifier = generate_verifier()
    return PkceAttempt(
        verifier=verifier,
        state=generate_state(),
        challenge=challenge_for(verifier),
        started_at=time.monotonic(),
        scope=scope,
        generation=generation,
    )


def build_authorize_url(
    origins: OriginConfig,
    attempt: PkceAttempt,
    *,
    app_name: str,
    callback_url: str,
    login_hint: str | None = None,
    workspace_hint: str | None = None,
    prompt: str | None = None,
) -> str:
    """Build the consent URL.

    It carries the *challenge* only.  The verifier stays in this process.
    """
    params = {
        "callback_url": callback_url,
        "code_challenge": attempt.challenge,
        "code_challenge_method": "S256",
        "state": attempt.state,
        "app_name": app_name,
        "scope": attempt.scope,
    }
    if login_hint:
        params["login_hint"] = login_hint
    if workspace_hint:
        params["workspace_hint"] = workspace_hint
    if prompt:
        params["prompt"] = prompt
    return f"{origins.authorize_url}?{urllib.parse.urlencode(params)}"


@dataclass(frozen=True)
class ExchangeResult:
    """What the exchange endpoint returned."""

    key: str
    scope: str | None
    user_id: str | None

    def to_credential(self, generation: int = 1) -> CredentialResult:
        return CredentialResult(
            api_key=self.key,
            source=SOURCE_PKCE,
            account_id=self.user_id,
            scope=self.scope,
            generation=generation,
        )


def _parse_error_body(body: object) -> tuple[str, str]:
    if isinstance(body, Mapping):
        err = str(body.get("error") or body.get("error_description") or "")
        desc = str(body.get("error_description") or body.get("message") or "")
        return err, desc
    return "", ""


def exchange_code(
    origins: OriginConfig,
    code: str,
    verifier: str,
    *,
    transport: Transport | None = None,
    timeout: float = DEFAULT_TIMEOUT,
    required_scope: str = REQUIRED_SCOPE,
) -> ExchangeResult:
    """Redeem ``code`` at ``<auth-base>/api/v1/auth/keys``.

    The exchange always goes to the *auth* origin.  ``https://api.orcarouter.ai
    /v1/auth/keys`` is a 404 and is never used.
    """
    transport = transport or UrllibTransport()
    payload = {
        "code": code,
        "code_verifier": verifier,
        "code_challenge_method": "S256",
    }
    try:
        result = transport.request(
            "POST", origins.exchange_url, json_body=payload, timeout=timeout
        )
    except NetworkError:
        raise
    except Exception as exc:  # pragma: no cover - transport specific
        raise NetworkError(redact(str(exc))) from exc

    body = _safe_json(result)
    err, desc = _parse_error_body(body)

    if result.status == 200:
        if not isinstance(body, Mapping) or not body.get("key"):
            raise CredentialError(
                "OrcaRouter returned no key for this code; start the connect "
                "flow again"
            )
        granted = body.get("scope")
        # The response reports what was *granted*, not what we asked for.
        if required_scope and granted != required_scope:
            raise ScopeDowngrade(granted, required_scope)
        return ExchangeResult(
            key=str(body["key"]),
            scope=str(granted) if granted is not None else None,
            user_id=str(body["user_id"]) if body.get("user_id") else None,
        )
    if result.status == 400:
        raise ChallengeRejected(
            "OrcaRouter rejected the PKCE challenge for this code "
            f"({err or 'invalid_request'}); start the connect flow again"
        )
    if result.status == 403:
        raise CodeRejected(
            "This authorization code is unknown, expired, or already used "
            "(codes are single-use with a 10 minute lifetime); start the "
            "connect flow again"
        )
    if result.status == 429:
        raise RateLimited(
            "OrcaRouter is rate limiting key issuance (at most 10 keys per "
            "user per 24 hours); reuse the stored key or wait before "
            "connecting again"
        )
    raise CredentialError(
        f"OrcaRouter exchange failed with HTTP {result.status}"
        + (f": {err}" if err else "")
    )


def _safe_json(result) -> object:
    try:
        return result.json()
    except NetworkError:
        return {}


def check_state(received: str | None, expected: str) -> None:
    """Constant-time comparison of the CSRF token, before the code is used."""
    if not received or not hmac.compare_digest(received, expected):
        raise StateMismatch(
            "the authorization response did not match this login attempt "
            "(state mismatch); nothing was saved, please try again"
        )


class LoopbackReceiver:
    """Flow A callback listener bound to ``127.0.0.1`` on an ephemeral port.

    The listener starts *before* the browser opens so the port is known and
    there is no race.
    """

    def __init__(self, expected_state: str, path: str = LOOPBACK_CALLBACK_PATH):
        self.expected_state = expected_state
        self.path = path
        self._code: str | None = None
        self._error: BaseException | None = None
        self._event = threading.Event()
        self._server: http.server.HTTPServer | None = None
        self._lock = threading.Lock()

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> int:
        receiver = self

        class Handler(http.server.BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *_args) -> None:  # keep secrets out of logs
                return

            def do_GET(self) -> None:  # noqa: N802 (stdlib naming)
                parsed = urllib.parse.urlsplit(self.path)
                if parsed.path != receiver.path:
                    self.send_response(404)
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                query = urllib.parse.parse_qs(parsed.query)
                receiver.deliver(
                    query.get("code", [None])[0],
                    query.get("error", [None])[0],
                    query.get("state", [None])[0],
                )
                body = (
                    b"<html><body style='font-family:sans-serif'>"
                    b"<h3>OrcaRouter connected.</h3>"
                    b"<p>You can close this tab and return to the app.</p>"
                    b"</body></html>"
                )
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(body)

        server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
        server.timeout = 1.0
        self._server = server
        return int(server.server_address[1])

    def serve_until_done(self, timeout: float) -> None:
        """Serve requests until a callback is delivered or ``timeout`` passes."""
        deadline = time.monotonic() + timeout
        server = self._server
        assert server is not None
        while not self._event.is_set() and time.monotonic() < deadline:
            server.handle_request()

    def deliver(self, code: str | None, error: str | None, state: str | None) -> None:
        """Record a callback.  ``state`` is verified before anything else."""
        with self._lock:
            if self._event.is_set():
                return
            try:
                check_state(state, self.expected_state)
            except StateMismatch as exc:
                self._error = exc
                self._event.set()
                return
            if error:
                if error == "access_denied":
                    self._error = AccessDenied("OrcaRouter authorization was denied")
                else:
                    self._error = CredentialError(
                        f"OrcaRouter authorization failed: {error}"
                    )
                self._event.set()
                return
            if not code:
                self._error = CredentialError(
                    "the authorization callback carried no code"
                )
                self._event.set()
                return
            self._code = code
            self._event.set()

    def wait(self, remaining: float) -> str:
        if not self._event.wait(timeout=max(0.0, remaining)):
            raise AuthorizationTimeout(
                "OrcaRouter authorization timed out before the browser "
                "returned a code; start the connect flow again"
            )
        if self._error is not None:
            raise self._error
        assert self._code is not None
        return self._code

    def close(self) -> None:
        if self._server is not None:
            try:
                self._server.server_close()
            finally:
                self._server = None

    def __enter__(self) -> "LoopbackReceiver":
        self.start()
        return self

    def __exit__(self, *_exc) -> None:
        self.close()


def authorize_flow_a(
    origins: OriginConfig,
    *,
    app_name: str,
    attempt: PkceAttempt,
    open_browser: Callable[[str], object] | None = None,
    timeout: float = AUTHORIZATION_TTL_SECONDS,
    on_url: Callable[[str], None] | None = None,
    is_cancelled: Callable[[], bool] | None = None,
) -> str:
    """Flow A: run the loopback listener, open the browser, return the code."""
    if timeout <= 0:
        raise AuthorizationTimeout(
            "OrcaRouter authorization timed out before the browser "
            "returned a code; start the connect flow again"
        )
    with LoopbackReceiver(attempt.state) as receiver:
        port = int(receiver._server.server_address[1])  # type: ignore[union-attr]
        url = build_authorize_url(
            origins,
            attempt,
            app_name=app_name,
            callback_url=f"http://127.0.0.1:{port}{LOOPBACK_CALLBACK_PATH}",
        )
        if on_url:
            on_url(url)
        opener = open_browser or webbrowser.open
        try:
            opener(url)
        except Exception:
            # A browser that will not open is not fatal: the caller has the URL.
            pass

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if is_cancelled and is_cancelled():
                raise AuthorizationCancelled("OrcaRouter authorization cancelled")
            receiver.serve_until_done(timeout=min(0.5, max(0.0, deadline - time.monotonic())))
            if receiver._event.is_set():
                break
        return receiver.wait(max(0.0, deadline - time.monotonic()))


def authorize_flow_b(
    origins: OriginConfig,
    *,
    app_name: str,
    state: str,
    challenge: str,
    scope: str = REQUIRED_SCOPE,
    open_browser: Callable[[str], object] | None = None,
    on_url: Callable[[str], None] | None = None,
) -> str:
    """Flow B: return the consent URL; the user pastes the displayed code back.

    ``callback_url=oob`` and a mandatory S256 challenge.
    """
    url = f"{origins.authorize_url}?" + urllib.parse.urlencode(
        {
            "callback_url": "oob",
            "code_challenge": challenge,
            "code_challenge_method": "S256",
            "state": state,
            "app_name": app_name,
            "scope": scope,
        }
    )
    if on_url:
        on_url(url)
    opener = open_browser or webbrowser.open
    try:
        opener(url)
    except Exception:
        pass
    return url
