"""Origin resolution and network policy for the OrcaRouter provider.

Authentication and inference live on **different public origins**:

* auth / consent / code exchange: ``https://www.orcarouter.ai``
* inference and model discovery:  ``https://api.orcarouter.ai/v1``

The two public origins are never derived from one another (no hostname
swapping, no blind ``/v1`` append).  Self-hosted deployments that run a single
shared origin are supported through ``ORCA_BASE_URL``, with
``ORCA_AUTH_BASE_URL`` / ``ORCA_API_BASE_URL`` taking precedence when set.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from urllib.parse import urlsplit

DEFAULT_AUTH_BASE = "https://www.orcarouter.ai"
DEFAULT_API_BASE = "https://api.orcarouter.ai/v1"

# Fixed protocol paths.  Note that the exchange endpoint is ``/api/v1/auth/keys``
# on the *auth* origin -- ``https://api.orcarouter.ai/v1/auth/keys`` is a 404.
AUTHORIZE_PATH = "/auth"
EXCHANGE_PATH = "/api/v1/auth/keys"
DEVICE_CODE_PATH = "/api/v1/auth/device/code"
DEVICE_TOKEN_PATH = "/api/v1/auth/device/token"

# Inference paths, relative to the API base (which already ends in ``/v1``).
MODELS_PATH = "/models"
CHAT_COMPLETIONS_PATH = "/chat/completions"
EMBEDDINGS_PATH = "/embeddings"
IMAGE_GENERATIONS_PATH = "/images/generations"

LOOPBACK_HOSTS = frozenset({"localhost", "127.0.0.1", "::1", "[::1]"})

AUTH_BASE_ENV = "ORCA_AUTH_BASE_URL"
API_BASE_ENV = "ORCA_API_BASE_URL"
SHARED_BASE_ENV = "ORCA_BASE_URL"
KEY_ENV = "ORCAROUTER_API_KEY"


class OriginError(ValueError):
    """Raised when a configured origin violates the network policy."""


@dataclass(frozen=True)
class OriginConfig:
    """Resolved auth + inference origins."""

    auth_base: str
    api_base: str

    @property
    def authorize_url(self) -> str:
        return self.auth_base + AUTHORIZE_PATH

    @property
    def exchange_url(self) -> str:
        return self.auth_base + EXCHANGE_PATH

    @property
    def device_code_url(self) -> str:
        return self.auth_base + DEVICE_CODE_PATH

    @property
    def device_token_url(self) -> str:
        return self.auth_base + DEVICE_TOKEN_PATH

    @property
    def models_url(self) -> str:
        return self.api_base + MODELS_PATH

    def api_url(self, path: str) -> str:
        return self.api_base + path


def is_loopback(host: str | None) -> bool:
    if not host:
        return False
    return host.lower() in LOOPBACK_HOSTS


def validate_origin(url: str, *, name: str) -> str:
    """Return ``url`` without a trailing slash, or raise ``OriginError``.

    HTTPS is required for every non-loopback origin.  Plain HTTP is permitted
    only for loopback development addresses.
    """
    parts = urlsplit(url)
    if parts.scheme not in ("http", "https"):
        raise OriginError(f"{name} must be an http(s) origin, got {url!r}")
    if parts.scheme == "http" and not is_loopback(parts.hostname):
        raise OriginError(
            f"{name} must use HTTPS for non-loopback hosts, got {url!r}"
        )
    if not parts.hostname:
        raise OriginError(f"{name} has no host: {url!r}")
    if parts.query or parts.fragment:
        raise OriginError(f"{name} must not carry a query or fragment: {url!r}")
    return url.rstrip("/")


def _shared_inference_base(shared: str) -> str:
    """Derive the inference base from a single shared self-hosted origin.

    This is the *documented* shared-origin fallback and only ever applies to a
    value the operator set explicitly in ``ORCA_BASE_URL``.  It is not used to
    derive one public origin from the other.
    """
    parts = urlsplit(shared)
    path = parts.path.rstrip("/")
    if path in ("", "/"):
        return shared.rstrip("/") + "/v1"
    return shared.rstrip("/")


def resolve_origins(env: dict | None = None) -> OriginConfig:
    """Resolve origins from the environment.

    Precedence, strongest first:
      1. ``ORCA_AUTH_BASE_URL`` / ``ORCA_API_BASE_URL`` (explicit overrides)
      2. ``ORCA_BASE_URL`` (shared self-hosted origin)
      3. the public OrcaRouter defaults
    """
    env = os.environ if env is None else env
    shared = (env.get(SHARED_BASE_ENV) or "").strip() or None

    auth_raw = (env.get(AUTH_BASE_ENV) or "").strip() or shared or DEFAULT_AUTH_BASE
    if (env.get(API_BASE_ENV) or "").strip():
        api_raw = env[API_BASE_ENV].strip()
    elif shared:
        api_raw = _shared_inference_base(shared)
    else:
        api_raw = DEFAULT_API_BASE

    return OriginConfig(
        auth_base=validate_origin(auth_raw, name=AUTH_BASE_ENV),
        api_base=validate_origin(api_raw, name=API_BASE_ENV),
    )
