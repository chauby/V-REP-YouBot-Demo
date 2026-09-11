"""The provider registry and the OpenAI-compatible inference client.

OrcaRouter appears as a **first-class named provider** with two entries, one
per authentication choice.  Both share the same inference base URL
(``https://api.orcarouter.ai/v1``), the same ``Bearer`` transport and the same
model namespace; only credential acquisition differs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping

from .config import (
    CHAT_COMPLETIONS_PATH,
    DEFAULT_API_BASE,
    DEFAULT_AUTH_BASE,
    EMBEDDINGS_PATH,
    IMAGE_GENERATIONS_PATH,
    KEY_ENV,
    OriginConfig,
    resolve_origins,
)
from .credentials import SOURCE_API_KEY, SOURCE_PKCE, CredentialResult
from .errors import ApiError, ConfigurationError, NetworkError, TerminalReauthRequired
from .transport import DEFAULT_TIMEOUT, Transport, UrllibTransport

ORCAROUTER_API_PROVIDER_ID = "orcarouter"
ORCAROUTER_AUTH_PROVIDER_ID = "orcarouter-oauth"


@dataclass(frozen=True)
class ProviderSpec:
    """One selectable provider entry in the registry."""

    id: str
    label: str
    auth_method: str  # "api_key" | "oauth_pkce"
    #: The credential-seam adapter this entry drives.  Lets a view join a
    #: credential's status back to the provider card that owns it.
    credential_source: str = SOURCE_API_KEY
    base_url: str = DEFAULT_API_BASE
    auth_base_url: str = DEFAULT_AUTH_BASE
    env_key: str = KEY_ENV
    wire_format: str = "openai"
    description: str = ""
    key_dashboard_url: str = "https://www.orcarouter.ai/console/authorized-apps"

    def to_public_dict(self) -> dict:
        return {
            "id": self.id,
            "label": self.label,
            "auth_method": self.auth_method,
            "credential_source": self.credential_source,
            "base_url": self.base_url,
            "env_key": self.env_key,
            "wire_format": self.wire_format,
            "description": self.description,
            "key_dashboard_url": self.key_dashboard_url,
        }


def _specs(origins: OriginConfig) -> tuple[ProviderSpec, ...]:
    return (
        ProviderSpec(
            id=ORCAROUTER_API_PROVIDER_ID,
            label="OrcaRouter - API",
            auth_method="api_key",
            credential_source=SOURCE_API_KEY,
            base_url=origins.api_base,
            auth_base_url=origins.auth_base,
            description=(
                "Use an existing OrcaRouter API key. Works without a browser "
                "and is the right choice for CI, containers and shared machines."
            ),
        ),
        ProviderSpec(
            id=ORCAROUTER_AUTH_PROVIDER_ID,
            label="OrcaRouter - Auth",
            auth_method="oauth_pkce",
            credential_source=SOURCE_PKCE,
            base_url=origins.api_base,
            auth_base_url=origins.auth_base,
            description=(
                "Sign in with your OrcaRouter account (OAuth 2.0 + PKCE). No "
                "client secret and no pre-registered redirect URI; the browser "
                "issues a normal sk-orca-… key for your account."
            ),
        ),
    )


#: Registry of OrcaRouter provider entries, populated for the resolved origins.
PROVIDERS: tuple[ProviderSpec, ...] = _specs(resolve_origins())

PROVIDER_BY_ID: dict[str, ProviderSpec] = {spec.id: spec for spec in PROVIDERS}


def get_provider(provider_id: str) -> ProviderSpec:
    try:
        return PROVIDER_BY_ID[provider_id]
    except KeyError:
        raise ConfigurationError(f"unknown OrcaRouter provider id {provider_id!r}") from None


@dataclass
class ChatMessage:
    role: str
    content: object


class OrcaRouterClient:
    """OpenAI-compatible client bound to one credential.

    The client is deliberately credential-source agnostic: it is constructed
    from a :class:`CredentialResult`, so a pasted key and a PKCE-issued key are
    indistinguishable from here down.
    """

    def __init__(
        self,
        credential: CredentialResult,
        *,
        origins: OriginConfig | None = None,
        transport: Transport | None = None,
        provider_id: str | None = None,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> None:
        self.credential = credential
        self.origins = origins or resolve_origins()
        self.transport = transport or UrllibTransport()
        self.timeout = timeout
        self.provider_id = provider_id or (
            ORCAROUTER_AUTH_PROVIDER_ID
            if credential.source == SOURCE_PKCE
            else ORCAROUTER_API_PROVIDER_ID
        )

    @property
    def spec(self) -> ProviderSpec:
        return ProviderSpec(
            id=self.provider_id,
            label=self.provider_id,
            auth_method="pkce" if self.provider_id == ORCAROUTER_AUTH_PROVIDER_ID else "api_key",
            base_url=self.origins.api_base,
            auth_base_url=self.origins.auth_base,
        )

    # -- transport ----------------------------------------------------------

    def _request(self, path: str, payload: Mapping) -> object:
        headers = {
            "Authorization": f"Bearer {self.credential.api_key}",
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        try:
            result = self.transport.request(
                "POST",
                self.origins.api_url(path),
                json_body=dict(payload),
                headers=headers,
                timeout=self.timeout,
            )
        except NetworkError:
            raise
        if result.status == 200:
            return result.json()
        if result.status in (401, 403):
            # Terminal: a durable key has no refresh grant.  The owning account
            # is marked for reauthentication; we never retry or fake a refresh.
            raise TerminalReauthRequired(
                account_id=self.credential.account_id,
                generation=self.credential.generation,
            )
        raise ApiError(result.status, _error_message(result))

    # -- endpoints ----------------------------------------------------------

    def chat_completion(
        self,
        model: str,
        messages: Iterable[Mapping | ChatMessage],
        **options,
    ) -> object:
        """``POST /chat/completions`` -- the text/vision entry point."""
        return self._request(
            CHAT_COMPLETIONS_PATH,
            {
                "model": model,
                "messages": [_message(m) for m in messages],
                **options,
            },
        )

    def embeddings(self, model: str, inputs, **options) -> object:
        return self._request(
            EMBEDDINGS_PATH, {"model": model, "input": inputs, **options}
        )

    def generate_image(self, model: str, prompt: str, **options) -> object:
        return self._request(
            IMAGE_GENERATIONS_PATH, {"model": model, "prompt": prompt, **options}
        )


def _message(message: Mapping | ChatMessage) -> dict:
    if isinstance(message, ChatMessage):
        return {"role": message.role, "content": message.content}
    return dict(message)


def _error_message(result) -> str:
    from .credentials import redact

    try:
        body = result.json()
    except Exception:
        return "unparseable error body"
    if isinstance(body, Mapping):
        for key in ("error", "message", "detail"):
            value = body.get(key)
            if isinstance(value, Mapping):
                value = value.get("message") or value.get("type")
            if value:
                return redact(str(value))[:300]
    return redact(result.body.decode("utf-8", "replace"))[:300]
