"""Application service behind the OrcaRouter settings UI.

Everything that touches a credential lives here, on the server side:

* the API key is written straight into the project's credential store and is
  never echoed back to the browser (only a masked form is);
* the model catalog is fetched server-side with the stored key, so the browser
  receives minimal model metadata and never a credential;
* the PKCE login lock is held here and released on every terminal path,
  including a ``pagehide`` cancellation from the page.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from ..adapters.api_key import ApiKeyCredentialSource
from ..adapters.pkce import PkceCredentialSource
from ..catalog import (
    CAPABILITY_CHAT,
    CAPABILITY_EMBEDDING,
    CAPABILITY_IMAGE,
    CAPABILITY_RERANK,
    CAPABILITY_VIDEO,
    CatalogCache,
    CatalogSnapshot,
    catalog_as_json,
    normalize_modalities,
)
from ..config import OriginConfig, resolve_origins
from ..credentials import (
    SOURCE_API_KEY,
    SOURCE_PKCE,
    CredentialResult,
    CredentialStore,
)
from ..errors import (
    AuthorizationCancelled,
    CredentialError,
    OrcaRouterError,
    TerminalReauthRequired,
)
from ..lifecycle import CredentialLifecycle, ReauthState
from ..provider import (
    ORCAROUTER_API_PROVIDER_ID,
    ORCAROUTER_AUTH_PROVIDER_ID,
    OrcaRouterClient,
    PROVIDERS,
)
from ..transport import Transport, UrllibTransport

#: Capabilities the settings UI can preview.  Each maps to one AI input surface.
CAPABILITY_SURFACES = (
    {"id": CAPABILITY_CHAT, "label": "Text chat", "modalities": []},
    {"id": CAPABILITY_CHAT, "label": "Image understanding", "modalities": ["image"]},
    {"id": CAPABILITY_EMBEDDING, "label": "Embeddings", "modalities": []},
    {"id": CAPABILITY_IMAGE, "label": "Image generation", "modalities": []},
    {"id": CAPABILITY_VIDEO, "label": "Video generation", "modalities": []},
    {"id": CAPABILITY_RERANK, "label": "Rerank", "modalities": []},
)


@dataclass
class ConnectSession:
    generation: int
    flow: str
    authorize_url: str | None = None


class OrcaRouterService:
    """Thread-safe service facade for the local UI and the CLI."""

    def __init__(
        self,
        *,
        store: CredentialStore | None = None,
        origins: OriginConfig | None = None,
        transport: Transport | None = None,
        open_browser=None,
    ) -> None:
        self.store = store or CredentialStore()
        self.origins = origins or resolve_origins()
        self.transport = transport or UrllibTransport()
        self.lifecycle = CredentialLifecycle(self.store)
        self.catalog = CatalogCache()
        self.api_key = ApiKeyCredentialSource(self.store)
        self.pkce = PkceCredentialSource(
            self.store,
            origins=self.origins,
            transport=self.transport,
            open_browser=open_browser,
        )
        self._lock = threading.Lock()
        self._session: ConnectSession | None = None
        # Release the server-side login lock when the worker itself reaches a
        # terminal state (timeout, denial, exchange error).  Without this an
        # abandoned Flow A login -- browser killed, no pagehide -- would keep
        # `login_busy` true until a client poll happened to arrive.
        self.pkce.on_terminal(self._release)

    # -- registry / status --------------------------------------------------

    def providers(self) -> list[dict]:
        return [spec.to_public_dict() for spec in PROVIDERS]

    @staticmethod
    def resolve_source_id(identifier: str) -> str:
        """Map a provider id or a credential-source id to the source id.

        The UI works in provider ids and the credential store in source ids;
        the registry is the single place that links them, so neither side has
        to guess.
        """
        for spec in PROVIDERS:
            if identifier == spec.id:
                return spec.credential_source
        if identifier in (SOURCE_API_KEY, SOURCE_PKCE):
            return identifier
        raise CredentialError(f"unknown credential source {identifier!r}")

    def status(self) -> dict[str, Any]:
        return {
            "providers": self.providers(),
            "accounts": [s.to_public_dict() for s in self.lifecycle.all_status()],
            "origins": {
                "auth_base": self.origins.auth_base,
                "api_base": self.origins.api_base,
            },
            "surfaces": [dict(s) for s in CAPABILITY_SURFACES],
            "login_busy": self._session is not None,
        }

    # -- API-key adapter ----------------------------------------------------

    def save_api_key(self, api_key: str) -> dict:
        result = self.api_key.obtain(api_key)
        self.lifecycle.refresh_after_login(SOURCE_API_KEY)
        self.catalog.invalidate()
        return {
            "ok": True,
            "credential": result.to_public_dict(),
            "status": self.lifecycle.status(SOURCE_API_KEY).to_public_dict(),
        }

    def clear_credential(self, source_id: str) -> dict:
        """Clear a credential.  Accepts either a provider id or a source id."""
        resolved = self.resolve_source_id(source_id)
        if resolved == SOURCE_API_KEY:
            self.api_key.clear()
        elif resolved == SOURCE_PKCE:
            self.pkce.clear()
        with self._lock:
            self._session = None
        self.catalog.invalidate()
        return {"ok": True, "status": self.lifecycle.status(resolved).to_public_dict()}

    # -- PKCE adapter -------------------------------------------------------

    def connect_start(self, flow: str = "A") -> dict:
        with self._lock:
            if self._session is not None:
                # A single login at a time; the UI shows this as busy.
                return {
                    "ok": False,
                    "busy": True,
                    "generation": self._session.generation,
                    "authorize_url": self._session.authorize_url,
                    "flow": self._session.flow,
                }
            attempt = self.pkce.start(flow=flow)  # type: ignore[arg-type]
            self._session = ConnectSession(
                generation=attempt.generation,
                flow=attempt.flow,
                authorize_url=attempt.authorize_url,
            )
        return {
            "ok": True,
            "busy": False,
            "generation": attempt.generation,
            "flow": attempt.flow,
            "authorize_url": attempt.authorize_url,
        }

    def connect_poll(self, generation: int) -> dict:
        session = self._session
        if session is None or session.generation != generation:
            return {"ok": False, "state": "superseded", "generation": generation}
        if self.pkce.is_cancelled(generation):
            return {"ok": False, "state": "superseded", "generation": generation}
        try:
            result = self.pkce.poll(generation)
        except AuthorizationCancelled as exc:
            self._release(generation)
            return {"ok": False, "state": "error", "error": str(exc)}
        except OrcaRouterError as exc:
            self._release(generation)
            return {"ok": False, "state": "error", "error": str(exc)}
        if result is None:
            return {
                "ok": True,
                "state": "pending",
                "generation": generation,
                "authorize_url": session.authorize_url,
            }
        self._release(generation)
        self.catalog.invalidate()
        return {
            "ok": True,
            "state": "done",
            "generation": generation,
            "credential": result.to_public_dict(),
            "status": self.lifecycle.status(SOURCE_PKCE).to_public_dict(),
        }

    def connect_complete(self, generation: int, code: str) -> dict:
        if self.pkce.is_cancelled(generation):
            return {"ok": False, "state": "superseded", "generation": generation}
        try:
            result = self.pkce.complete(generation, code)
        except OrcaRouterError as exc:
            self._release(generation)
            return {"ok": False, "state": "error", "error": str(exc)}
        self._release(generation)
        self.catalog.invalidate()
        return {
            "ok": True,
            "state": "done",
            "credential": result.to_public_dict(),
            "status": self.lifecycle.status(SOURCE_PKCE).to_public_dict(),
        }

    def connect_cancel(self, generation: int | None = None) -> dict:
        """Release the login lock for one attempt.

        Called by the explicit Cancel button, by provider switches, and by the
        page's ``pagehide`` handler.  Cancellation is scoped to a generation,
        so a delayed ``pagehide`` beacon cannot abort a login the user started
        afterwards.  A fresh generation is then issued so any late response
        from the cancelled attempt is recognised as stale and dropped.
        """
        with self._lock:
            session = self._session
            target = generation if generation is not None else (
                session.generation if session else None
            )
            # Only release the lock if it belongs to the attempt being
            # cancelled (or if there is nothing in flight).
            if session is not None and target is not None and session.generation != target:
                return {"ok": True, "cancelled": target, "busy": True}
            self._session = None
        self.pkce.cancel(target)
        self.pkce.next_generation()
        return {
            "ok": True,
            "cancelled": target,
            "busy": False,
        }

    def _release(self, generation: int | None = None) -> None:
        """Release the login lock, optionally only for one generation."""
        with self._lock:
            if self._session is None:
                return
            if generation is None or self._session.generation == generation:
                self._session = None

    # -- model catalog ------------------------------------------------------

    def models(self, capability: str = CAPABILITY_CHAT, modalities: str = "") -> str:
        required = normalize_modalities(modalities)
        credential = self._usable_credential()
        api_key = credential.api_key if credential else None
        snapshot = self.catalog.refresh(
            capability,
            required,
            origins=self.origins,
            api_key=api_key,
            transport=self.transport,
        )
        return catalog_as_json(snapshot)

    def _usable_credential(self) -> CredentialResult | None:
        for source_id in (SOURCE_API_KEY, SOURCE_PKCE):
            status = self.lifecycle.status(source_id)
            if status.state is ReauthState.READY:
                try:
                    return self.lifecycle.credential_for(source_id)
                except OrcaRouterError:
                    continue
        return None

    # -- inference ----------------------------------------------------------

    def chat(self, model: str, messages: list, **options) -> dict:
        """Send a real completion through the provider code path.

        Returns the raw OpenAI-compatible payload; a terminal 401 marks the
        exact account generation for reauthentication.
        """
        credential = self._usable_credential()
        if credential is None:
            raise CredentialError(
                "no usable OrcaRouter credential; add an API key or connect "
                "an account first"
            )
        client = OrcaRouterClient(
            credential, origins=self.origins, transport=self.transport
        )
        try:
            payload = client.chat_completion(model, messages, **options)
        except TerminalReauthRequired:
            self.lifecycle.mark_rejected(credential)
            raise
        return {"ok": True, "response": payload, "model": model}
