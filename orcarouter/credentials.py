"""The credential seam.

Both authentication choices produce a :class:`CredentialResult` holding a
normal OrcaRouter API key.  Everything downstream -- the inference client, the
model catalog, the GUI -- consumes that one object and never asks where it came
from.  Adding a third acquisition method later means adding one
:class:`CredentialSource`, not touching the provider.

The PKCE-issued key is **durable, not refreshable**: OrcaRouter returns a
long-lived key with no refresh grant, so it is reused until the user revokes it.
"""

from __future__ import annotations

import json
import os
import re
import stat
import tempfile
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .config import KEY_ENV
from .errors import CredentialError

#: ``sk-orca-`` followed by the key body.  Used for redaction only -- a prefix
#: check is a typo guard, never proof that a credential is valid.
KEY_PATTERN = re.compile(r"sk-orca-[A-Za-z0-9_\-]{4,}")

SOURCE_API_KEY = "api_key"
SOURCE_PKCE = "pkce"

_CONFIG_HOME = Path(
    os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config"
)
DEFAULT_STORE_PATH = _CONFIG_HOME / "v-rep-youbot-demo" / "orcarouter.json"


def mask_secret(secret: str | None) -> str:
    """Return a displayable form of a secret that never reveals its body."""
    if not secret:
        return ""
    tail = secret[-4:] if len(secret) > 12 else ""
    return f"sk-orca-…{tail}" if secret.startswith("sk-orca-") else "…" + tail


def redact(text: str) -> str:
    """Strip anything shaped like an OrcaRouter key out of ``text``."""
    return KEY_PATTERN.sub("sk-orca-…redacted", str(text))


def looks_like_key(value: str) -> bool:
    """Lightweight format check for obvious input mistakes.

    An ``sk-orca-`` prefix is *not* proof that a credential is valid; only a
    real request establishes that.
    """
    return bool(value) and value.startswith("sk-orca-") and len(value) > 20


@dataclass(frozen=True)
class CredentialResult:
    """A usable OrcaRouter credential plus the provenance of *how* we got it."""

    api_key: str
    source: str
    account_id: str | None = None
    scope: str | None = None
    #: Monotonic per-account credential generation.  A rejected request only
    #: ever invalidates the exact generation that made it.
    generation: int = 1

    def __post_init__(self) -> None:
        if not self.api_key:
            raise CredentialError("empty OrcaRouter credential")

    @property
    def masked(self) -> str:
        return mask_secret(self.api_key)

    def to_public_dict(self) -> dict[str, Any]:
        """Serialisable form that is safe for logs, UI and telemetry."""
        return {
            "source": self.source,
            "masked_key": self.masked,
            "account_id": self.account_id,
            "scope": self.scope,
            "generation": self.generation,
        }

    def __repr__(self) -> str:  # pragma: no cover - defensive
        return (
            f"CredentialResult(source={self.source!r}, key={self.masked!r}, "
            f"account_id={self.account_id!r}, generation={self.generation})"
        )


class CredentialSource(ABC):
    """One way of acquiring and keeping an OrcaRouter credential.

    Implementations are the adapters on the credential seam; the provider and
    the catalog only ever see the :class:`CredentialResult` they return.
    """

    source_id: str = ""
    label: str = ""

    @abstractmethod
    def obtain(self) -> CredentialResult:
        """Interactively acquire a credential (prompt, browser, ...)."""

    def load(self) -> CredentialResult | None:
        """Return a previously stored credential for this source, if any."""
        return None

    def clear(self) -> None:
        """Remove any credential this source stored."""


class CredentialStore:
    """Small JSON credential file with owner-only permissions.

    This is intentionally the *only* store the integration writes to.  No new
    keychain, no plaintext side files, no secrets in URLs or logs.
    """

    def __init__(self, path: Path | str | None = None) -> None:
        self.path = Path(path) if path is not None else DEFAULT_STORE_PATH

    # -- raw access ---------------------------------------------------------

    def _read(self) -> dict:
        try:
            raw = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return {}
        except OSError as exc:  # unreadable file: behave as "no credential"
            raise CredentialError(f"cannot read credential store: {exc}") from exc
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise CredentialError(
                "credential store is corrupt; clear it and reconnect"
            ) from exc
        return data if isinstance(data, dict) else {}

    def _write(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(self.path.parent), prefix=".orca-")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2, sort_keys=True)
            os.chmod(tmp, stat.S_IRUSR | stat.S_IWUSR)
            os.replace(tmp, self.path)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # -- credential API -----------------------------------------------------

    def save(self, result: CredentialResult) -> CredentialResult:
        """Persist ``result``, bumping its generation.

        The previous credential is *replaced atomically* only after the new one
        is durable, so a transient failure never destroys a working key.
        """
        data = self._read()
        previous = data.get(result.source) or {}
        try:
            prev_gen = int(previous.get("generation", 0))
        except (TypeError, ValueError):
            prev_gen = 0

        stored = asdict(result)
        stored["generation"] = max(result.generation, prev_gen + 1)
        # Never keep a refresh token: there is no refresh grant to use it with.
        stored.pop("refresh_token", None)
        data[result.source] = stored
        self._write(data)
        return CredentialResult(**stored)

    def load(self, source_id: str | None = None) -> CredentialResult | None:
        """Load a stored credential, optionally restricted to one source."""
        data = self._read()
        if source_id is not None:
            entry = data.get(source_id)
            return self._from_entry(entry)
        # Prefer the pasted API key, then any PKCE-issued key.
        for sid in (SOURCE_API_KEY, SOURCE_PKCE):
            entry = data.get(sid)
            found = self._from_entry(entry)
            if found is not None:
                return found
        return None

    def clear(self, source_id: str | None = None) -> None:
        if source_id is None:
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass
            return
        data = self._read()
        if data.pop(source_id, None) is not None:
            self._write(data)

    def set_needs_reauth(self, source_id: str, generation: int, value: bool) -> None:
        data = self._read()
        entry = data.get(source_id)
        if not isinstance(entry, dict):
            return
        # Generation-safe: a late failure from an old request must never mark a
        # freshly reauthorized credential as broken.
        if int(entry.get("generation", 0)) != generation:
            return
        entry["needs_reauth"] = bool(value)
        self._write(data)

    def needs_reauth(self, source_id: str) -> bool:
        entry = self._read().get(source_id)
        return bool(isinstance(entry, dict) and entry.get("needs_reauth"))

    @staticmethod
    def _from_entry(entry: object) -> CredentialResult | None:
        if not isinstance(entry, dict) or not entry.get("api_key"):
            return None
        return CredentialResult(
            api_key=entry["api_key"],
            source=entry.get("source", SOURCE_API_KEY),
            account_id=entry.get("account_id"),
            scope=entry.get("scope"),
            generation=int(entry.get("generation", 1)),
        )


def default_sources(store: CredentialStore | None = None) -> list[CredentialSource]:
    """The two user-visible authentication choices, in display order."""
    # Imported here to keep the credential seam free of import cycles.
    from .adapters.api_key import ApiKeyCredentialSource
    from .adapters.pkce import PkceCredentialSource

    store = store or CredentialStore()
    return [ApiKeyCredentialSource(store), PkceCredentialSource(store)]


def resolve_credential(
    store: CredentialStore | None = None, env: dict | None = None
) -> CredentialResult:
    """Return a credential, preferring the environment then the store.

    ``ORCAROUTER_API_KEY`` is the project's existing secret mechanism, so it
    wins over anything on disk; that keeps CI and container usage working with
    no interactive step.
    """
    store = store or CredentialStore()
    env = os.environ if env is None else env
    from_env = (env.get(KEY_ENV) or "").strip()
    if from_env:
        return CredentialResult(api_key=from_env, source=SOURCE_API_KEY)
    found = store.load()
    if found is None:
        raise CredentialError(
            "no OrcaRouter credential: set ORCAROUTER_API_KEY or connect an "
            "account with the 'OrcaRouter - Auth' option"
        )
    return found
