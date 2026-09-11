"""Credential lifecycle: durable key reuse and terminal 401 recovery.

An OrcaRouter key obtained through PKCE is **durable but not refreshable**.
There is no refresh endpoint, so:

* the stored key is reused across restarts until OrcaRouter revokes it;
* a ``401`` from the relay is terminal -- it marks the exact account and
  credential generation that made the rejected request as ``needsReauth``;
* no proactive refresh is scheduled and no refresh grant is invented;
* the old secret is **not** deleted before a replacement succeeds, so a
  transient failure never causes irreversible account loss.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .credentials import (
    SOURCE_API_KEY,
    SOURCE_PKCE,
    CredentialResult,
    CredentialStore,
)
from .errors import ConfigurationError, TerminalReauthRequired


class ReauthState(str, Enum):
    READY = "ready"
    NEEDS_REAUTH = "needs_reauth"
    NOT_CONFIGURED = "not_configured"


@dataclass(frozen=True)
class AccountStatus:
    """Public status for one authentication choice.  No secret inside."""

    source_id: str
    state: ReauthState
    masked_key: str = ""
    account_id: str | None = None
    generation: int = 0
    detail: str = ""

    @property
    def usable(self) -> bool:
        return self.state is ReauthState.READY

    def to_public_dict(self) -> dict:
        return {
            "source_id": self.source_id,
            "state": self.state.value,
            "masked_key": self.masked_key,
            "account_id": self.account_id,
            "generation": self.generation,
            "usable": self.usable,
            "detail": self.detail,
        }


class CredentialLifecycle:
    """Single place that decides whether a stored credential may be used."""

    def __init__(self, store: CredentialStore | None = None) -> None:
        self.store = store or CredentialStore()

    # -- status -------------------------------------------------------------

    def status(self, source_id: str) -> AccountStatus:
        credential = self.store.load(source_id)
        if credential is None:
            env_credential = self._from_env(source_id)
            if env_credential is not None:
                return AccountStatus(
                    source_id=source_id,
                    state=ReauthState.READY,
                    masked_key=env_credential.masked,
                    generation=env_credential.generation,
                    detail="credential supplied by the environment",
                )
            return AccountStatus(
                source_id=source_id,
                state=ReauthState.NOT_CONFIGURED,
                detail="no credential stored for this option",
            )
        if self.store.needs_reauth(source_id):
            return AccountStatus(
                source_id=source_id,
                state=ReauthState.NEEDS_REAUTH,
                masked_key=credential.masked,
                account_id=credential.account_id,
                generation=credential.generation,
                detail=(
                    "OrcaRouter rejected this credential (401). A durable key "
                    "cannot be refreshed -- connect the account again."
                ),
            )
        return AccountStatus(
            source_id=source_id,
            state=ReauthState.READY,
            masked_key=credential.masked,
            account_id=credential.account_id,
            generation=credential.generation,
        )

    def all_status(self) -> tuple[AccountStatus, ...]:
        return (self.status(SOURCE_API_KEY), self.status(SOURCE_PKCE))

    # -- transitions --------------------------------------------------------

    def credential_for(self, source_id: str) -> CredentialResult:
        """Return a usable credential or explain why there is none."""
        status = self.status(source_id)
        if status.state is ReauthState.NEEDS_REAUTH:
            raise TerminalReauthRequired(
                account_id=status.account_id, generation=status.generation
            )
        if status.state is ReauthState.NOT_CONFIGURED:
            raise ConfigurationError(
                f"no OrcaRouter credential for {source_id!r}; connect first"
            )
        credential = self.store.load(source_id) or self._from_env(source_id)
        assert credential is not None
        return credential

    def mark_rejected(self, rejected: CredentialResult) -> None:
        """Mark the exact account/generation that made the rejected request.

        Generation-safe: a late failure from an older request is ignored, so it
        can never poison a freshly reauthorized credential.
        """
        self.store.set_needs_reauth(rejected.source, rejected.generation, True)

    def clear_reauth(self, source_id: str) -> None:
        """Called after a successful re-login for this exact source."""
        status = self.status(source_id)
        if status.generation:
            self.store.set_needs_reauth(source_id, status.generation, False)

    def refresh_after_login(self, source_id: str) -> AccountStatus:
        """Clear the reauth flag for the *new* generation after a login."""
        credential = self.store.load(source_id)
        if credential is not None:
            self.store.set_needs_reauth(source_id, credential.generation, False)
        return self.status(source_id)

    # -- internals ----------------------------------------------------------

    @staticmethod
    def _from_env(source_id: str) -> CredentialResult | None:
        from .credentials import resolve_credential

        if source_id != SOURCE_API_KEY:
            return None
        try:
            return resolve_credential()
        except Exception:
            return None
