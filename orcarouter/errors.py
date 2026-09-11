"""Typed errors for the OrcaRouter provider.

Every error message is safe to show to a user: credentials, codes and the
PKCE verifier are never interpolated into messages.
"""

from __future__ import annotations


class OrcaRouterError(Exception):
    """Base class for every OrcaRouter failure."""


class ConfigurationError(OrcaRouterError):
    """The provider is not configured (missing key, bad origin, ...)."""


# --- credential acquisition -------------------------------------------------


class CredentialError(OrcaRouterError):
    """Something went wrong while obtaining or storing a credential."""


class AccessDenied(CredentialError):
    """The user declined the authorization request."""


class StateMismatch(CredentialError):
    """The ``state`` echoed back did not match the one we sent."""


class CodeRejected(CredentialError):
    """The auth code was unknown, expired, or already used (HTTP 403)."""


class ChallengeRejected(CredentialError):
    """The server rejected our challenge/method, e.g. a downgrade (HTTP 400)."""


class ScopeDowngrade(CredentialError):
    """The granted scope is narrower than what this client needs."""

    def __init__(self, granted: str | None, required: str) -> None:
        self.granted = granted
        self.required = required
        super().__init__(
            f"OrcaRouter granted scope {granted!r} but {required!r} is required; "
            "ask a workspace owner to widen your role, then connect again"
        )


class AuthorizationTimeout(CredentialError):
    """The authorization window closed before a code arrived."""


class AuthorizationCancelled(CredentialError):
    """The user (or the host application) cancelled the authorization."""


class RateLimited(CredentialError):
    """HTTP 429 -- e.g. more than 10 PKCE keys issued per user per 24h."""


class AuthorizationPending(CredentialError):
    """Device grant: keep polling."""


class SlowDown(CredentialError):
    """Device grant: increase the poll interval."""


# --- transport / inference --------------------------------------------------


class NetworkError(OrcaRouterError):
    """The request never produced an HTTP response."""


class ApiError(OrcaRouterError):
    """The API answered with a non-success status."""

    def __init__(self, status: int, message: str) -> None:
        self.status = status
        self.message = message
        super().__init__(f"OrcaRouter API error {status}: {message}")


class TerminalReauthRequired(OrcaRouterError):
    """The stored credential was rejected with 401.

    This is terminal: a durable OrcaRouter key has no refresh grant, so the
    owning account must be marked ``needsReauth`` and sent through the connect
    flow again.  Retrying the same key forever is never correct.
    """

    def __init__(self, account_id: str | None, generation: int) -> None:
        self.account_id = account_id
        self.generation = generation
        super().__init__(
            "OrcaRouter rejected the stored credential (401); "
            "reconnect this account to issue a new key"
        )


# --- model catalog ----------------------------------------------------------


class CatalogError(OrcaRouterError):
    """The model catalog could not be fetched or parsed."""
