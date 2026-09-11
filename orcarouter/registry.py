"""Provider registry constants, mirrored for the GUI layer.

The single definition lives in :mod:`orcarouter.provider`; this module exists so
callers that only need the registry (and not the client) do not import the
transport stack.
"""

from __future__ import annotations

from .provider import (
    ORCAROUTER_API_PROVIDER_ID,
    ORCAROUTER_AUTH_PROVIDER_ID,
    PROVIDER_BY_ID,
    PROVIDERS,
    ProviderSpec,
    get_provider,
)

__all__ = [
    "ORCAROUTER_API_PROVIDER_ID",
    "ORCAROUTER_AUTH_PROVIDER_ID",
    "PROVIDER_BY_ID",
    "PROVIDERS",
    "ProviderSpec",
    "get_provider",
]
