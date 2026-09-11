"""OrcaRouter integration for the V-REP / CoppeliaSim YouBot demos.

OrcaRouter is an OpenAI-compatible AI gateway (https://www.orcarouter.ai).
This package adds it as a first-class provider with two explicit ways to
obtain a credential:

* ``OrcaRouter - API``  - paste an existing ``sk-orca-...`` API key.
* ``OrcaRouter - Auth`` - sign in with an OrcaRouter account using
  OAuth 2.0 + PKCE (RFC 7636) and receive a normal OrcaRouter API key.

Both paths produce the *same* credential result and feed the same inference
client and the same model catalog, so nothing downstream cares which one the
user picked.
"""

from .config import OriginConfig, resolve_origins
from .credentials import CredentialResult, CredentialSource, CredentialStore
from .adapters.api_key import ApiKeyCredentialSource
from .adapters.pkce import PkceCredentialSource
from .catalog import CatalogSnapshot, ModelInfo, fetch_catalog, filter_models
from .provider import OrcaRouterClient
from .registry import (
    ORCAROUTER_API_PROVIDER_ID,
    ORCAROUTER_AUTH_PROVIDER_ID,
    PROVIDERS,
    ProviderSpec,
    get_provider,
)
from .lifecycle import CredentialLifecycle, ReauthState

__all__ = [
    "OriginConfig",
    "resolve_origins",
    "CredentialResult",
    "CredentialSource",
    "CredentialStore",
    "ApiKeyCredentialSource",
    "PkceCredentialSource",
    "CatalogSnapshot",
    "ModelInfo",
    "fetch_catalog",
    "filter_models",
    "OrcaRouterClient",
    "ORCAROUTER_API_PROVIDER_ID",
    "ORCAROUTER_AUTH_PROVIDER_ID",
    "PROVIDERS",
    "ProviderSpec",
    "get_provider",
    "CredentialLifecycle",
    "ReauthState",
]

__version__ = "1.0.0"
