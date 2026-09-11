"""Authentication adapters (one per OrcaRouter entry in the provider registry)."""

from .api_key import ApiKeyCredentialSource
from .pkce import PkceCredentialSource

__all__ = ["ApiKeyCredentialSource", "PkceCredentialSource"]
