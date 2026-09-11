"""Origin resolution and network policy."""

from __future__ import annotations

import pytest

from orcarouter.config import (
    DEFAULT_API_BASE,
    DEFAULT_AUTH_BASE,
    OriginError,
    resolve_origins,
    validate_origin,
)


def test_defaults_use_distinct_public_origins():
    origins = resolve_origins({})
    assert origins.auth_base == "https://www.orcarouter.ai"
    assert origins.api_base == "https://api.orcarouter.ai/v1"


def test_auth_paths_are_on_the_auth_origin():
    origins = resolve_origins({})
    assert origins.authorize_url == "https://www.orcarouter.ai/auth"
    assert origins.exchange_url == "https://www.orcarouter.ai/api/v1/auth/keys"
    # The relay is at /v1 on api.orcarouter.ai; the auth endpoints are not.
    # https://api.orcarouter.ai/v1/auth/keys is a 404.
    assert origins.exchange_url == "https://www.orcarouter.ai/api/v1/auth/keys"


def test_inference_paths_are_on_the_api_origin():
    origins = resolve_origins({})
    assert origins.models_url == "https://api.orcarouter.ai/v1/models"
    assert origins.api_url("/chat/completions") == (
        "https://api.orcarouter.ai/v1/chat/completions"
    )


def test_explicit_overrides_win():
    origins = resolve_origins({
        "ORCA_BASE_URL": "https://shared.example",
        "ORCA_AUTH_BASE_URL": "https://auth.example",
        "ORCA_API_BASE_URL": "https://api.example/v1",
    })
    assert origins.auth_base == "https://auth.example"
    assert origins.api_base == "https://api.example/v1"


def test_shared_self_hosted_base_supplies_both():
    origins = resolve_origins({"ORCA_BASE_URL": "https://orca.example"})
    assert origins.auth_base == "https://orca.example"
    assert origins.api_base == "https://orca.example/v1"


def test_shared_base_with_path_is_used_as_the_inference_base():
    origins = resolve_origins({"ORCA_BASE_URL": "https://orca.example/gateway/v1"})
    assert origins.api_base == "https://orca.example/gateway/v1"


def test_api_override_still_wins_over_shared_base():
    origins = resolve_origins({
        "ORCA_BASE_URL": "https://shared.example",
        "ORCA_API_BASE_URL": "https://relay.example/v1",
    })
    assert origins.auth_base == "https://shared.example"
    assert origins.api_base == "https://relay.example/v1"


def test_one_origin_is_never_derived_from_the_other():
    """A shared base is the only derivation path, and it is the documented one."""
    origins = resolve_origins({})
    assert DEFAULT_AUTH_BASE.replace("www.", "") not in origins.api_base
    assert origins.api_base != origins.auth_base


@pytest.mark.parametrize("url", [
    "http://api.orcarouter.ai/v1",
    "http://example.com",
    "ftp://api.orcarouter.ai",
    "https://api.orcarouter.ai/v1?x=1",
    "https://api.orcarouter.ai/v1#frag",
])
def test_invalid_origins_are_rejected(url):
    with pytest.raises(OriginError):
        validate_origin(url, name="test")


@pytest.mark.parametrize("url", [
    "http://localhost:8080",
    "http://127.0.0.1:1234",
    "http://[::1]:9000",
    "https://anything.example",
])
def test_allowed_origins(url):
    assert validate_origin(url, name="test") == url.rstrip("/")


def test_loopback_over_http_is_rejected_when_env_is_resolved():
    with pytest.raises(OriginError):
        resolve_origins({"ORCA_API_BASE_URL": "http://evil.example/v1"})
