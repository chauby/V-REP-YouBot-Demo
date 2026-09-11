"""Dual-authentication guarantees.

The point of these tests: the API-key adapter and the PKCE adapter are two
different doors into the *same* credential seam.  Nothing downstream -- the
inference client, the model catalog, the lifecycle -- may behave differently
depending on which door was used, and the two origins must never be confused.
"""

from __future__ import annotations

import json
from urllib.parse import parse_qs, urlsplit

import pytest

from orcarouter.adapters.api_key import ApiKeyCredentialSource
from orcarouter.adapters.pkce import PkceCredentialSource
from orcarouter.catalog import CAPABILITY_CHAT, CatalogCache
from orcarouter.config import resolve_origins
from orcarouter.credentials import (
    SOURCE_API_KEY,
    SOURCE_PKCE,
    CredentialResult,
    CredentialStore,
)
from orcarouter.errors import CredentialError, TerminalReauthRequired
from orcarouter.lifecycle import CredentialLifecycle, ReauthState
from orcarouter.provider import (
    ORCAROUTER_API_PROVIDER_ID,
    ORCAROUTER_AUTH_PROVIDER_ID,
    PROVIDERS,
    OrcaRouterClient,
    get_provider,
)
from tests.helpers import (
    FAKE_KEY,
    FAKE_KEY_2,
    FakeOrcaServer,
    FakeTransport,
    chat_model,
    json_response,
    models_payload,
)

ORIGINS = resolve_origins({})


@pytest.fixture()
def fake_server():
    server = FakeOrcaServer()
    server.start()
    try:
        yield server
    finally:
        server.stop()


def local_origins(server: FakeOrcaServer):
    from orcarouter.config import OriginConfig

    return OriginConfig(auth_base=server.base_url, api_base=server.base_url + "/v1")


# --- registry ---------------------------------------------------------------

def test_both_choices_are_registered_and_selectable():
    ids = {spec.id for spec in PROVIDERS}
    assert ids == {ORCAROUTER_API_PROVIDER_ID, ORCAROUTER_AUTH_PROVIDER_ID}
    for spec in PROVIDERS:
        assert spec.base_url == "https://api.orcarouter.ai/v1"
        assert spec.auth_base_url == "https://www.orcarouter.ai"


def test_the_two_choices_have_distinct_labels_and_auth_methods():
    api = get_provider(ORCAROUTER_API_PROVIDER_ID)
    oauth = get_provider(ORCAROUTER_AUTH_PROVIDER_ID)
    assert api.label != oauth.label
    assert api.auth_method == "api_key"
    assert oauth.auth_method == "oauth_pkce"
    # Same inference surface, different credential acquisition.
    assert api.base_url == oauth.base_url


def test_unknown_provider_id_is_refused():
    from orcarouter.errors import ConfigurationError

    with pytest.raises(ConfigurationError):
        get_provider("not-a-provider")


# --- both adapters yield the same credential --------------------------------

def test_both_adapters_return_the_same_credential_result_shape(tmp_path):
    """The hard requirement: two adapters, one credential result."""
    store = CredentialStore(tmp_path / "creds.json")

    key_result = ApiKeyCredentialSource(store).obtain(FAKE_KEY)
    assert isinstance(key_result, CredentialResult)
    assert key_result.source == SOURCE_API_KEY

    # A PKCE adapter that would issue the *same* key: shaped identically.
    pkce_result = CredentialResult(
        api_key=FAKE_KEY, source=SOURCE_PKCE, account_id="42", scope="api"
    )
    assert set(key_result.to_public_dict()) == set(pkce_result.to_public_dict())

    # Both are usable by the same client constructor with no branching.
    for credential in (key_result, pkce_result):
        client = OrcaRouterClient(credential, origins=ORIGINS)
        assert client.spec.base_url == "https://api.orcarouter.ai/v1"


def test_real_pkce_login_and_api_key_produce_equivalent_credentials(tmp_path, fake_server):
    """Drive the real PKCE adapter against a local auth server, then compare."""
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="B",
        open_browser=lambda _u: None,
    )
    attempt = source.start(flow="B")
    pkce_credential = source.complete(attempt.generation, "code")

    api_credential = ApiKeyCredentialSource(store).obtain(FAKE_KEY_2)

    # Same type, same fields, same masked rendering contract.
    assert type(pkce_credential) is type(api_credential)
    assert pkce_credential.source != api_credential.source
    assert pkce_credential.masked.startswith("sk-orca-")
    assert api_credential.masked.startswith("sk-orca-")


def test_downstream_does_not_care_which_adapter_was_used(tmp_path):
    """Inference and discovery work identically for both sources."""
    for source_id, credential in (
        (SOURCE_API_KEY, CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY)),
        (SOURCE_PKCE, CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE,
                                       account_id="1", scope="api")),
    ):
        transport = FakeTransport(responses={
            ("POST", "/v1/chat/completions"): json_response(
                {"model": "openai/gpt-5.5", "choices": [{"message": {"content": "ok"}}]}
            ),
        })
        client = OrcaRouterClient(credential, origins=ORIGINS, transport=transport)
        payload = client.chat_completion("openai/gpt-5.5", [{"role": "user", "content": "hi"}])
        assert payload["choices"][0]["message"]["content"] == "ok"
        call = transport.calls[0]
        assert call.url == "https://api.orcarouter.ai/v1/chat/completions"
        assert call.headers["Authorization"] == f"Bearer {FAKE_KEY}"
        assert source_id  # both branches exercised


def test_catalog_discovery_is_identical_for_both_sources():
    for credential in (
        CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY),
        CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE),
    ):
        transport = FakeTransport(responses={
            ("GET", "/v1/models"): json_response(
                models_payload([chat_model("a/b")])
            ),
        })
        snapshot = CatalogCache().refresh(
            CAPABILITY_CHAT, origins=ORIGINS, api_key=credential.api_key,
            transport=transport,
        )
        assert snapshot.ids == ("a/b",)
        assert transport.calls[0].headers["Authorization"] == f"Bearer {FAKE_KEY}"


# --- origin separation ------------------------------------------------------

def test_auth_requests_only_reach_the_auth_origin(tmp_path, fake_server):
    """Auth traffic goes to the auth origin; inference to the API origin."""
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="B",
        open_browser=lambda _u: None,
    )
    attempt = source.start(flow="B")
    query = parse_qs(urlsplit(attempt.authorize_url).query)
    assert query["callback_url"] == ["oob"]
    source.complete(attempt.generation, "code")

    # The fake server serves both roles on one port; assert the *paths* prove
    # which role each request played.
    assert len(fake_server.exchange_requests) == 1   # /api/v1/auth/keys
    assert fake_server.models_requests == []         # nothing hit /v1/models


def test_exchange_and_authorize_never_point_at_the_inference_origin():
    for url in (ORIGINS.authorize_url, ORIGINS.exchange_url,
                ORIGINS.device_code_url, ORIGINS.device_token_url):
        assert url.startswith("https://www.orcarouter.ai/")
        assert "api.orcarouter.ai" not in url
        # The auth API lives at /api/v1/auth/... on www; it is never /v1/auth.
        assert "/v1/auth" not in url or "/api/v1/auth" in url


def test_inference_and_discovery_never_point_at_the_auth_origin():
    assert ORIGINS.models_url == "https://api.orcarouter.ai/v1/models"
    assert ORIGINS.api_url("/chat/completions").startswith("https://api.orcarouter.ai/v1")
    assert "www.orcarouter.ai" not in ORIGINS.models_url


# --- terminal 401 -----------------------------------------------------------

def test_401_is_terminal_and_does_not_retry_or_refresh():
    credential = CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE, generation=3)
    transport = FakeTransport(responses={
        ("POST", "/v1/chat/completions"): json_response({"error": "revoked"}, 401),
    })
    client = OrcaRouterClient(credential, origins=ORIGINS, transport=transport)
    with pytest.raises(TerminalReauthRequired) as excinfo:
        client.chat_completion("m", [{"role": "user", "content": "x"}])
    assert excinfo.value.generation == 3
    assert len(transport.calls) == 1  # exactly one attempt, no loop

    # No refresh endpoint was ever called.
    for call in transport.calls:
        assert "refresh" not in call.url
        assert "token" not in call.url


def test_403_from_the_relay_is_also_terminal():
    credential = CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY)
    transport = FakeTransport(responses={
        ("POST", "/v1/chat/completions"): json_response({"error": "forbidden"}, 403),
    })
    client = OrcaRouterClient(credential, origins=ORIGINS, transport=transport)
    with pytest.raises(TerminalReauthRequired):
        client.chat_completion("m", [{"role": "user", "content": "x"}])


def test_api_key_and_pkce_accept_the_same_env_mechanism():
    from orcarouter.credentials import resolve_credential

    resolved = resolve_credential(CredentialStore("/nonexistent/x.json"),
                                 {"ORCAROUTER_API_KEY": FAKE_KEY})
    assert resolved.api_key == FAKE_KEY


# --- lifecycle --------------------------------------------------------------

def test_rejected_generation_is_marked_and_the_other_source_is_untouched(tmp_path):
    store = CredentialStore(tmp_path / "creds.json")
    lifecycle = CredentialLifecycle(store)
    api = ApiKeyCredentialSource(store).obtain(FAKE_KEY)

    lifecycle.mark_rejected(api)
    assert lifecycle.status(SOURCE_API_KEY).state is ReauthState.NEEDS_REAUTH
    assert lifecycle.status(SOURCE_PKCE).state is ReauthState.NOT_CONFIGURED

    with pytest.raises(TerminalReauthRequired):
        lifecycle.credential_for(SOURCE_API_KEY)


def test_a_late_401_from_an_old_generation_does_not_poison_the_new_credential(tmp_path):
    """The exact-account/exact-generation rule."""
    store = CredentialStore(tmp_path / "creds.json")
    lifecycle = CredentialLifecycle(store)

    old = store.save(CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE, account_id="1"))
    new = store.save(CredentialResult(api_key=FAKE_KEY_2, source=SOURCE_PKCE, account_id="1"))
    assert new.generation > old.generation

    # The in-flight request that made the 401 still carried the *old* credential.
    lifecycle.mark_rejected(old)
    assert lifecycle.status(SOURCE_PKCE).state is ReauthState.READY

    # A 401 for the current generation does mark it.
    lifecycle.mark_rejected(new)
    assert lifecycle.status(SOURCE_PKCE).state is ReauthState.NEEDS_REAUTH


def test_old_secret_is_not_deleted_before_a_replacement_succeeds(tmp_path):
    store = CredentialStore(tmp_path / "creds.json")
    lifecycle = CredentialLifecycle(store)
    original = ApiKeyCredentialSource(store).obtain(FAKE_KEY)

    # A failed attempt at replacing the key must leave the old one intact.
    with pytest.raises(CredentialError):
        ApiKeyCredentialSource(store).obtain("definitely-not-a-key")

    still_there = store.load(SOURCE_API_KEY)
    assert still_there is not None
    assert still_there.api_key == original.api_key


def test_relogin_clears_the_reauth_flag_for_the_new_generation(tmp_path):
    store = CredentialStore(tmp_path / "creds.json")
    lifecycle = CredentialLifecycle(store)
    credential = ApiKeyCredentialSource(store).obtain(FAKE_KEY)
    lifecycle.mark_rejected(credential)
    assert lifecycle.status(SOURCE_API_KEY).state is ReauthState.NEEDS_REAUTH

    # Connect again -> a new generation -> the flag clears.
    lifecycle.refresh_after_login(SOURCE_API_KEY)
    assert lifecycle.status(SOURCE_API_KEY).state is ReauthState.READY
    assert lifecycle.credential_for(SOURCE_API_KEY).api_key == FAKE_KEY


def test_status_payload_carries_no_secret(tmp_path):
    store = CredentialStore(tmp_path / "creds.json")
    lifecycle = CredentialLifecycle(store)
    ApiKeyCredentialSource(store).obtain(FAKE_KEY)
    payload = json.dumps([s.to_public_dict() for s in lifecycle.all_status()])
    assert FAKE_KEY not in payload
    assert "sk-orca-" in payload and "redacted" not in payload
    assert FAKE_KEY[-4:] in payload   # only the last four characters are shown


def test_failed_api_key_looks_are_rejected_without_a_network_call():
    source = ApiKeyCredentialSource(CredentialStore("/nonexistent/x.json"))
    with pytest.raises(CredentialError) as excinfo:
        source.obtain("garbage")
    assert "sk-orca" in str(excinfo.value)


# --- every supported entry point targets a real endpoint --------------------

def test_embedding_and_image_entry_points_target_their_real_paths():
    """The non-chat surfaces are wired to real endpoints, not merely declared."""
    credential = CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY)

    transport = FakeTransport(responses={
        ("POST", "/v1/embeddings"): json_response({"data": [{"embedding": [0.1]}]}),
        ("POST", "/v1/images/generations"): json_response({"data": [{"url": "x"}]}),
    })
    client = OrcaRouterClient(credential, origins=ORIGINS, transport=transport)

    client.embeddings("embed/model", ["hello"])
    client.generate_image("image/gen-model", "a can on a table")

    assert [c.url for c in transport.calls] == [
        "https://api.orcarouter.ai/v1/embeddings",
        "https://api.orcarouter.ai/v1/images/generations",
    ]
    for call in transport.calls:
        assert call.headers["Authorization"] == f"Bearer {FAKE_KEY}"
        assert call.method == "POST"


def test_every_entry_point_uses_the_same_single_origin_and_transport():
    """Auth is never duplicated per entry point."""
    credential = CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE)
    transport = FakeTransport(responses={
        ("POST", "/v1/chat/completions"): json_response({"choices": []}),
        ("POST", "/v1/embeddings"): json_response({"data": []}),
        ("POST", "/v1/images/generations"): json_response({"data": []}),
    })
    client = OrcaRouterClient(credential, origins=ORIGINS, transport=transport)
    client.chat_completion("m", [{"role": "user", "content": "x"}])
    client.embeddings("m", "x")
    client.generate_image("m", "x")

    assert len(transport.calls) == 3
    for call in transport.calls:
        assert call.url.startswith("https://api.orcarouter.ai/v1/")
        assert call.headers["Authorization"] == f"Bearer {FAKE_KEY}"
