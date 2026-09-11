"""The credential seam: both adapters produce the same result shape, the store
redacts, and nothing secret leaks into logs or errors.
"""

from __future__ import annotations

import json
import stat

import pytest

from orcarouter.credentials import (
    SOURCE_API_KEY,
    SOURCE_PKCE,
    CredentialError,
    CredentialResult,
    CredentialStore,
    looks_like_key,
    mask_secret,
    redact,
    resolve_credential,
)
from tests.helpers import FAKE_KEY, FAKE_KEY_2


@pytest.fixture()
def store(tmp_path):
    return CredentialStore(tmp_path / "orcarouter.json")


def test_api_key_roundtrip_save_load_clear(store):
    result = CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY, scope="api")
    saved = store.save(result)
    assert saved.generation == 1

    loaded = store.load(SOURCE_API_KEY)
    assert loaded is not None
    assert loaded.api_key == FAKE_KEY
    assert loaded.source == SOURCE_API_KEY

    store.clear(SOURCE_API_KEY)
    assert store.load(SOURCE_API_KEY) is None


def test_store_file_is_owner_only(store):
    store.save(CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY))
    mode = stat.S_IMODE(store.path.stat().st_mode)
    assert mode == 0o600


def test_store_never_keeps_a_refresh_token(store):
    """A durable OrcaRouter key is not a refresh token; no refresh is stored."""
    store.save(CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE))
    raw = json.loads(store.path.read_text())
    assert "refresh_token" not in raw[SOURCE_PKCE]


def test_generation_increments_per_save(store):
    first = store.save(CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE))
    second = store.save(CredentialResult(api_key=FAKE_KEY_2, source=SOURCE_PKCE))
    assert second.generation == first.generation + 1


def test_redaction_hides_the_key_body():
    assert FAKE_KEY not in redact(f"failed with {FAKE_KEY} in the header")
    assert "sk-orca-…redacted" in redact(f"a {FAKE_KEY} b")


def test_mask_never_reveals_the_body():
    masked = mask_secret(FAKE_KEY)
    assert masked != FAKE_KEY
    assert FAKE_KEY[8:20] not in masked


def test_credential_result_public_dict_has_no_key():
    result = CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY, account_id="7")
    public = result.to_public_dict()
    assert "api_key" not in public
    assert FAKE_KEY not in json.dumps(public)
    assert public["masked_key"]


def test_repr_does_not_leak():
    result = CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY)
    assert FAKE_KEY not in repr(result)


def test_looks_like_key_is_only_a_typo_guard():
    assert looks_like_key(FAKE_KEY)
    assert not looks_like_key("not-a-key")
    assert not looks_like_key("sk-orca-")
    assert not looks_like_key("")


def test_corrupt_store_is_reported_not_crashed(store):
    store.path.parent.mkdir(parents=True, exist_ok=True)
    store.path.write_text("{not json")
    with pytest.raises(CredentialError) as excinfo:
        store.load()
    assert "corrupt" in str(excinfo.value)


def test_resolve_credential_prefers_the_environment(store):
    store.save(CredentialResult(api_key=FAKE_KEY_2, source=SOURCE_API_KEY))
    resolved = resolve_credential(store, {"ORCAROUTER_API_KEY": FAKE_KEY})
    assert resolved.api_key == FAKE_KEY


def test_resolve_credential_without_any_source_explains_both_paths(store):
    with pytest.raises(CredentialError) as excinfo:
        resolve_credential(store, {})
    message = str(excinfo.value)
    assert "ORCAROUTER_API_KEY" in message
    assert "Auth" in message


def test_needs_reauth_is_generation_safe(store):
    first = store.save(CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE))
    second = store.save(CredentialResult(api_key=FAKE_KEY_2, source=SOURCE_PKCE))
    assert second.generation != first.generation

    # A 401 belonging to the *old* generation arrives after a successful
    # re-login: it must not mark the freshly issued credential as broken.
    store.set_needs_reauth(SOURCE_PKCE, first.generation, True)
    assert store.needs_reauth(SOURCE_PKCE) is False

    # A 401 for the *current* generation does mark exactly that account.
    store.set_needs_reauth(SOURCE_PKCE, second.generation, True)
    assert store.needs_reauth(SOURCE_PKCE) is True
