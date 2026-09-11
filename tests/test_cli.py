"""CLI entry points: both authentication choices stay discoverable and the
clear command removes the credential it names.
"""

from __future__ import annotations

import json

import pytest

from orcarouter import cli
from orcarouter.credentials import (
    SOURCE_API_KEY,
    SOURCE_PKCE,
    CredentialResult,
    CredentialStore,
)
from orcarouter.provider import (
    ORCAROUTER_API_PROVIDER_ID,
    ORCAROUTER_AUTH_PROVIDER_ID,
)
from tests.helpers import FAKE_KEY, FAKE_KEY_2


@pytest.fixture()
def store_path(tmp_path):
    return str(tmp_path / "orcarouter.json")


def run(argv, capsys):
    code = cli.main(argv)
    captured = capsys.readouterr()
    return code, captured.out, captured.err


# --- discoverability --------------------------------------------------------

def test_both_authentication_choices_are_discoverable_in_help(capsys):
    with pytest.raises(SystemExit):
        cli.main(["--help"])
    out = capsys.readouterr().out
    assert "set-key" in out
    assert "connect" in out
    assert "OAuth 2.0 + PKCE" in out


def test_connect_exposes_both_flows(capsys):
    with pytest.raises(SystemExit):
        cli.main(["connect", "--help"])
    out = capsys.readouterr().out
    assert "loopback" in out
    assert "out-of-band" in out


# --- status -----------------------------------------------------------------

def test_status_lists_both_choices(store_path, capsys):
    code, out, _ = run(["--store", store_path, "status"], capsys)
    assert code == 0
    assert ORCAROUTER_API_PROVIDER_ID in out
    assert ORCAROUTER_AUTH_PROVIDER_ID in out
    assert "https://api.orcarouter.ai/v1" in out
    assert "https://www.orcarouter.ai" in out


def test_status_shows_only_a_mask(store_path, capsys):
    CredentialStore(store_path).save(
        CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY)
    )
    code, out, _ = run(["--store", store_path, "status"], capsys)
    assert code == 0
    assert FAKE_KEY not in out
    assert "sk-orca-" in out


# --- set-key / clear --------------------------------------------------------

def test_set_key_then_clear_by_provider_id(store_path, capsys):
    store = CredentialStore(store_path)
    code, out, _ = run(
        ["--store", store_path, "set-key", "--key", FAKE_KEY], capsys
    )
    assert code == 0
    assert FAKE_KEY not in out
    assert store.load(SOURCE_API_KEY).api_key == FAKE_KEY

    # The default --source is the provider id the UI also uses.
    code, out, _ = run(["--store", store_path, "clear"], capsys)
    assert code == 0
    assert store.load(SOURCE_API_KEY) is None


def test_clear_accepts_the_pkce_provider_id(store_path, capsys):
    store = CredentialStore(store_path)
    store.save(CredentialResult(api_key=FAKE_KEY_2, source=SOURCE_PKCE))
    code, _, _ = run(
        ["--store", store_path, "clear", "--source", ORCAROUTER_AUTH_PROVIDER_ID],
        capsys,
    )
    assert code == 0
    assert store.load(SOURCE_PKCE) is None


def test_clear_accepts_a_raw_credential_source_id(store_path, capsys):
    store = CredentialStore(store_path)
    store.save(CredentialResult(api_key=FAKE_KEY, source=SOURCE_API_KEY))
    code, _, _ = run(["--store", store_path, "clear", "--source", "api_key"], capsys)
    assert code == 0
    assert store.load(SOURCE_API_KEY) is None


def test_set_key_rejects_a_non_key_without_touching_the_store(store_path, capsys):
    code, _, err = run(["--store", store_path, "set-key", "--key", "nope"], capsys)
    assert code == 2
    assert "sk-orca" in err
    assert CredentialStore(store_path).load() is None


def test_chat_without_a_credential_explains_both_paths(store_path, capsys):
    code, _, err = run(
        ["--store", store_path, "chat", "--model", "m", "--prompt", "x"], capsys
    )
    assert code == 2
    assert "connect" in err and "set-key" in err


def test_unknown_source_is_rejected_by_argparse(store_path, capsys):
    with pytest.raises(SystemExit):
        cli.main(["--store", store_path, "clear", "--source", "gmail"])
    assert "invalid choice" in capsys.readouterr().err
