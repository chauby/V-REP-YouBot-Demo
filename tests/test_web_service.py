"""The settings-UI service: server-side credentials, the GUI login lock, and
the ``pagehide`` cancellation contract.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path

import pytest

from orcarouter.catalog import CAPABILITY_CHAT, SEED_MODELS
from orcarouter.config import OriginConfig
from orcarouter.credentials import SOURCE_API_KEY, SOURCE_PKCE, CredentialStore
from orcarouter.errors import CredentialError, TerminalReauthRequired
from orcarouter.lifecycle import ReauthState
from orcarouter.provider import (
    ORCAROUTER_API_PROVIDER_ID,
    ORCAROUTER_AUTH_PROVIDER_ID,
    PROVIDERS,
)
from orcarouter.web.server import OrcaRouterUIServer
from orcarouter.web.service import OrcaRouterService
from tests.helpers import FAKE_KEY, FakeOrcaServer, FakeTransport, chat_model, json_response, models_payload


@pytest.fixture()
def fake_server():
    server = FakeOrcaServer()
    server.start()
    try:
        yield server
    finally:
        server.stop()


def make_service(tmp_path, origins=None, transport=None, server=None):
    if origins is None and server is not None:
        origins = resolve_local(server)
    return OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=origins,
        transport=transport,
        open_browser=lambda _u: None,
    )


def resolve_local(server: FakeOrcaServer) -> OriginConfig:
    return OriginConfig(auth_base=server.base_url, api_base=server.base_url + "/v1")


# --- API key path -----------------------------------------------------------

def test_save_api_key_stores_and_reports_only_a_mask(tmp_path):
    service = OrcaRouterService(store=CredentialStore(tmp_path / "creds.json"))
    response = service.save_api_key(FAKE_KEY)
    assert response["ok"] is True
    assert FAKE_KEY not in json.dumps(response)
    assert response["status"]["state"] == "ready"
    assert response["status"]["masked_key"].startswith("sk-orca-")


def test_save_api_key_rejects_an_obviously_wrong_value(tmp_path):
    service = OrcaRouterService(store=CredentialStore(tmp_path / "creds.json"))
    with pytest.raises(CredentialError):
        service.save_api_key("hunter2")


def test_clear_removes_both_sources(tmp_path):
    service = OrcaRouterService(store=CredentialStore(tmp_path / "creds.json"))
    service.save_api_key(FAKE_KEY)
    service.clear_credential(SOURCE_API_KEY)
    assert service.status()["accounts"][0]["state"] == "not_configured"

    service.clear_credential(SOURCE_PKCE)
    assert service.status()["accounts"][1]["state"] == "not_configured"


def test_unknown_source_is_refused(tmp_path):
    service = OrcaRouterService(store=CredentialStore(tmp_path / "creds.json"))
    with pytest.raises(CredentialError):
        service.clear_credential("gmail")


# --- catalog ----------------------------------------------------------------

def test_models_endpoint_returns_the_live_catalog_without_a_key(tmp_path):
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response(models_payload([chat_model("a/b")])),
    })
    service = make_service(tmp_path, origins=OriginConfig(
        auth_base="https://www.orcarouter.ai", api_base="https://api.orcarouter.ai/v1"),
        transport=transport)
    payload = json.loads(service.models(CAPABILITY_CHAT))
    assert payload["source"] == "live"
    assert payload["models"][0]["id"] == "a/b"
    # No credential material in what the browser receives.
    assert "api_key" not in json.dumps(payload)


def test_models_endpoint_falls_back_to_the_seed_and_says_so(tmp_path):
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response({"error": "boom"}, 500),
    })
    service = make_service(tmp_path, origins=OriginConfig(
        auth_base="https://www.orcarouter.ai", api_base="https://api.orcarouter.ai/v1"),
        transport=transport)
    payload = json.loads(service.models(CAPABILITY_CHAT))
    assert payload["degraded"] is True
    assert payload["source"] == "seed"
    assert any(m["id"] == "openai/gpt-5.5" for m in payload["models"])


def test_models_endpoint_refuses_to_degrade_on_a_revoked_key(tmp_path):
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response({"error": "revoked"}, 401),
    })
    service = make_service(tmp_path, origins=OriginConfig(
        auth_base="https://www.orcarouter.ai", api_base="https://api.orcarouter.ai/v1"),
        transport=transport)
    with pytest.raises(TerminalReauthRequired):
        service.models(CAPABILITY_CHAT)


# --- GUI login lock ---------------------------------------------------------

def test_connect_start_returns_a_url_and_a_generation(tmp_path, fake_server):
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    response = service.connect_start(flow="B")
    assert response["ok"] is True
    assert response["generation"] >= 1
    assert response["authorize_url"].startswith(fake_server.base_url)


def test_second_connect_is_reported_busy_rather_than_starting_twice(tmp_path, fake_server):
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    service.connect_start(flow="B")
    second = service.connect_start(flow="B")
    assert second["ok"] is False
    assert second["busy"] is True
    service.connect_cancel()


def test_connect_complete_persists_and_clears_the_lock(tmp_path, fake_server):
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    started = service.connect_start(flow="B")
    done = service.connect_complete(started["generation"], "the-code")
    assert done["ok"] is True
    assert done["state"] == "done"
    assert FAKE_KEY not in json.dumps(done)
    # The lock is released: a new login can start.
    again = service.connect_start(flow="B")
    assert again["ok"] is True
    service.connect_cancel()


def test_exchange_error_releases_the_lock_and_stores_nothing(tmp_path, fake_server):
    fake_server.exchange_status = 403
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    started = service.connect_start(flow="B")
    failed = service.connect_complete(started["generation"], "bad-code")
    assert failed["ok"] is False
    assert "already used" in failed["error"] or "expired" in failed["error"]
    assert service.status()["login_busy"] is False


def test_cancel_releases_the_lock_synchronously(tmp_path, fake_server):
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    service.connect_start(flow="A")
    assert service.status()["login_busy"] is True
    result = service.connect_cancel()
    assert result["busy"] is False
    assert service.status()["login_busy"] is False
    # ...and a second login can start without remounting anything.
    second = service.connect_start(flow="A")
    assert second["ok"] is True
    service.connect_cancel()


def test_pagehide_style_cancel_is_idempotent_and_allows_a_second_login(tmp_path, fake_server):
    """The back-forward-cache shape, at the service layer.

    A page that is frozen and restored does not remount; the busy flag must
    already be clear and a fresh login must be startable.
    """
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    started = service.connect_start(flow="A")
    generation = started["generation"]

    # pagehide -> synchronous cancellation with the generation that was rendered.
    payload = service.connect_cancel(generation)
    assert payload["busy"] is False
    assert service.status()["login_busy"] is False

    # A late poll for the invalidated generation must not resurrect the session.
    late = service.connect_poll(generation)
    assert late["ok"] is False
    assert late["state"] == "superseded"
    assert service.status()["login_busy"] is False

    # No remount: start again immediately.
    second = service.connect_start(flow="A")
    assert second["ok"] is True
    assert second["generation"] != generation
    service.connect_cancel()


def test_repeated_pagehide_is_safe(tmp_path, fake_server):
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    service.connect_start(flow="A")
    for _ in range(3):
        service.connect_cancel()
        assert service.status()["login_busy"] is False


def test_provider_switch_cancels_an_in_flight_login(tmp_path, fake_server):
    """Focusing the API-key field cancels a pending PKCE login."""
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    service.connect_start(flow="A")
    assert service.status()["login_busy"] is True
    service.connect_cancel()
    # The API-key path then works normally.
    service.save_api_key(FAKE_KEY)
    assert service.status()["accounts"][0]["state"] == "ready"


# --- terminal 401 through the service ---------------------------------------

def test_chat_marks_the_exact_generation_on_401(tmp_path):
    transport = FakeTransport(responses={
        ("POST", "/v1/chat/completions"): json_response({"error": "revoked"}, 401),
    })
    service = make_service(
        tmp_path,
        origins=OriginConfig(auth_base="https://www.orcarouter.ai",
                             api_base="https://api.orcarouter.ai/v1"),
        transport=transport,
    )
    service.save_api_key(FAKE_KEY)
    with pytest.raises(TerminalReauthRequired):
        service.chat("openai/gpt-5.5", [{"role": "user", "content": "hi"}])
    assert service.lifecycle.status(SOURCE_API_KEY).state is ReauthState.NEEDS_REAUTH
    # The other choice is untouched.
    assert service.lifecycle.status(SOURCE_PKCE).state is ReauthState.NOT_CONFIGURED


def test_chat_without_any_credential_explains_both_options(tmp_path):
    service = make_service(tmp_path, origins=OriginConfig(
        auth_base="https://www.orcarouter.ai", api_base="https://api.orcarouter.ai/v1"))
    with pytest.raises(CredentialError) as excinfo:
        service.chat("m", [])
    message = str(excinfo.value)
    assert "API key" in message or "api key" in message
    assert "connect" in message


# --- HTTP surface -----------------------------------------------------------

def test_http_surface_serves_the_ui_and_never_echoes_the_key(tmp_path, fake_server):
    import urllib.request

    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    server = OrcaRouterUIServer(service, host="127.0.0.1", port=0)
    thread = server.start_background()
    base = f"http://127.0.0.1:{server.port}"
    try:
        with urllib.request.urlopen(base + "/", timeout=10) as resp:
            html = resp.read().decode()
        assert "OrcaRouter - API" in html
        assert "Connect with OrcaRouter" in html
        assert 'type="password"' in html

        req = urllib.request.Request(
            base + "/api/credentials/api-key",
            data=json.dumps({"api_key": FAKE_KEY}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            body = resp.read().decode()
        assert FAKE_KEY not in body
        assert json.loads(body)["ok"] is True
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_http_models_endpoint_serves_the_catalog(tmp_path):
    import urllib.request

    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response(models_payload([chat_model("a/b")])),
    })
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=OriginConfig(auth_base="https://www.orcarouter.ai",
                             api_base="https://api.orcarouter.ai/v1"),
        transport=transport,
    )
    server = OrcaRouterUIServer(service, port=0)
    thread = server.start_background()
    try:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{server.port}/api/models?capability=chat", timeout=10
        ) as resp:
            payload = json.loads(resp.read().decode())
        assert payload["source"] == "live"
        assert payload["models"][0]["id"] == "a/b"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


# --- the selector is a listbox, never free text ------------------------------

def test_model_selector_is_a_listbox_and_not_a_free_text_field():
    """The model control must be a picker bound to the live catalog."""
    static = Path(__file__).resolve().parents[1] / "orcarouter" / "web" / "static"
    html = (static / "index.html").read_text()
    script = (static / "app.js").read_text()

    assert 'role="combobox"' in html
    assert 'role="listbox"' in html
    assert 'aria-expanded="false"' in html
    # A hidden field mirrors the pick; it is not user-editable.
    assert 'type="hidden" id="orca-model-id"' in html
    assert "data-model-id" in script

    # There is no free-text model input anywhere.
    assert 'id="orca-model-input"' not in html
    for forbidden in ('id="orca-model-text"', 'name="orca-model-text"'):
        assert forbidden not in html


def test_selector_options_come_from_the_api_response(tmp_path):
    """The rendered options are exactly the filtered API result."""
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response(models_payload([
            chat_model("openai/gpt-5.5", ("text", "image")),
            chat_model("text/only", ("text",)),
            chat_model("image/only", ("text", "image"), ("image-generation", "openai")),
        ])),
    })
    service = make_service(
        tmp_path,
        origins=OriginConfig(auth_base="https://www.orcarouter.ai",
                             api_base="https://api.orcarouter.ai/v1"),
        transport=transport,
    )
    text_ids = [m["id"] for m in json.loads(service.models("chat"))["models"]]
    assert text_ids == ["openai/gpt-5.5", "text/only"]

    multi = json.loads(service.models("chat", "image"))
    # Only the models that explicitly declare image input survive.
    assert [m["id"] for m in multi["models"]] == ["openai/gpt-5.5"]
    assert multi["source"] == "live"
    assert multi["degraded"] is False


def test_an_outage_never_degrades_into_free_text_or_unverified_examples(tmp_path):
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response({"error": "down"}, 503),
    })
    service = make_service(
        tmp_path,
        origins=OriginConfig(auth_base="https://www.orcarouter.ai",
                             api_base="https://api.orcarouter.ai/v1"),
        transport=transport,
    )
    payload = json.loads(service.models("chat"))
    assert payload["degraded"] is True
    assert payload["source"] == "seed"
    # Everything offered is a verified fallback entry, clearly labelled.
    verified = {m.id for m in SEED_MODELS}
    assert {m["id"] for m in payload["models"]} <= verified
    assert payload["detail"]


def test_restored_selection_is_revalidated_against_the_compatible_list(tmp_path):
    """A stale model id must not be silently kept."""
    static = Path(__file__).resolve().parents[1] / "orcarouter" / "web" / "static"
    script = (static / "app.js").read_text()
    # The compatibility re-check is present before the value is kept.
    assert "clearSelection()" in script
    assert "state.selectedModel" in script


# --- regression: the browser's id space must actually work -------------------

def test_clear_accepts_the_provider_ids_the_ui_sends(tmp_path):
    """The settings UI posts provider ids, not credential-source ids.

    A mismatch here meant the Clear buttons silently did nothing (HTTP 400)
    while the stored key stayed on disk.
    """
    service = OrcaRouterService(store=CredentialStore(tmp_path / "creds.json"))
    service.save_api_key(FAKE_KEY)
    assert service.status()["accounts"][0]["state"] == "ready"

    result = service.clear_credential(ORCAROUTER_API_PROVIDER_ID)
    assert result["ok"] is True
    assert service.status()["accounts"][0]["state"] == "not_configured"
    assert service.lifecycle.store.load(SOURCE_API_KEY) is None


def test_clear_accepts_the_pkce_provider_id(tmp_path):
    service = OrcaRouterService(store=CredentialStore(tmp_path / "creds.json"))
    from orcarouter.credentials import CredentialResult

    service.store.save(CredentialResult(api_key=FAKE_KEY, source=SOURCE_PKCE))
    assert service.status()["accounts"][1]["state"] == "ready"

    service.clear_credential(ORCAROUTER_AUTH_PROVIDER_ID)
    assert service.status()["accounts"][1]["state"] == "not_configured"


def test_clear_still_accepts_a_raw_credential_source_id(tmp_path):
    service = OrcaRouterService(store=CredentialStore(tmp_path / "creds.json"))
    service.save_api_key(FAKE_KEY)
    service.clear_credential(SOURCE_API_KEY)
    assert service.status()["accounts"][0]["state"] == "not_configured"


def test_every_provider_id_resolves_to_its_credential_source():
    """The registry is the single link between the two id spaces."""
    for spec in PROVIDERS:
        assert OrcaRouterService.resolve_source_id(spec.id) == spec.credential_source
    assert OrcaRouterService.resolve_source_id(SOURCE_API_KEY) == SOURCE_API_KEY
    with pytest.raises(CredentialError):
        OrcaRouterService.resolve_source_id("nope")


def test_the_ui_sends_ids_the_service_accepts(tmp_path):
    """Guards the exact bug: read the ids straight out of app.js."""
    script = (Path(__file__).resolve().parents[1] / "orcarouter" / "web"
              / "static" / "app.js").read_text()
    import re

    sent = set(re.findall(r'source_id:\s*"([^"]+)"', script))
    assert sent, "no clear_credential calls found in the UI"
    for identifier in sent:
        assert OrcaRouterService.resolve_source_id(identifier) in (
            SOURCE_API_KEY, SOURCE_PKCE,
        )


# --- regression: the login lock cannot stick --------------------------------

def test_an_abandoned_login_releases_the_lock_on_timeout(tmp_path, fake_server):
    """No client call ever arrives when the browser dies mid-login.

    The worker's own terminal path must release the server-side lock, or the
    UI stays permanently busy and even a reload cannot start a new login.
    """
    import time

    from orcarouter.adapters.pkce import PkceCredentialSource
    from orcarouter.web.service import OrcaRouterService

    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    # Shrink the worker's authorization window so the test does not wait out
    # the real TTL; everything else is the production path.
    service.pkce = PkceCredentialSource(
        service.store, origins=resolve_local(fake_server), flow="A",
        open_browser=lambda _u: None,
    )
    service.pkce.on_terminal(service._release)
    import orcarouter.adapters.pkce as pkce_module

    original = pkce_module.AUTHORIZATION_TTL_SECONDS
    # Far shorter than the real 10 minute TTL.  Note the worker's terminal
    # notification can land while connect_start still holds the service lock,
    # so the transient "busy" state is not asserted here: the invariant under
    # test is that the lock does not *stay* held.
    pkce_module.AUTHORIZATION_TTL_SECONDS = 3.0
    try:
        started = service.connect_start(flow="A")
        assert started["generation"]

        deadline = time.monotonic() + 30
        while service.status()["login_busy"] and time.monotonic() < deadline:
            time.sleep(0.1)
        assert service.status()["login_busy"] is False, "login lock stuck"

        # The user can start a new login without restarting anything.
        assert service.connect_start(flow="A")["ok"] is True
        service.connect_cancel()
    finally:
        pkce_module.AUTHORIZATION_TTL_SECONDS = original


def test_cancelling_an_old_generation_does_not_release_a_newer_login(tmp_path, fake_server):
    """A delayed pagehide beacon carries the attempt it owns."""
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    first = service.connect_start(flow="B")["generation"]
    service.connect_cancel(first)
    second = service.connect_start(flow="B")["generation"]
    assert second != first

    # The stale beacon for the first attempt must not clear the second.
    result = service.connect_cancel(first)
    assert result["busy"] is True
    assert service.status()["login_busy"] is True

    # The current attempt still completes.
    done = service.connect_complete(second, "the-code")
    assert done["ok"] is True
    assert service.status()["login_busy"] is False


def test_a_cancelled_attempt_cannot_store_a_late_credential(tmp_path, fake_server):
    service = OrcaRouterService(
        store=CredentialStore(tmp_path / "creds.json"),
        origins=resolve_local(fake_server),
        open_browser=lambda _u: None,
    )
    generation = service.connect_start(flow="B")["generation"]
    service.connect_cancel(generation)

    late = service.connect_complete(generation, "stale-code")
    assert late["ok"] is False
    assert late["state"] == "superseded"
    assert service.store.load(SOURCE_PKCE) is None
