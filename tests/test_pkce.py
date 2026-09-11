"""PKCE protocol, the connect adapter, and the full authorize->exchange->persist
round trip against a local fake auth server.

Every credential in this file is a fake.  Nothing here touches the network.
"""

from __future__ import annotations

import base64
import hashlib
import json
from urllib.parse import parse_qs, urlsplit

import pytest

from orcarouter.adapters.pkce import PkceCredentialSource
from orcarouter.config import resolve_origins
from orcarouter.credentials import SOURCE_PKCE, CredentialStore
from orcarouter.errors import (
    AccessDenied,
    AuthorizationCancelled,
    AuthorizationTimeout,
    ChallengeRejected,
    CodeRejected,
    CredentialError,
    NetworkError,
    RateLimited,
    ScopeDowngrade,
    StateMismatch,
    TerminalReauthRequired,
)
from orcarouter.pkce import (
    b64url,
    build_authorize_url,
    challenge_for,
    check_state,
    exchange_code,
    generate_state,
    generate_verifier,
    new_attempt,
)
from orcarouter.config import AUTHORIZE_PATH
from orcarouter.transport import HttpResult
from tests.helpers import FAKE_KEY, FakeOrcaServer, FakeTransport, json_response

ORIGINS = resolve_origins({})


# --- verifier / challenge ---------------------------------------------------

def test_challenge_is_unpadded_base64url_sha256_of_the_verifier():
    verifier = "abc123"
    expected = base64.urlsafe_b64encode(
        hashlib.sha256(verifier.encode()).digest()
    ).decode().rstrip("=")
    assert challenge_for(verifier) == expected
    assert "=" not in challenge_for(verifier)


def test_verifier_and_state_are_fresh_per_attempt():
    a = new_attempt()
    b = new_attempt()
    assert a.verifier != b.verifier
    assert a.state != b.state
    assert a.challenge != b.challenge


def test_verifier_uses_a_cryptographic_rng_and_has_entropy():
    verifiers = {generate_verifier() for _ in range(200)}
    assert len(verifiers) == 200                      # no collisions
    assert all(len(v) == 43 for v in verifiers)       # 32 bytes -> 43 b64url chars
    assert len({generate_state() for _ in range(200)}) == 200
    assert generate_verifier() != generate_verifier()


def test_attempt_repr_does_not_expose_the_verifier_or_state():
    attempt = new_attempt(generation=3)
    text = repr(attempt)
    assert attempt.verifier not in text
    assert attempt.state not in text
    assert "3" in text


# --- authorize URL ----------------------------------------------------------

def test_authorize_url_uses_the_auth_origin_and_sends_only_the_challenge():
    attempt = new_attempt()
    url = build_authorize_url(
        ORIGINS, attempt, app_name="Test App", callback_url="oob"
    )
    parts = urlsplit(url)
    assert parts.scheme == "https"
    assert parts.netloc == "www.orcarouter.ai"
    assert parts.path == AUTHORIZE_PATH

    query = parse_qs(parts.query)
    assert query["code_challenge"] == [attempt.challenge]
    assert query["code_challenge_method"] == ["S256"]
    assert query["state"] == [attempt.state]
    assert query["app_name"] == ["Test App"]
    assert query["scope"] == ["api"]

    # The verifier must never appear anywhere in the URL.
    assert attempt.verifier not in url
    assert "code_verifier" not in query


def test_authorize_url_never_targets_the_inference_origin():
    url = build_authorize_url(ORIGINS, new_attempt(), app_name="X", callback_url="oob")
    assert "api.orcarouter.ai" not in url


def test_flow_b_uses_the_literal_oob_callback():
    from orcarouter.pkce import authorize_flow_b

    attempt = new_attempt()
    url = authorize_flow_b(
        ORIGINS, app_name="X", state=attempt.state, challenge=attempt.challenge,
        open_browser=lambda _u: None,
    )
    query = parse_qs(urlsplit(url).query)
    assert query["callback_url"] == ["oob"]
    assert query["code_challenge_method"] == ["S256"]
    assert attempt.verifier not in url


def test_flat_b64url_has_no_padding():
    raw = b"\x00\x01\x02"
    assert "=" not in b64url(raw)
    assert b64url(raw) == "AAEC"


# --- state comparison -------------------------------------------------------

def test_state_mismatch_is_refused():
    with pytest.raises(StateMismatch):
        check_state("wrong", "expected")
    with pytest.raises(StateMismatch):
        check_state(None, "expected")
    check_state("same", "same")  # no raise


# --- exchange ---------------------------------------------------------------

def test_exchange_posts_to_the_auth_origin_with_the_right_body():
    transport = FakeTransport(responses={
        ("POST", "/api/v1/auth/keys"): json_response(
            {"key": FAKE_KEY, "user_id": "9", "scope": "api"}
        ),
    })
    result = exchange_code(ORIGINS, "the-code", "the-verifier", transport=transport)
    assert result.key == FAKE_KEY
    assert result.user_id == "9"
    assert result.scope == "api"

    call = transport.calls[0]
    assert call.url == "https://www.orcarouter.ai/api/v1/auth/keys"
    assert call.method == "POST"
    assert call.json_body == {
        "code": "the-code",
        "code_verifier": "the-verifier",
        "code_challenge_method": "S256",
    }


def test_exchange_never_uses_the_404_relay_path():
    transport = FakeTransport(responses={
        ("POST", "/api/v1/auth/keys"): json_response({"key": FAKE_KEY, "scope": "api"}),
    })
    exchange_code(ORIGINS, "c", "v", transport=transport)
    for call in transport.calls:
        # Everything goes to the auth origin, never to the relay.
        assert call.url.startswith("https://www.orcarouter.ai/")
        assert "api.orcarouter.ai" not in call.url


@pytest.mark.parametrize("status,expected", [
    (400, ChallengeRejected),
    (403, CodeRejected),
    (429, RateLimited),
])
def test_exchange_error_statuses_map_to_typed_errors(status, expected):
    transport = FakeTransport(responses={
        ("POST", "/api/v1/auth/keys"): json_response({"error": "nope"}, status),
    })
    with pytest.raises(expected):
        exchange_code(ORIGINS, "c", "v", transport=transport)


def test_403_message_mentions_single_use_and_expiry():
    transport = FakeTransport(responses={
        ("POST", "/api/v1/auth/keys"): json_response({"error": "invalid_grant"}, 403),
    })
    with pytest.raises(CodeRejected) as excinfo:
        exchange_code(ORIGINS, "c", "v", transport=transport)
    message = str(excinfo.value)
    assert "already used" in message or "expired" in message


def test_429_explains_the_issuance_cap():
    transport = FakeTransport(responses={
        ("POST", "/api/v1/auth/keys"): json_response({"error": "rate_limited"}, 429),
    })
    with pytest.raises(RateLimited) as excinfo:
        exchange_code(ORIGINS, "c", "v", transport=transport)
    assert "24 hours" in str(excinfo.value)


def test_scope_downgrade_is_reported_not_ignored():
    transport = FakeTransport(responses={
        ("POST", "/api/v1/auth/keys"): json_response(
            {"key": FAKE_KEY, "user_id": "1", "scope": "connector"}
        ),
    })
    with pytest.raises(ScopeDowngrade) as excinfo:
        exchange_code(ORIGINS, "c", "v", transport=transport, required_scope="api")
    assert excinfo.value.granted == "connector"


def test_response_without_a_key_is_an_error():
    transport = FakeTransport(responses={
        ("POST", "/api/v1/auth/keys"): json_response({"scope": "api"}),
    })
    with pytest.raises(CredentialError):
        exchange_code(ORIGINS, "c", "v", transport=transport)


def test_network_failure_is_not_a_credential_leak():
    class Boom(FakeTransport):
        def request(self, *a, **kw):
            raise NetworkError("connection refused")

    with pytest.raises(NetworkError):
        exchange_code(ORIGINS, "c", "v", transport=Boom())


def test_error_messages_never_contain_the_verifier_or_code():
    secret_verifier = "verifier-DO-NOT-LEAK"
    transport = FakeTransport(responses={
        ("POST", "/api/v1/auth/keys"): json_response(
            {"error": f"bad {FAKE_KEY}"}, 403
        ),
    })
    with pytest.raises(CodeRejected) as excinfo:
        exchange_code(ORIGINS, "code-DO-NOT-LEAK", secret_verifier, transport=transport)
    message = str(excinfo.value)
    assert secret_verifier not in message
    assert "code-DO-NOT-LEAK" not in message
    assert FAKE_KEY not in message


# --- the connect adapter, against a real local fake auth server -------------

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


def test_flow_b_end_to_end_authorize_exchange_persist(tmp_path, fake_server):
    """The real adapter drives a real local HTTP server end to end."""
    store = CredentialStore(tmp_path / "creds.json")
    seen_urls: list[str] = []
    source = PkceCredentialSource(
        store,
        origins=local_origins(fake_server),
        flow="B",
        open_browser=lambda url: seen_urls.append(url),
    )

    attempt = source.start(flow="B")
    assert attempt.authorize_url is not None
    query = parse_qs(urlsplit(attempt.authorize_url).query)
    assert query["callback_url"] == ["oob"]
    assert query["code_challenge_method"] == ["S256"]
    assert seen_urls and seen_urls[0] == attempt.authorize_url

    result = source.complete(attempt.generation, "the-oob-code")

    # The exchange reached the auth origin with the verifier in the body only.
    assert len(fake_server.exchange_requests) == 1
    body = fake_server.exchange_requests[0]
    assert body["code"] == "the-oob-code"
    assert body["code_challenge_method"] == "S256"
    assert body["code_verifier"]

    # ...and the credential was persisted through the normal store.
    assert result.api_key == FAKE_KEY
    assert result.source == SOURCE_PKCE
    stored = store.load(SOURCE_PKCE)
    assert stored is not None and stored.api_key == FAKE_KEY


def test_flow_a_end_to_end_loopback_callback(tmp_path):
    """Full Flow A: loopback listener, browser callback, exchange, persist."""
    import threading
    import urllib.request

    store = CredentialStore(tmp_path / "creds.json")
    server = FakeOrcaServer()
    server.start()
    try:
        holder: dict = {}

        def fake_browser(url: str) -> None:
            holder["url"] = url

            def bounce() -> None:
                query = parse_qs(urlsplit(url).query)
                callback = query["callback_url"][0]
                state = query["state"][0]
                sep = "&" if "?" in callback else "?"
                final = f"{callback}{sep}code=the-loopback-code&state={state}"
                try:
                    urllib.request.urlopen(final, timeout=10).read()
                except Exception as exc:  # pragma: no cover - diagnostics
                    holder["bounce_error"] = str(exc)

            threading.Thread(target=bounce, daemon=True).start()

        source = PkceCredentialSource(
            store,
            origins=local_origins(server),
            flow="A",
            open_browser=fake_browser,
        )
        attempt = source.start(flow="A")
        assert attempt.authorize_url is not None

        # The callback URL is loopback, so the consent screen can redirect back.
        callback = parse_qs(urlsplit(attempt.authorize_url).query)["callback_url"][0]
        assert callback.startswith("http://127.0.0.1:")
        assert "state" not in holder.get("bounce_error", "")

        result = source.wait(attempt.generation, timeout=15)
        assert result is not None
        assert result.api_key == FAKE_KEY
        assert store.load(SOURCE_PKCE).api_key == FAKE_KEY
        assert server.exchange_requests[0]["code"] == "the-loopback-code"
    finally:
        server.stop()


def test_flow_a_rejects_a_callback_with_the_wrong_state(tmp_path):
    import threading
    import urllib.request

    store = CredentialStore(tmp_path / "creds.json")
    server = FakeOrcaServer()
    server.start()
    try:
        def fake_browser(url: str) -> None:
            def bounce() -> None:
                query = parse_qs(urlsplit(url).query)
                callback = query["callback_url"][0]
                final = f"{callback}?code=attacker-code&state=not-the-state"
                try:
                    urllib.request.urlopen(final, timeout=10).read()
                except Exception:
                    pass

            threading.Thread(target=bounce, daemon=True).start()

        source = PkceCredentialSource(
            store, origins=local_origins(server), flow="A", open_browser=fake_browser
        )
        attempt = source.start(flow="A")
        with pytest.raises(StateMismatch):
            source.wait(attempt.generation, timeout=15)
        # Nothing was exchanged and nothing was stored.
        assert server.exchange_requests == []
        assert store.load(SOURCE_PKCE) is None
    finally:
        server.stop()


def test_denial_is_reported_and_stores_nothing(tmp_path, fake_server):
    import threading
    import urllib.request

    store = CredentialStore(tmp_path / "creds.json")

    def fake_browser(url: str) -> None:
        def bounce() -> None:
            query = parse_qs(urlsplit(url).query)
            callback = query["callback_url"][0]
            state = query["state"][0]
            try:
                urllib.request.urlopen(
                    f"{callback}?error=access_denied&state={state}", timeout=10
                ).read()
            except Exception:
                pass

        threading.Thread(target=bounce, daemon=True).start()

    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="A", open_browser=fake_browser
    )
    attempt = source.start(flow="A")
    with pytest.raises(AccessDenied):
        source.wait(attempt.generation, timeout=15)
    assert store.load(SOURCE_PKCE) is None
    assert fake_server.exchange_requests == []


def test_cancel_releases_the_login_lock(tmp_path, fake_server):
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="A",
        open_browser=lambda _u: None,
    )
    attempt = source.start(flow="A")
    assert source.is_current(attempt.generation)
    source.cancel()
    assert not source.is_current(attempt.generation)
    with pytest.raises(AuthorizationCancelled):
        source.wait(attempt.generation, timeout=5)
    # A second login can start after a cancel.
    second = source.start(flow="A")
    assert second.generation != attempt.generation
    source.cancel()


def test_timeout_path_is_terminal(tmp_path):
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=ORIGINS, flow="A", open_browser=lambda _u: None
    )
    attempt = source.start(flow="A")
    # No browser ever calls back.  The listener honours a short window so the
    # test does not wait out the real 10 minute authorization TTL.
    source._attempt.started_at -= 10_000
    receiver_window = 0.5
    from orcarouter.pkce import authorize_flow_a

    with pytest.raises(AuthorizationTimeout):
        authorize_flow_a(
            ORIGINS,
            app_name="Test",
            attempt=source._attempt,
            open_browser=lambda _u: None,
            timeout=receiver_window,
        )


def test_a_late_success_cannot_overwrite_a_newer_login(tmp_path, fake_server):
    """Generation guard: an old attempt's completion is dropped."""
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="B",
        open_browser=lambda _u: None,
    )
    first = source.start(flow="B")
    second = source.start(flow="B")
    assert first.generation != second.generation

    # The first attempt finishes late, after a newer one has begun.
    with pytest.raises(AuthorizationCancelled):
        source.complete(first.generation, "old-code")
    assert store.load(SOURCE_PKCE) is None

    # The current attempt still works.
    result = source.complete(second.generation, "new-code")
    assert result.api_key == FAKE_KEY


def test_complete_drops_the_code_when_the_attempt_expired(tmp_path):
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(store, origins=ORIGINS, flow="B",
                                  open_browser=lambda _u: None)
    attempt = source.start(flow="B")
    source._attempt.started_at -= 10_000  # simulate an expired window
    with pytest.raises(CredentialError) as excinfo:
        source.complete(attempt.generation, "stale-code")
    assert "expired" in str(excinfo.value)


def test_complete_rejects_an_empty_code(tmp_path):
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(store, origins=ORIGINS, flow="B",
                                  open_browser=lambda _u: None)
    attempt = source.start(flow="B")
    with pytest.raises(CredentialError):
        source.complete(attempt.generation, "   ")


def test_clear_removes_the_stored_credential(tmp_path, fake_server):
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(store, origins=local_origins(fake_server), flow="B",
                                  open_browser=lambda _u: None)
    attempt = source.start(flow="B")
    source.complete(attempt.generation, "code")
    assert source.load() is not None
    source.clear()
    assert source.load() is None


def test_each_attempt_sends_a_different_challenge(tmp_path, fake_server):
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(store, origins=local_origins(fake_server), flow="B",
                                  open_browser=lambda _u: None)
    first = source.start(flow="B")
    second = source.start(flow="B")
    c1 = parse_qs(urlsplit(first.authorize_url).query)["code_challenge"][0]
    c2 = parse_qs(urlsplit(second.authorize_url).query)["code_challenge"][0]
    assert c1 != c2


# --- regression: cancellation is generation-scoped --------------------------

def test_cancelling_a_generation_does_not_uncancel_it(tmp_path, fake_server):
    """A cancelled attempt must stay cancelled.

    Clearing the flag when the next generation starts would let the invalidated
    Flow A worker keep running for the full authorization TTL.
    """
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="A",
        open_browser=lambda _u: None,
    )
    first = source.start(flow="A")
    source.cancel(first.generation)
    assert source.is_cancelled(first.generation)

    # Starting (or reserving) the next generation must not revive the old one.
    source.next_generation()
    source.start(flow="A")
    assert source.is_cancelled(first.generation)
    assert not source.is_current(first.generation)
    source.cancel()


def test_cancelling_an_old_generation_does_not_abort_a_newer_login(tmp_path, fake_server):
    """A delayed pagehide beacon carries an old generation; it must not win."""
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="B",
        open_browser=lambda _u: None,
    )
    old = source.start(flow="B")
    new = source.start(flow="B")

    source.cancel(old.generation)          # stale beacon
    assert source.is_current(new.generation)

    result = source.complete(new.generation, "code")
    assert result.api_key == FAKE_KEY


def test_a_cancelled_flow_a_login_never_redeems_its_code(tmp_path):
    """Cancelling must stop the worker before it exchanges the code."""
    import threading
    import time
    import urllib.request

    store = CredentialStore(tmp_path / "creds.json")
    server = FakeOrcaServer()
    server.start()
    try:
        def fake_browser(url: str) -> None:
            # Deliver a valid callback *after* the cancellation, mimicking a
            # consent tab the user approves following a cancel in the app.
            def bounce() -> None:
                time.sleep(0.4)
                query = parse_qs(urlsplit(url).query)
                callback = query["callback_url"][0]
                state = query["state"][0]
                try:
                    urllib.request.urlopen(
                        f"{callback}?code=late-code&state={state}", timeout=10
                    ).read()
                except Exception:
                    pass

            threading.Thread(target=bounce, daemon=True).start()

        source = PkceCredentialSource(
            store, origins=local_origins(server), flow="A", open_browser=fake_browser
        )
        attempt = source.start(flow="A")
        source.cancel(attempt.generation)
        with pytest.raises(AuthorizationCancelled):
            source.wait(attempt.generation, timeout=5)

        # Give the bounced callback time to arrive had the worker still been up.
        time.sleep(1.0)
        assert server.exchange_requests == [], "a cancelled login redeemed its code"
        assert store.load(SOURCE_PKCE) is None
    finally:
        server.stop()


def test_terminal_hook_fires_for_every_attempt_path(tmp_path, fake_server):
    """Hosts holding a login lock are notified without waiting for a poll."""
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="B",
        open_browser=lambda _u: None,
    )
    seen: list[int] = []
    source.on_terminal(seen.append)

    # Flow B has no worker thread: the caller's own exchange result is the
    # terminal signal, so no worker notification is expected here.
    attempt = source.start(flow="B")
    source.complete(attempt.generation, "code")
    assert seen == []

    # A Flow A attempt that is cancelled reaches a terminal state.
    a = source.start(flow="A")
    source.cancel(a.generation)
    with pytest.raises(AuthorizationCancelled):
        source.wait(a.generation, timeout=5)
    assert a.generation in seen


def test_a_bad_terminal_observer_cannot_break_the_flow(tmp_path, fake_server):
    store = CredentialStore(tmp_path / "creds.json")
    source = PkceCredentialSource(
        store, origins=local_origins(fake_server), flow="B",
        open_browser=lambda _u: None,
    )

    def boom(_generation):
        raise RuntimeError("observer is broken")

    source.on_terminal(boom)
    attempt = source.start(flow="B")
    assert source.complete(attempt.generation, "code").api_key == FAKE_KEY
