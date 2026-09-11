"""Model discovery, capability filtering, and outage fallback."""

from __future__ import annotations

import json

import pytest

from orcarouter.catalog import (
    CAPABILITY_CHAT,
    CAPABILITY_EMBEDDING,
    CAPABILITY_IMAGE,
    CAPABILITY_RERANK,
    CAPABILITY_VIDEO,
    SEED_MODELS,
    CatalogCache,
    CatalogError,
    ModelInfo,
    catalog_as_json,
    fetch_catalog,
    filter_models,
    parse_catalog,
    parse_model,
    seed_snapshot,
)
from orcarouter.config import resolve_origins
from orcarouter.errors import CatalogError as CatalogErr
from orcarouter.errors import TerminalReauthRequired
from tests.helpers import (
    FAKE_KEY,
    FakeTransport,
    chat_model,
    json_response,
    models_payload,
)


# --- fixtures covering every shape the catalog can advertise ----------------

def full_fixture_records():
    return [
        chat_model("text/model", ("text",), ("openai",)),
        chat_model("vision/model", ("text", "image"), ("openai", "anthropic")),
        chat_model("audio/model", ("text", "audio"), ("openai",)),
        chat_model("video-in/model", ("text", "video"), ("gemini",)),
        {
            "id": "embed/model",
            "supported_endpoint_types": ["embeddings"],
            "architecture": {"input_modalities": ["text"], "output_modalities": []},
        },
        {
            "id": "image/gen-model",
            "supported_endpoint_types": ["image-generation"],
            "architecture": {"input_modalities": ["text"], "output_modalities": ["image"]},
        },
        {
            "id": "video/gen-model",
            "supported_endpoint_types": ["openai-video"],
        },
        {
            "id": "rerank/model",
            "supported_endpoint_types": ["jina-rerank"],
        },
        # A model the chat capability endpoint may return but this client
        # cannot speak as text.
        {
            "id": "image/only-model",
            "supported_endpoint_types": ["image-generation", "openai"],
            "architecture": {"input_modalities": ["text"], "output_modalities": ["image"]},
        },
    ]


def models():
    return parse_catalog(models_payload(full_fixture_records()))


# --- parsing ----------------------------------------------------------------

def test_parse_preserves_vendor_namespace():
    parsed = parse_model({"id": "openai/gpt-5.5"})
    assert parsed is not None
    assert parsed.id == "openai/gpt-5.5"


def test_parse_reads_architecture_and_limits():
    parsed = parse_model({
        "id": "a/b",
        "name": "A B",
        "owned_by": "X",
        "context_length": 1000,
        "max_completion_tokens": 100,
        "supported_endpoint_types": ["openai"],
        "architecture": {"input_modalities": ["text", "image"],
                         "output_modalities": ["text"]},
        "reasoning": {"efforts": ["low", "high"]},
    })
    assert parsed.input_modalities == ("text", "image")
    assert parsed.context_length == 1000
    assert parsed.reasoning_efforts == ("low", "high")


def test_parse_falls_back_to_top_provider_context():
    parsed = parse_model({"id": "a/b", "top_provider": {"context_length": 4096}})
    assert parsed.context_length == 4096


@pytest.mark.parametrize("record", [None, 5, {}, {"id": ""}, {"id": 7}, "x"])
def test_unusable_records_are_skipped(record):
    assert parse_model(record) is None


def test_parse_bounds_and_deduplicates():
    records = [chat_model(f"m/{i}") for i in range(20)] + [chat_model("m/0")]
    parsed = parse_catalog(models_payload(records), limit=5)
    assert len(parsed) == 5
    assert len({m.id for m in parsed}) == 5


def test_parse_rejects_a_non_list_payload():
    with pytest.raises(CatalogErr):
        parse_catalog({"data": "nope"})


# --- capability filters -----------------------------------------------------

def test_chat_filter_excludes_non_text_specialists():
    ids = {m.id for m in filter_models(models(), CAPABILITY_CHAT)}
    assert "text/model" in ids
    assert "vision/model" in ids
    # image/only-model advertises the openai endpoint but is an image generator.
    assert "image/only-model" not in ids
    assert "embed/model" not in ids
    assert "image/gen-model" not in ids
    assert "video/gen-model" not in ids
    assert "rerank/model" not in ids


def test_chat_filter_requires_a_supported_endpoint_type():
    unsupported = ModelInfo(id="x/y", name="x/y", supported_endpoint_types=("weird",))
    assert filter_models([unsupported], CAPABILITY_CHAT) == ()


def test_multimodal_image_filter_is_fail_closed():
    ids = {m.id for m in filter_models(models(), CAPABILITY_CHAT,
                                       required_input_modalities=["image"])}
    assert ids == {"vision/model"}


def test_multimodal_audio_and_video_filters():
    audio = filter_models(models(), CAPABILITY_CHAT, required_input_modalities=["audio"])
    video = filter_models(models(), CAPABILITY_CHAT, required_input_modalities=["video"])
    assert {m.id for m in audio} == {"audio/model"}
    assert {m.id for m in video} == {"video-in/model"}


def test_undeclared_modality_is_not_accepted():
    """Fail closed: a model with no architecture block matches nothing."""
    bare = ModelInfo(id="bare/model", name="bare", supported_endpoint_types=("openai",))
    assert bare.input_modalities == ()
    assert filter_models([bare], CAPABILITY_CHAT, required_input_modalities=["image"]) == ()
    # It is still a valid text model.
    assert filter_models([bare], CAPABILITY_CHAT) == (bare,)


def test_embedding_image_video_rerank_filters():
    all_models = models()
    assert {m.id for m in filter_models(all_models, CAPABILITY_EMBEDDING)} == {"embed/model"}
    assert {m.id for m in filter_models(all_models, CAPABILITY_IMAGE)} == {
        "image/gen-model", "image/only-model"}
    assert {m.id for m in filter_models(all_models, CAPABILITY_VIDEO)} == {"video/gen-model"}
    assert {m.id for m in filter_models(all_models, CAPABILITY_RERANK)} == {"rerank/model"}


def test_unknown_capability_is_refused():
    with pytest.raises(CatalogErr):
        filter_models(models(), "telepathy")


# --- live discovery ---------------------------------------------------------

def test_fetch_catalog_uses_the_api_origin_and_bearer_auth():
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response(models_payload([chat_model("a/b")])),
    })
    snapshot = fetch_catalog(
        CAPABILITY_CHAT, origins=resolve_origins({}), api_key=FAKE_KEY,
        transport=transport,
    )
    assert snapshot.source == "live"
    assert snapshot.ids == ("a/b",)
    call = transport.calls[0]
    assert call.url == "https://api.orcarouter.ai/v1/models?capability=chat"
    assert call.headers["Authorization"] == f"Bearer {FAKE_KEY}"
    assert "www.orcarouter.ai" not in call.url


def test_fetch_catalog_401_is_terminal_reauth_not_a_retry():
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response({"error": "invalid key"}, 401),
    })
    with pytest.raises(TerminalReauthRequired):
        fetch_catalog(CAPABILITY_CHAT, origins=resolve_origins({}),
                      api_key=FAKE_KEY, transport=transport)
    assert len(transport.calls) == 1  # no retry loop


def test_fetch_catalog_network_failure_is_a_catalog_error():
    from orcarouter.errors import NetworkError

    class Boom(FakeTransport):
        def request(self, *a, **kw):
            raise NetworkError("connection refused")

    with pytest.raises(CatalogErr):
        fetch_catalog(CAPABILITY_CHAT, origins=resolve_origins({}), transport=Boom())


# --- cache and fallback -----------------------------------------------------

def test_live_success_replaces_the_cache_and_is_authoritative():
    cache = CatalogCache()
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response(models_payload([
            chat_model("live/model", ("text", "image")),
        ])),
    })
    first = cache.refresh(CAPABILITY_CHAT, origins=resolve_origins({}), transport=transport)
    assert first.source == "live"
    assert first.degraded is False
    assert first.ids == ("live/model",)

    # A second success is authoritative: the previous result is not merged in.
    transport2 = FakeTransport(responses={
        ("GET", "/v1/models"): json_response(models_payload([chat_model("other/model")])),
    })
    second = cache.refresh(CAPABILITY_CHAT, origins=resolve_origins({}), transport=transport2)
    assert second.ids == ("other/model",)


def test_outage_without_cache_falls_back_to_the_verified_seed():
    cache = CatalogCache()
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response({"error": "boom"}, 500),
    })
    snapshot = cache.refresh(CAPABILITY_CHAT, origins=resolve_origins({}),
                             transport=transport)
    assert snapshot.source == "seed"
    assert snapshot.degraded is True
    assert "openai/gpt-5.5" in snapshot.ids
    assert "orcarouter/auto" in snapshot.ids


def test_outage_with_cache_reuses_last_known_good_live_catalog():
    cache = CatalogCache()
    good = FakeTransport(responses={
        ("GET", "/v1/models"): json_response(models_payload([chat_model("live/model")])),
    })
    cache.refresh(CAPABILITY_CHAT, origins=resolve_origins({}), transport=good)
    bad = FakeTransport(responses={
        ("GET", "/v1/models"): json_response({"error": "boom"}, 503),
    })
    snapshot = cache.refresh(CAPABILITY_CHAT, origins=resolve_origins({}), transport=bad)
    assert snapshot.source == "live"          # last known-good, not the seed
    assert snapshot.ids == ("live/model",)
    assert snapshot.degraded is True
    assert "last known-good" in snapshot.detail


def test_seed_is_never_mixed_into_a_live_result():
    cache = CatalogCache()
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response(models_payload([chat_model("live/only")])),
    })
    snapshot = cache.refresh(CAPABILITY_CHAT, origins=resolve_origins({}), transport=transport)
    assert "openai/gpt-5.5" not in snapshot.ids


def test_seed_preserves_verified_metadata_and_reasoning_ladder():
    seed = {m.id: m for m in SEED_MODELS}
    gpt = seed["openai/gpt-5.5"]
    assert gpt.reasoning_efforts == ("low", "medium", "high", "xhigh")
    assert "image" in gpt.input_modalities
    assert gpt.context_length == 272000
    # orcarouter/auto is deliberately text-only in the verified seed.
    assert seed["orcarouter/auto"].input_modalities == ("text",)


def test_seed_snapshot_is_already_filtered_for_the_surface():
    snapshot = seed_snapshot(CAPABILITY_CHAT, ["image"])
    assert "openai/gpt-5.5" in snapshot.ids
    # deepseek and orcarouter/auto declare no image input, so they are excluded.
    assert "deepseek/deepseek-v4-pro" not in snapshot.ids
    assert "orcarouter/auto" not in snapshot.ids


def test_reauth_errors_propagate_through_the_cache():
    cache = CatalogCache()
    transport = FakeTransport(responses={
        ("GET", "/v1/models"): json_response({"error": "revoked"}, 401),
    })
    with pytest.raises(TerminalReauthRequired):
        cache.refresh(CAPABILITY_CHAT, origins=resolve_origins({}), transport=transport)
    # A revoked key must not silently degrade into a seed catalog.
    assert cache.get(CAPABILITY_CHAT) is None


def test_degraded_status_is_reported_in_the_public_payload():
    snapshot = seed_snapshot(CAPABILITY_CHAT, detail="live discovery failed")
    payload = json.loads(catalog_as_json(snapshot))
    assert payload["degraded"] is True
    assert payload["source"] == "seed"
    assert payload["count"] == len(snapshot)
    assert payload["detail"] == "live discovery failed"
    # No credential material in the payload.
    assert "api_key" not in json.dumps(payload)


def test_seed_ids_match_the_documented_fallback_list():
    assert [m.id for m in SEED_MODELS] == [
        "openai/gpt-5.5",
        "anthropic/claude-opus-4.8",
        "google/gemini-3.5-flash",
        "deepseek/deepseek-v4-pro",
        "orcarouter/auto",
    ]


def test_catalog_error_alias_is_exported():
    assert CatalogError is CatalogErr
