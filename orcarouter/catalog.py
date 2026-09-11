"""Live model discovery and capability filtering.

The single source of truth for the OrcaRouter model list is
``GET <api-base>/models`` on the *inference* origin.  Model ids keep their
``vendor/model`` namespace exactly as returned.

Each AI input surface asks for its own capability view, so the selector a user
sees can never offer a model that surface cannot speak:

===============  ===========================================================
surface          filter
===============  ===========================================================
text chat/agent  ``?capability=chat`` and a text-capable endpoint type
image / audio /  chat **plus** the matching entry in
video understanding ``architecture.input_modalities`` (fail closed otherwise)
embedding        ``?capability=embedding``
image generation ``?capability=image``
video generation ``?capability=video``
rerank           ``?capability=rerank``
===============  ===========================================================

When live discovery fails, a small **verified** fallback catalog keeps a fresh
installation usable.  It is never merged into a successful live result: live
discovery is authoritative.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence

from .config import OriginConfig, resolve_origins
from .errors import ApiError, CatalogError, NetworkError, TerminalReauthRequired
from .transport import DEFAULT_TIMEOUT, Transport, UrllibTransport

#: Capability values accepted by the catalog endpoint.
CAPABILITY_CHAT = "chat"
CAPABILITY_EMBEDDING = "embedding"
CAPABILITY_IMAGE = "image"
CAPABILITY_VIDEO = "video"
CAPABILITY_RERANK = "rerank"

#: Endpoint types the OpenAI-compatible chat client in this project can speak.
TEXT_ENDPOINT_TYPES = frozenset({"openai", "anthropic", "gemini", "openai-response"})
#: Endpoint types that are *not* text chat, even though the chat capability
#: filter may return them.
NON_TEXT_ENDPOINT_TYPES = frozenset({"image-generation", "openai-video", "jina-rerank"})

MAX_MODELS = 1000


@dataclass(frozen=True)
class ModelInfo:
    """Minimal model metadata.  No credential ever appears here."""

    id: str
    name: str
    owned_by: str = ""
    context_length: int | None = None
    max_completion_tokens: int | None = None
    input_modalities: tuple[str, ...] = ()
    output_modalities: tuple[str, ...] = ()
    supported_endpoint_types: tuple[str, ...] = ()
    reasoning_efforts: tuple[str, ...] = ()
    description: str = ""

    @property
    def supports_text(self) -> bool:
        return bool(TEXT_ENDPOINT_TYPES.intersection(self.supported_endpoint_types))

    @property
    def is_non_text_specialist(self) -> bool:
        return bool(NON_TEXT_ENDPOINT_TYPES.intersection(self.supported_endpoint_types))

    def accepts_input(self, modality: str) -> bool:
        """Fail closed: an undeclared modality is not accepted."""
        return modality in self.input_modalities

    def to_public_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "owned_by": self.owned_by,
            "context_length": self.context_length,
            "max_completion_tokens": self.max_completion_tokens,
            "input_modalities": list(self.input_modalities),
            "output_modalities": list(self.output_modalities),
            "supported_endpoint_types": list(self.supported_endpoint_types),
            "reasoning_efforts": list(self.reasoning_efforts),
        }


@dataclass
class CatalogSnapshot:
    """One catalog view, plus whether it came from live discovery."""

    models: tuple[ModelInfo, ...]
    source: str  # "live" | "last_known_good" | "seed"
    capability: str = CAPABILITY_CHAT
    degraded: bool = False
    detail: str = ""

    @property
    def ids(self) -> tuple[str, ...]:
        return tuple(m.id for m in self.models)

    def __len__(self) -> int:
        return len(self.models)


# ---------------------------------------------------------------------------
# Verified cold-start seed
# ---------------------------------------------------------------------------
# Every entry below was verified against
# GET https://api.orcarouter.ai/v1/models on 2026-09-11.  The catalog endpoint
# does not expose a reasoning-effort ladder, so the ladder recorded for
# openai/gpt-5.5 is the value verified by the provider's own documentation and
# is preserved here rather than being inferred from the model name.

SEED_MODELS: tuple[ModelInfo, ...] = (
    ModelInfo(
        id="openai/gpt-5.5",
        name="OpenAI: GPT-5.5",
        owned_by="OpenAI",
        context_length=272000,
        max_completion_tokens=128000,
        input_modalities=("text", "image", "file"),
        output_modalities=("text",),
        supported_endpoint_types=("openai", "openai-response"),
        reasoning_efforts=("low", "medium", "high", "xhigh"),
    ),
    ModelInfo(
        id="anthropic/claude-opus-4.8",
        name="Anthropic: Claude Opus 4.8",
        owned_by="Anthropic",
        input_modalities=("text", "image"),
        output_modalities=("text",),
        supported_endpoint_types=("openai", "anthropic", "openai-response"),
        reasoning_efforts=("low", "medium", "high"),
    ),
    ModelInfo(
        id="google/gemini-3.5-flash",
        name="Gemini 3.5 Flash",
        owned_by="Google",
        input_modalities=("text", "image"),
        output_modalities=("text",),
        supported_endpoint_types=("openai", "gemini"),
        reasoning_efforts=("low", "medium", "high"),
    ),
    ModelInfo(
        id="deepseek/deepseek-v4-pro",
        name="DeepSeek: DeepSeek V4 Pro",
        owned_by="custom",
        input_modalities=("text",),
        output_modalities=("text",),
        supported_endpoint_types=("openai", "openai-response"),
        reasoning_efforts=("low", "medium", "high"),
    ),
    ModelInfo(
        id="orcarouter/auto",
        name="OrcaRouter Auto",
        owned_by="orcarouter",
        input_modalities=("text",),
        output_modalities=("text",),
        supported_endpoint_types=("openai", "openai-response", "anthropic", "gemini"),
    ),
)


def _as_tuple(value: object) -> tuple[str, ...]:
    if isinstance(value, str):
        return (value,)
    if isinstance(value, Sequence):
        return tuple(str(v) for v in value)
    return ()


def _as_int(value: object) -> int | None:
    try:
        return int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


def _extract_reasoning_efforts(record: Mapping) -> tuple[str, ...]:
    """Read a reasoning-effort ladder only when the catalog declares one.

    Capabilities are never guessed from a model name.
    """
    for key in ("reasoning_efforts", "reasoning_effort", "supported_reasoning_efforts"):
        if key in record:
            return _as_tuple(record.get(key))
    reasoning = record.get("reasoning")
    if isinstance(reasoning, Mapping):
        for key in ("efforts", "supported_efforts", "levels"):
            if key in reasoning:
                return _as_tuple(reasoning.get(key))
    return ()


def parse_model(record: object) -> ModelInfo | None:
    """Parse one catalog record, returning ``None`` for unusable entries."""
    if not isinstance(record, Mapping):
        return None
    model_id = record.get("id")
    if not isinstance(model_id, str) or not model_id.strip():
        return None
    architecture = record.get("architecture")
    architecture = architecture if isinstance(architecture, Mapping) else {}
    top_provider = record.get("top_provider")
    top_provider = top_provider if isinstance(top_provider, Mapping) else {}

    context_length = _as_int(record.get("context_length"))
    if context_length is None:
        context_length = _as_int(top_provider.get("context_length")) or None
    max_completion = _as_int(record.get("max_completion_tokens"))
    if max_completion is None:
        max_completion = _as_int(top_provider.get("max_completion_tokens")) or None

    name = record.get("name")
    return ModelInfo(
        id=model_id.strip(),
        name=str(name) if name else model_id.strip(),
        owned_by=str(record.get("owned_by") or ""),
        context_length=context_length,
        max_completion_tokens=max_completion,
        input_modalities=_as_tuple(architecture.get("input_modalities")),
        output_modalities=_as_tuple(architecture.get("output_modalities")),
        supported_endpoint_types=_as_tuple(record.get("supported_endpoint_types")),
        reasoning_efforts=_extract_reasoning_efforts(record),
        description=str(record.get("description") or "")[:400],
    )


def parse_catalog(payload: object, limit: int = MAX_MODELS) -> tuple[ModelInfo, ...]:
    """Parse a ``/models`` response into bounded, de-duplicated models."""
    if isinstance(payload, Mapping):
        records: Iterable = payload.get("data", [])
    elif isinstance(payload, Sequence) and not isinstance(payload, (str, bytes)):
        records = payload
    else:
        raise CatalogError("catalog response was not a model list")
    if isinstance(records, (str, bytes)) or not isinstance(records, Sequence):
        raise CatalogError("catalog response has no model array")

    models: list[ModelInfo] = []
    seen: set[str] = set()
    for record in records:
        if len(models) >= limit:
            break
        parsed = parse_model(record)
        if parsed is None or parsed.id in seen:
            continue
        seen.add(parsed.id)
        models.append(parsed)
    return tuple(models)


def fetch_catalog(
    capability: str = CAPABILITY_CHAT,
    *,
    origins: OriginConfig | None = None,
    api_key: str | None = None,
    transport: Transport | None = None,
    timeout: float = DEFAULT_TIMEOUT,
    limit: int = MAX_MODELS,
) -> CatalogSnapshot:
    """Fetch the live catalog for one capability.

    A 401 means the credential was revoked: this raises
    :class:`TerminalReauthRequired` instead of retrying, because a durable key
    has no refresh grant.
    """
    origins = origins or resolve_origins()
    transport = transport or UrllibTransport()
    headers = {"Accept": "application/json"}
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"

    url = f"{origins.models_url}?capability={capability}"
    try:
        result = transport.request("GET", url, headers=headers, timeout=timeout)
    except NetworkError as exc:
        raise CatalogError(f"model discovery failed: {exc}") from exc

    if result.status == 200:
        return CatalogSnapshot(
            models=parse_catalog(result.json(), limit=limit),
            source="live",
            capability=capability,
        )
    if result.status in (401, 403):
        raise TerminalReauthRequired(account_id=None, generation=0)
    if result.status == 429:
        raise CatalogError("model discovery is rate limited; try again shortly")
    raise CatalogError(f"model discovery failed with HTTP {result.status}")


# ---------------------------------------------------------------------------
# Capability filters
# ---------------------------------------------------------------------------


def normalize_modalities(modalities: Iterable[str] | str | None) -> tuple[str, ...]:
    """Accept ``"image"``, ``"image,audio"`` or ``["image"]`` uniformly."""
    if modalities is None:
        return ()
    if isinstance(modalities, str):
        parts = modalities.split(",")
    else:
        parts = list(modalities)
    return tuple(p.strip() for p in parts if p and p.strip())


def filter_models(
    models: Iterable[ModelInfo],
    capability: str = CAPABILITY_CHAT,
    *,
    required_input_modalities: Iterable[str] | str = (),
) -> tuple[ModelInfo, ...]:
    """Apply one surface's capability rules to a candidate model list.

    This is the single place capability policy lives: GUI pages and CLI
    commands call it rather than re-deriving rules.
    """
    required = tuple(
        m for m in normalize_modalities(required_input_modalities) if m != "text"
    )
    out: list[ModelInfo] = []
    for model in models:
        if capability == CAPABILITY_CHAT:
            if not model.supports_text or model.is_non_text_specialist:
                continue
            # Multimodal surfaces fail closed: a model that does not declare the
            # modality this surface uploads is never offered.
            if any(not model.accepts_input(m) for m in required):
                continue
        elif capability == CAPABILITY_EMBEDDING:
            if "embeddings" not in model.supported_endpoint_types:
                continue
        elif capability == CAPABILITY_IMAGE:
            if "image-generation" not in model.supported_endpoint_types:
                continue
        elif capability == CAPABILITY_VIDEO:
            if "openai-video" not in model.supported_endpoint_types:
                continue
        elif capability == CAPABILITY_RERANK:
            if "jina-rerank" not in model.supported_endpoint_types:
                continue
        else:
            raise CatalogError(f"unknown capability {capability!r}")
        out.append(model)
    return tuple(out)


def seed_snapshot(
    capability: str = CAPABILITY_CHAT,
    required_input_modalities: Iterable[str] | str = (),
    *,
    detail: str = "",
) -> CatalogSnapshot:
    """The verified fallback catalog, already filtered for the surface."""
    return CatalogSnapshot(
        models=filter_models(
            SEED_MODELS,
            capability,
            required_input_modalities=required_input_modalities,
        ),
        source="seed",
        capability=capability,
        degraded=True,
        detail=detail or "using the verified cold-start catalog",
    )


class CatalogCache:
    """Bounded catalog cache with an explicit degraded state.

    Live success replaces the cache.  On failure the last known-good live
    catalog is reused; with no cache at all, the verified seed is used.  The
    seed is never mixed into a live result.
    """

    def __init__(self, max_age_seconds: float = 900.0) -> None:
        self.max_age_seconds = max_age_seconds
        self._snapshots: dict[tuple[str, tuple[str, ...]], CatalogSnapshot] = {}

    @staticmethod
    def _key(capability: str, required: tuple[str, ...]) -> tuple[str, tuple[str, ...]]:
        return (capability, tuple(sorted(required)))

    def get(
        self,
        capability: str = CAPABILITY_CHAT,
        required_input_modalities: Iterable[str] | str = (),
    ) -> CatalogSnapshot | None:
        return self._snapshots.get(
            self._key(capability, normalize_modalities(required_input_modalities))
        )

    def refresh(
        self,
        capability: str = CAPABILITY_CHAT,
        required_input_modalities: Iterable[str] | str = (),
        *,
        origins: OriginConfig | None = None,
        api_key: str | None = None,
        transport: Transport | None = None,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> CatalogSnapshot:
        """Refresh one capability view, falling back rather than failing."""
        required = normalize_modalities(required_input_modalities)
        key = self._key(capability, required)
        try:
            live = fetch_catalog(
                capability,
                origins=origins,
                api_key=api_key,
                transport=transport,
                timeout=timeout,
            )
            live.models = filter_models(
                live.models, capability, required_input_modalities=required
            )
            self._snapshots[key] = live
            return live
        except TerminalReauthRequired:
            raise
        except (CatalogError, NetworkError, ApiError) as exc:
            cached = self._snapshots.get(key)
            if cached is not None:
                cached.degraded = True
                cached.detail = f"live discovery failed ({exc}); using last known-good catalog"
                return cached
            fallback = seed_snapshot(
                capability, required, detail=f"live discovery failed ({exc})"
            )
            self._snapshots[key] = fallback
            return fallback

    def invalidate(self) -> None:
        self._snapshots.clear()


def catalog_as_json(snapshot: CatalogSnapshot) -> str:
    """Serialize a snapshot for the local web UI (no credentials inside)."""
    return json.dumps(
        {
            "source": snapshot.source,
            "degraded": snapshot.degraded,
            "detail": snapshot.detail,
            "capability": snapshot.capability,
            "count": len(snapshot),
            "models": [m.to_public_dict() for m in snapshot.models],
        },
        separators=(",", ":"),
    )
