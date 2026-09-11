"""Authentication adapter: paste an existing ``sk-orca-...`` API key."""

from __future__ import annotations

from typing import Callable

from ..credentials import (
    SOURCE_API_KEY,
    CredentialResult,
    CredentialSource,
    CredentialStore,
    looks_like_key,
)
from ..errors import CredentialError

PROMPT = "OrcaRouter API key (sk-orca-…): "


class ApiKeyCredentialSource(CredentialSource):
    """The "OrcaRouter - API" choice.

    The key is read from a prompt (or supplied directly by the host UI) and
    written to the project's credential store.  It never reaches a URL, a log
    line, an error message, a telemetry event, or a browser that should not
    hold it.
    """

    source_id = SOURCE_API_KEY
    label = "OrcaRouter - API"

    def __init__(
        self,
        store: CredentialStore,
        prompt: Callable[[str], str] | None = None,
    ) -> None:
        self.store = store
        self._prompt = prompt

    def obtain(self, api_key: str | None = None) -> CredentialResult:
        """Validate, store and return the credential.

        ``api_key`` is supplied by the GUI/CLI; when it is omitted an
        interactive prompt is used instead.
        """
        if api_key is None:
            reader = self._prompt or _default_prompt
            api_key = reader(PROMPT)
        api_key = (api_key or "").strip()
        if not api_key:
            raise CredentialError("no OrcaRouter API key was entered")
        if not looks_like_key(api_key):
            # A prefix check is only a typo guard; validity is established by
            # the first real request, never by a paid probe.
            raise CredentialError(
                "that does not look like an OrcaRouter API key "
                "(expected an sk-orca-… value)"
            )
        result = CredentialResult(api_key=api_key, source=self.source_id, scope="api")
        return self.store.save(result)

    def load(self) -> CredentialResult | None:
        return self.store.load(self.source_id)

    def clear(self) -> None:
        self.store.clear(self.source_id)


def _default_prompt(message: str) -> str:
    import getpass

    return getpass.getpass(message)
