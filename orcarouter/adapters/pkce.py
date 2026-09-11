"""Authentication adapter: "Connect with OrcaRouter" (OAuth 2.0 + PKCE).

This adapter owns the whole lifecycle of one interactive login:

1. mint a fresh verifier/challenge/state,
2. open the consent screen (Flow A loopback, or Flow B out-of-band),
3. exchange the code at the **auth** origin,
4. persist the resulting durable API key,
5. release every lock it took, on every terminal path.

It is intentionally independent of the API-key adapter: a user with no browser
still pastes a key, and a user with no key still signs in.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Literal

from ..config import OriginConfig, resolve_origins
from ..credentials import (
    SOURCE_PKCE,
    CredentialResult,
    CredentialSource,
    CredentialStore,
)
from ..errors import AuthorizationCancelled, AuthorizationTimeout, CredentialError
from ..pkce import (
    AUTHORIZATION_TTL_SECONDS,
    PkceAttempt,
    authorize_flow_a,
    authorize_flow_b,
    exchange_code,
    new_attempt,
)
from ..transport import Transport, UrllibTransport

Flow = Literal["A", "B"]

#: Flow A is used by default: this client is a local application (a local web UI
#: or a CLI running on the same machine as the simulation) that can bind
#: 127.0.0.1, so the code returns automatically and the user clicks once.
#: Flow B is kept for deployments that cannot receive a redirect.
DEFAULT_FLOW: Flow = "A"


@dataclass
class LoginAttempt:
    """Public, non-secret state for one in-flight login."""

    generation: int
    flow: Flow
    authorize_url: str | None = None
    done: bool = False


class PkceCredentialSource(CredentialSource):
    """The "OrcaRouter - Auth" choice."""

    source_id = SOURCE_PKCE
    label = "OrcaRouter - Auth"

    def __init__(
        self,
        store: CredentialStore,
        *,
        origins: OriginConfig | None = None,
        transport: Transport | None = None,
        app_name: str = "V-REP YouBot Demo",
        flow: Flow = DEFAULT_FLOW,
        open_browser: Callable[[str], object] | None = None,
    ) -> None:
        self.store = store
        self.origins = origins or resolve_origins()
        self.transport = transport or UrllibTransport()
        self.app_name = app_name
        self.flow = flow
        self._open_browser = open_browser

        self._lock = threading.Lock()
        # Cancellation is tracked *per generation*.  A single shared flag would
        # let a stale worker (or a delayed pagehide beacon) cancel a login the
        # user started afterwards.
        self._cancelled: set[int] = set()
        self._generation = 0
        self._attempt: PkceAttempt | None = None

        # Per-generation async state.  A late response belonging to an older
        # generation is dropped rather than written anywhere.
        self._urls: dict[int, str] = {}
        self._results: dict[int, CredentialResult] = {}
        self._failures: dict[int, BaseException] = {}
        self._threads: dict[int, threading.Thread] = {}
        self._ready: dict[int, threading.Event] = {}
        # Observers notified when an attempt reaches a terminal state, so a
        # host that holds a login lock can release it without waiting for the
        # client to poll.
        self._terminal_hooks: list[Callable[[int], None]] = []

    # -- generation / cancellation -----------------------------------------

    @property
    def generation(self) -> int:
        return self._generation

    def next_generation(self) -> int:
        """Start a new attempt generation, invalidating older responses.

        Previously cancelled generations stay cancelled: clearing them here is
        what would let an invalidated worker keep running.
        """
        with self._lock:
            self._generation += 1
            return self._generation

    def cancel(self, generation: int | None = None) -> None:
        """Release the login lock for ``generation`` (default: the current one).

        Safe to call at any time, from any path.  Cancelling an already
        superseded generation is a no-op, so a stale ``pagehide`` beacon cannot
        abort a login the user started afterwards.
        """
        with self._lock:
            target = self._generation if generation is None else generation
            self._cancelled.add(target)
            if len(self._cancelled) > 64:  # bound the bookkeeping
                self._cancelled = {g for g in self._cancelled if g >= target - 32}

    def is_cancelled(self, generation: int | None = None) -> bool:
        target = self._generation if generation is None else generation
        return target in self._cancelled

    def is_current(self, generation: int) -> bool:
        return generation == self._generation and generation not in self._cancelled

    def on_terminal(self, hook: Callable[[int], None]) -> None:
        """Register a callback invoked when an attempt cannot make progress.

        Used by a host holding a server-side login lock to release it on every
        terminal path -- including a timeout, where no client call ever comes.
        """
        self._terminal_hooks.append(hook)

    def _notify_terminal(self, generation: int) -> None:
        for hook in list(self._terminal_hooks):
            try:
                hook(generation)
            except Exception:  # a bad observer must not break the flow
                pass

    # -- flows --------------------------------------------------------------

    def start(self, *, generation: int | None = None, flow: Flow | None = None) -> LoginAttempt:
        """Begin an authorization attempt and return its public descriptor.

        The descriptor carries the consent URL and the generation -- never the
        verifier or the state.
        """
        gen = self.next_generation() if generation is None else generation
        attempt = new_attempt(generation=gen)
        self._attempt = attempt
        chosen: Flow = flow or self.flow

        if chosen == "A":
            thread = threading.Thread(
                target=self._run_flow_a,
                args=(gen, attempt),
                name=f"orca-pkce-{gen}",
                daemon=True,
            )
            self._threads[gen] = thread
            thread.start()
            # Wait briefly so the consent URL exists before the caller renders.
            ready = threading.Event()
            self._ready[gen] = ready
            ready.wait(timeout=5.0)
        else:
            url = authorize_flow_b(
                self.origins,
                app_name=self.app_name,
                state=attempt.state,
                challenge=attempt.challenge,
                scope=attempt.scope,
                open_browser=self._open_browser,
                on_url=lambda u: self._urls.__setitem__(gen, u),
            )
            self._urls[gen] = url
        return LoginAttempt(gen, chosen, self._urls.get(gen))

    def _run_flow_a(self, gen: int, attempt: PkceAttempt) -> None:
        def on_url(url: str) -> None:
            self._urls[gen] = url
            self._ready.get(gen, threading.Event()).set()

        try:
            code = authorize_flow_a(
                self.origins,
                app_name=self.app_name,
                attempt=attempt,
                open_browser=self._open_browser,
                timeout=AUTHORIZATION_TTL_SECONDS,
                on_url=on_url,
                # Stop as soon as this attempt is cancelled *or* superseded, so
                # an abandoned login cannot still redeem its code upstream.
                is_cancelled=lambda: not self.is_current(gen),
            )
            self._finish(gen, attempt, code)
        except BaseException as exc:  # surfaced to the UI through poll()
            self._failures[gen] = exc
        finally:
            self._ready.get(gen, threading.Event()).set()
            self._threads.pop(gen, None)
            self._notify_terminal(gen)

    def complete(self, generation: int, code: str) -> CredentialResult:
        """Flow B: exchange a user-supplied out-of-band code."""
        if not self.is_current(generation):
            raise AuthorizationCancelled(
                "this authorization attempt is no longer current; start again"
            )
        attempt = self._attempt
        if attempt is None or attempt.generation != generation:
            raise CredentialError(
                "no pending OrcaRouter authorization; start the connect flow again"
            )
        if attempt.expired():
            raise CredentialError(
                "this authorization attempt expired; start the connect flow again"
            )
        if not code or not code.strip():
            raise CredentialError("no authorization code was entered")
        return self._finish(generation, attempt, code.strip())

    def poll(self, generation: int) -> CredentialResult | None:
        """Return the result of a Flow A attempt once it is ready."""
        if generation in self._failures:
            raise self._failures.pop(generation)
        result = self._results.get(generation)
        if result is not None and self.is_current(generation):
            return result
        return None

    def wait(self, generation: int, timeout: float = AUTHORIZATION_TTL_SECONDS):
        thread = self._threads.get(generation)
        if thread is not None:
            thread.join(timeout=max(0.0, timeout))
        return self.poll(generation)

    # -- internals ----------------------------------------------------------

    def _finish(self, generation: int, attempt: PkceAttempt, code: str) -> CredentialResult:
        # Check *before* redeeming the code.  A cancelled or superseded attempt
        # must not spend one of the user's limited key issuances upstream, even
        # if its callback arrived in the instant between two cancellation
        # checks.
        if not self.is_current(generation):
            raise AuthorizationCancelled(
                "this OrcaRouter login was cancelled or superseded"
            )
        exchange = exchange_code(
            self.origins, code, attempt.verifier, transport=self.transport
        )
        # Re-check after the round trip: a login cancelled while the exchange
        # was in flight must not have its result written anywhere.
        if not self.is_current(generation):
            raise AuthorizationCancelled(
                "a newer OrcaRouter login superseded this one"
            )
        result = self.store.save(exchange.to_credential())
        self._results[generation] = result
        return result

    # -- CredentialSource ---------------------------------------------------

    def obtain(self, code: str | None = None, generation: int | None = None) -> CredentialResult:
        """Run a full interactive login (CLI convenience entry point)."""
        descriptor = self.start(generation=generation)
        if descriptor.flow == "B":
            if code is None:
                raise CredentialError(
                    "this flow shows a code in the browser; pass it back to "
                    "finish connecting"
                )
            return self.complete(descriptor.generation, code)
        result = self.wait(descriptor.generation)
        if result is None:
            raise CredentialError("the OrcaRouter login did not complete")
        return result

    def load(self) -> CredentialResult | None:
        return self.store.load(self.source_id)

    def clear(self) -> None:
        self.store.clear(self.source_id)
        self.cancel()