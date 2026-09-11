"""Command line entry points for the OrcaRouter provider.

Both authentication choices are discoverable here:

* ``orcarouter connect``   -- Connect with OrcaRouter (OAuth 2.0 + PKCE)
* ``orcarouter set-key``   -- paste an existing ``sk-orca-…`` key

They feed the same inference client and the same model catalog.
"""

from __future__ import annotations

import argparse
import json
import sys

from .catalog import CAPABILITY_CHAT, CatalogCache, fetch_catalog
from .config import resolve_origins
from .credentials import (
    SOURCE_API_KEY,
    SOURCE_PKCE,
    CredentialError,
    CredentialStore,
    mask_secret,
)
from .errors import OrcaRouterError, TerminalReauthRequired
from .lifecycle import CredentialLifecycle, ReauthState
from .provider import (
    ORCAROUTER_API_PROVIDER_ID,
    ORCAROUTER_AUTH_PROVIDER_ID,
    OrcaRouterClient,
)
from .registry import PROVIDERS


def _store(args) -> CredentialStore:
    return CredentialStore(args.store)


def cmd_status(args) -> int:
    lifecycle = CredentialLifecycle(_store(args))
    origins = resolve_origins()
    print("OrcaRouter providers:")
    for spec in PROVIDERS:
        print(f"  {spec.id:<20} {spec.label:<20} base_url={spec.base_url}")
    print(f"  auth origin: {origins.auth_base}")
    print()
    for account in lifecycle.all_status():
        state = account.state.value
        masked = mask_secret(account.masked_key) if account.masked_key else "-"
        print(f"  {account.source_id:<20} {state:<16} {masked}")
        if account.detail:
            print(f"      {account.detail}")
    return 0


def cmd_set_key(args) -> int:
    """The API-key adapter, from the CLI."""
    from .web.service import OrcaRouterService

    service = OrcaRouterService(store=_store(args), origins=resolve_origins())
    key = args.key
    if not key:
        import getpass

        key = getpass.getpass("OrcaRouter API key (sk-orca-…): ")
    result = service.save_api_key(key)
    print(f"stored credential for {ORCAROUTER_API_PROVIDER_ID}: "
          f"{result['credential']['masked_key']}")
    return 0


def cmd_connect(args) -> int:
    """The PKCE adapter, from the CLI (Flow A loopback by default)."""
    from .adapters.pkce import PkceCredentialSource

    source = PkceCredentialSource(
        _store(args), origins=resolve_origins(), flow=args.flow
    )
    if args.flow == "B":
        attempt = source.start(flow="B")
        print("Authorize in your browser, then paste the code back here:")
        print("  " + (attempt.authorize_url or ""))
        code = args.code or input("Code: ").strip()
        result = source.complete(attempt.generation, code)
    else:
        print("Opening your browser to authorize…")
        result = source.obtain()
    print(f"connected: {result.masked} (scope {result.scope}, "
          f"account {result.account_id})")
    return 0


def cmd_clear(args) -> int:
    """Clear one authentication choice.

    ``--source`` accepts a provider id (``orcarouter`` / ``orcarouter-oauth``)
    or a credential-source id; the registry maps between them, so the CLI, the
    UI and the store all agree on what is being removed.
    """
    from .web.service import OrcaRouterService

    service = OrcaRouterService(store=_store(args), origins=resolve_origins())
    resolved = service.resolve_source_id(args.source)
    service.clear_credential(resolved)
    print(f"cleared {args.source} ({resolved})")
    return 0


def cmd_models(args) -> int:
    """List the live catalog for one capability."""
    store = _store(args)
    lifecycle = CredentialLifecycle(store)
    key = None
    for source_id in (SOURCE_API_KEY, SOURCE_PKCE):
        status = lifecycle.status(source_id)
        if status.state is ReauthState.READY:
            try:
                key = lifecycle.credential_for(source_id).api_key
                break
            except OrcaRouterError:
                continue
    cache = CatalogCache()
    snapshot = cache.refresh(
        args.capability, args.modalities, origins=resolve_origins(), api_key=key
    )
    print(f"# source={snapshot.source} degraded={snapshot.degraded} "
          f"count={len(snapshot)}")
    if snapshot.detail:
        print(f"# {snapshot.detail}")
    for model in snapshot.models:
        print(f"{model.id}\t{model.name}")
    return 0


def cmd_chat(args) -> int:
    """One real completion through the provider code path."""
    store = _store(args)
    lifecycle = CredentialLifecycle(store)
    for source_id in (SOURCE_API_KEY, SOURCE_PKCE):
        status = lifecycle.status(source_id)
        if status.state is not ReauthState.READY:
            continue
        credential = lifecycle.credential_for(source_id)
        client = OrcaRouterClient(credential, origins=resolve_origins())
        try:
            payload = client.chat_completion(
                args.model, [{"role": "user", "content": args.prompt}]
            )
        except TerminalReauthRequired:
            lifecycle.mark_rejected(credential)
            print("credential rejected (401); this account needs reauthentication",
                  file=sys.stderr)
            return 2
        choice = (payload.get("choices") or [{}])[0]
        message = choice.get("message") or {}
        print(json.dumps({
            "model": payload.get("model"),
            "content": message.get("content"),
            "usage": payload.get("usage"),
            "credential_source": credential.source,
        }, indent=2)[:4000])
        return 0
    print("no usable OrcaRouter credential; run `orcarouter connect` or "
          "`orcarouter set-key`", file=sys.stderr)
    return 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="orcarouter",
        description="OrcaRouter provider for the V-REP YouBot demos.",
    )
    parser.add_argument("--store", default=None, help="credential file path")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", help="show both authentication choices").set_defaults(
        func=cmd_status
    )

    set_key = sub.add_parser("set-key", help="store an existing sk-orca-… API key")
    set_key.add_argument("--key", default=None, help="the key (omit to be prompted)")
    set_key.set_defaults(func=cmd_set_key)

    connect = sub.add_parser(
        "connect", help="Connect with OrcaRouter (OAuth 2.0 + PKCE)"
    )
    connect.add_argument("--flow", choices=("A", "B"), default="A",
                         help="A = loopback redirect (default), B = out-of-band code")
    connect.add_argument("--code", default=None, help="Flow B code")
    connect.set_defaults(func=cmd_connect)

    clear = sub.add_parser("clear", help="remove a stored credential")
    clear.add_argument(
        "--source",
        choices=(ORCAROUTER_API_PROVIDER_ID, ORCAROUTER_AUTH_PROVIDER_ID,
                 SOURCE_API_KEY, SOURCE_PKCE),
        default=ORCAROUTER_API_PROVIDER_ID,
        help="which authentication choice to clear",
    )
    clear.set_defaults(func=cmd_clear)

    models = sub.add_parser("models", help="list the live model catalog")
    models.add_argument("--capability", default=CAPABILITY_CHAT)
    models.add_argument("--modalities", default="", help="e.g. image")
    models.set_defaults(func=cmd_models)

    chat = sub.add_parser("chat", help="send one real completion")
    chat.add_argument("--model", required=True)
    chat.add_argument("--prompt", required=True)
    chat.set_defaults(func=cmd_chat)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except CredentialError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except OrcaRouterError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
