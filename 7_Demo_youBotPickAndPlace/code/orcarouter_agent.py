"""OrcaRouter-backed task planner for the youBot pick-and-place demo.

This is the demo's AI entry point.  It is wired to the OrcaRouter provider so
that either authentication choice works:

* ``OrcaRouter - API``  -- an existing ``sk-orca-…`` key (env var or credential file)
* ``OrcaRouter - Auth`` -- a key issued by the OAuth 2.0 + PKCE connect flow

Nothing here knows which one was used: both produce the same credential and the
same inference client, and the model selector is filled from the live catalog.

Run it from the repository root:

    python 7_Demo_youBotPickAndPlace/code/orcarouter_agent.py --list-models
    python 7_Demo_youBotPickAndPlace/code/orcarouter_agent.py --prompt "Plan a grasp"
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from orcarouter.catalog import CAPABILITY_CHAT, CatalogCache  # noqa: E402
from orcarouter.config import resolve_origins  # noqa: E402
from orcarouter.credentials import (  # noqa: E402
    SOURCE_API_KEY,
    SOURCE_PKCE,
    CredentialStore,
    mask_secret,
)
from orcarouter.errors import OrcaRouterError, TerminalReauthRequired  # noqa: E402
from orcarouter.lifecycle import CredentialLifecycle, ReauthState  # noqa: E402
from orcarouter.provider import OrcaRouterClient  # noqa: E402

SYSTEM_PROMPT = (
    "You are the task planner for a KUKA youBot mobile manipulator simulated in "
    "CoppeliaSim. Given a request, reply with a short ordered list of motion "
    "steps naming the chassis, arm and gripper actions."
)


def resolve_credential(store: CredentialStore, lifecycle: CredentialLifecycle):
    """Return the first usable credential, whichever adapter produced it."""
    for source_id in (SOURCE_API_KEY, SOURCE_PKCE):
        status = lifecycle.status(source_id)
        if status.state is ReauthState.READY:
            return lifecycle.credential_for(source_id)
        if status.state is ReauthState.NEEDS_REAUTH:
            print(
                f"note: {source_id} needs reauthentication "
                f"(`python -m orcarouter.cli connect`)",
                file=sys.stderr,
            )
    raise SystemExit(
        "no usable OrcaRouter credential; run `python -m orcarouter.cli connect` "
        "or `python -m orcarouter.cli set-key`"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prompt", default="Plan how to pick up the can at point A "
                                           "and place it on the table at point B.")
    parser.add_argument("--model", default=None,
                        help="model id; defaults to the first compatible live model")
    parser.add_argument("--list-models", action="store_true")
    parser.add_argument("--multimodal", metavar="IMAGE_PATH", default=None,
                        help="attach an image to the request")
    parser.add_argument("--store", default=None)
    args = parser.parse_args(argv)

    origins = resolve_origins()
    store = CredentialStore(args.store)
    lifecycle = CredentialLifecycle(store)
    credential = resolve_credential(store, lifecycle)
    print(f"using credential {mask_secret(credential.api_key)} "
          f"from {credential.source} (inference at {origins.api_base})",
          file=sys.stderr)

    required = ("image",) if args.multimodal else ()
    catalog = CatalogCache().refresh(
        CAPABILITY_CHAT,
        required,
        origins=origins,
        api_key=credential.api_key,
    )
    if catalog.degraded:
        print(f"catalog: {catalog.detail} (source={catalog.source})", file=sys.stderr)

    if args.list_models:
        for model in catalog.models:
            print(f"{model.id}\t{model.name}")
        return 0

    if not catalog.models:
        raise SystemExit("no compatible model for this request")

    model = args.model or catalog.models[0].id
    if model not in catalog.ids:
        # The selected model is not compatible with the current surface: fail
        # loudly instead of silently sending an unsupported request.
        raise SystemExit(
            f"{model!r} is not available for this surface; "
            f"pick one of: {', '.join(catalog.ids[:8])}"
        )

    content: object = args.prompt
    if args.multimodal:
        content = [
            {"type": "text", "text": args.prompt},
            {"type": "image_url", "image_url": {"url": args.multimodal}},
        ]

    client = OrcaRouterClient(credential, origins=origins)
    try:
        payload = client.chat_completion(
            model,
            [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": content},
            ],
        )
    except TerminalReauthRequired:
        lifecycle.mark_rejected(credential)
        raise SystemExit(
            "OrcaRouter rejected this credential (401). A durable key cannot be "
            "refreshed; reconnect the account to issue a new one."
        )

    choice = (payload.get("choices") or [{}])[0]
    print((choice.get("message") or {}).get("content", json.dumps(payload)[:2000]))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except OrcaRouterError as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
