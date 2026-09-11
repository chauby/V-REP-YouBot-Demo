"""Launch the OrcaRouter settings UI (loopback only)."""

from __future__ import annotations

import argparse
import sys
import webbrowser

from ..config import resolve_origins
from ..credentials import CredentialStore
from ..web.server import OrcaRouterUIServer
from ..web.service import OrcaRouterService


def build_service(store_path: str | None = None) -> OrcaRouterService:
    return OrcaRouterService(
        store=CredentialStore(store_path),
        origins=resolve_origins(),
        open_browser=webbrowser.open,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="orcarouter-ui",
        description="OrcaRouter provider settings for the V-REP YouBot demos.",
    )
    parser.add_argument("--port", type=int, default=0, help="0 picks a free port")
    parser.add_argument("--store", default=None, help="credential file path")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)

    service = build_service(args.store)
    server = OrcaRouterUIServer(service, port=args.port)
    url = f"http://127.0.0.1:{server.port}/"
    print(f"OrcaRouter settings: {url}")
    print(f"  auth origin: {service.origins.auth_base}")
    print(f"  api origin:  {service.origins.api_base}")
    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
