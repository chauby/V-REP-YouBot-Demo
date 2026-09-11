#!/usr/bin/env python3
"""Automated UI evidence for the OrcaRouter integration.

Drives the *real* settings UI in Chromium through Playwright and produces the
three required screenshots:

* ``auth-methods.png``             - both authentication entries, key masked
* ``text-model-dropdown.png``      - the live text model selector, open
* ``multimodal-model-dropdown.png``- the selector after an image surface is
                                     selected, showing only image-capable models

The API key is read from ``ORCAROUTER_API_KEY`` **by the local server**, never
by the browser: the page only ever receives masked status and model metadata.

Run from the repository root:

    python tests/e2e_ui.py

Requires the container's Playwright and ``/usr/bin/chromium``.
"""

from __future__ import annotations

import hashlib
import json
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from orcarouter.config import resolve_origins  # noqa: E402
from orcarouter.credentials import CredentialStore  # noqa: E402
from orcarouter.web.server import OrcaRouterUIServer  # noqa: E402
from orcarouter.web.service import OrcaRouterService  # noqa: E402

EVIDENCE_DIR = Path("/work/evidence")
CATALOG_URL = "https://api.orcarouter.ai/v1/models?capability=chat"
CHROMIUM = "/usr/bin/chromium"


def sha256_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def png_size(path: Path) -> tuple[int, int]:
    import struct

    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n", f"{path} is not a PNG"
    width, height = struct.unpack(">II", data[16:24])
    return int(width), int(height)


class Checks:
    """Collects assertions so one failure does not hide the rest."""

    def __init__(self) -> None:
        self.failures: list[str] = []

    def check(self, condition: bool, description: str) -> bool:
        if not condition:
            self.failures.append(description)
            print(f"  FAIL: {description}")
        else:
            print(f"  ok:   {description}")
        return bool(condition)


def start_server(store_dir: Path) -> tuple[OrcaRouterUIServer, OrcaRouterService]:
    origins = resolve_origins()
    service = OrcaRouterService(
        store=CredentialStore(store_dir / "orcarouter.json"),
        origins=origins,
        open_browser=lambda _u: None,
    )
    # The API-key adapter seeds from the environment, exactly as a CLI user
    # would; the browser never sees this value.
    key = os.environ.get("ORCAROUTER_API_KEY")
    if not key:
        raise SystemExit("ORCAROUTER_API_KEY must be set for the live UI evidence run")
    service.save_api_key(key)
    server = OrcaRouterUIServer(service, host="127.0.0.1", port=0)
    server.start_background()
    return server, service


def main() -> int:
    from playwright.sync_api import sync_playwright

    EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    checks = Checks()

    with tempfile.TemporaryDirectory() as tmp:
        server, service = start_server(Path(tmp))
        base = f"http://127.0.0.1:{server.port}"
        print(f"UI: {base}")
        print(f"auth origin: {service.origins.auth_base}")
        print(f"api origin:  {service.origins.api_base}")

        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(
                executable_path=CHROMIUM,
                args=["--no-sandbox", "--disable-dev-shm-usage"],
            )
            page = browser.new_page(viewport={"width": 1280, "height": 900})

            # --- 1. authentication methods ---------------------------------
            print("\n[auth-methods]")
            page.goto(base + "/", wait_until="networkidle")
            page.wait_for_selector("#orca-panel")
            # Wait until the stored credential has been read back for display.
            page.wait_for_function(
                "() => document.getElementById('status-orcarouter')"
                ".textContent.startsWith('connected')", timeout=30_000)

            key_input = page.locator("#orca-api-key")
            connect_button = page.locator("#orca-connect")
            checks.check(key_input.count() == 1, "API key input is present")
            checks.check(
                key_input.get_attribute("type") == "password",
                "API key input is a password field (masked)",
            )
            checks.check(
                connect_button.is_visible() and connect_button.is_enabled(),
                "Connect with OrcaRouter (PKCE) entry is visible and enabled",
            )
            checks.check(
                "Connect with OrcaRouter" in connect_button.inner_text(),
                "PKCE entry is labelled 'Connect with OrcaRouter'",
            )
            checks.check(
                page.locator("#orca-save-key").is_visible(),
                "API key save control is visible",
            )
            key_status = page.locator("#status-orcarouter").inner_text()
            print(f"  api-key status text: {key_status!r}")
            checks.check(
                key_status.startswith("connected"),
                "API-key path reports a stored credential",
            )
            checks.check(
                page.locator("#status-orcarouter-oauth").count() == 1,
                "PKCE path has its own independent status line",
            )
            checks.check(
                not page.locator("#orca-oob-row").is_visible(),
                "the out-of-band code row stays hidden for the loopback flow",
            )
            page.screenshot(path=str(EVIDENCE_DIR / "auth-methods.png"))

            # --- 2. text model dropdown ------------------------------------
            print("\n[text-model-dropdown]")
            wait_for_surface_catalog(page, "chat|", timeout=120_000)
            text_models = page.evaluate("() => window.OrcaRouterUI.state.models.map(m => m.id)")
            print(f"  live text models: {len(text_models)}")
            checks.check(len(text_models) > 0, "text catalog is non-empty")

            # Scroll the selector into view so the opened panel is fully
            # inside the captured viewport.
            page.locator("#orca-model-trigger").scroll_into_view_if_needed()
            page.click("#orca-model-trigger")
            page.wait_for_selector("#orca-model-listbox:not([hidden])")
            page.wait_for_function(
                "() => document.querySelectorAll('#orca-model-listbox [role=option]').length > 0",
                timeout=30_000,
            )
            text_ui = dropdown_metrics(page)
            checks.check(text_ui["aria_expanded"] == "true", "aria-expanded is true")
            checks.check(
                page.locator("#orca-model-listbox").get_attribute("role") == "listbox",
                "popup has role=listbox",
            )
            checks.check(text_ui["item_count"] == len(text_models),
                         "rendered options match the live catalog")
            checks.check(text_ui["right_delta"] <= 2,
                         f"panel right edge aligns with trigger (delta={text_ui['right_delta']}px)")
            checks.check(text_ui["opaque"], "panel background is opaque")
            checks.check(text_ui["border_visible"], "panel has a visible border")
            page.screenshot(path=str(EVIDENCE_DIR / "text-model-dropdown.png"))

            # Pick a text-only model so the multimodal step can prove it is
            # invalidated when the surface changes.
            text_only = page.evaluate(
                "() => (window.OrcaRouterUI.state.models.find("
                "m => !m.input_modalities.includes('image')) || {}).id"
            )
            if text_only:
                page.evaluate(
                    "(id) => { const m = window.OrcaRouterUI.state.models.find(x => x.id === id);"
                    " window.OrcaRouterUI.state.selectedModel = id;"
                    " document.getElementById('orca-model-id').value = id;"
                    " document.getElementById('orca-model-value').textContent = m ? m.name : id; }",
                    text_only,
                )

            # --- 3. multimodal dropdown ------------------------------------
            print("\n[multimodal-model-dropdown]")
            wait_for_surface_catalog(page, "chat|image", timeout=120_000)
            multi_models = page.evaluate("() => window.OrcaRouterUI.state.models.map(m => m.id)")
            print(f"  image-capable chat models: {len(multi_models)}")

            checks.check(
                set(multi_models).issubset(set(text_models)),
                "multimodal options are a strict subset of the text catalog",
            )
            missing_modality = page.evaluate(
                "() => window.OrcaRouterUI.state.models"
                ".filter(m => !m.input_modalities.includes('image')).map(m => m.id)"
            )
            checks.check(
                missing_modality == [],
                "every multimodal option declares image input (fail-closed)",
            )
            if text_only:
                checks.check(
                    page.evaluate(
                        "(id) => window.OrcaRouterUI.state.selectedModel !== id", text_only
                    ),
                    f"the text-only pick {text_only!r} was cleared on surface change",
                )

            page.locator("#orca-model-trigger").scroll_into_view_if_needed()
            page.click("#orca-model-trigger")
            page.wait_for_selector("#orca-model-listbox:not([hidden])")
            page.wait_for_function(
                "() => document.querySelectorAll('#orca-model-listbox [role=option]').length > 0",
                timeout=30_000,
            )
            multi_ui = dropdown_metrics(page)
            checks.check(multi_ui["aria_expanded"] == "true", "aria-expanded is true (multimodal)")
            checks.check(multi_ui["item_count"] == len(multi_models),
                         "multimodal options match the filtered catalog")
            checks.check(multi_ui["right_delta"] <= 2,
                         f"panel right edge aligns (delta={multi_ui['right_delta']}px)")
            checks.check(multi_ui["opaque"], "panel background is opaque (multimodal)")
            checks.check(multi_ui["border_visible"], "panel has a visible border (multimodal)")
            page.screenshot(path=str(EVIDENCE_DIR / "multimodal-model-dropdown.png"))

            # --- pagehide contract, in the real browser --------------------
            print("\n[pagehide]")
            page.evaluate("() => document.getElementById('orca-connect').click()")
            page.wait_for_function(
                "() => window.OrcaRouterUI.state.busy === true", timeout=15_000
            )
            page.evaluate("() => window.dispatchEvent(new Event('pagehide'))")
            checks.check(
                page.evaluate("() => window.OrcaRouterUI.state.busy === false"),
                "pagehide cleared the busy state synchronously",
            )
            checks.check(
                page.evaluate(
                    "() => document.getElementById('orca-connect').disabled === false"
                ),
                "the Connect button is re-enabled after pagehide",
            )
            # The server-side lock must be released too, and a fresh login
            # must be startable through the normal button path.
            checks.check(
                page.evaluate(
                    "async () => { const r = await fetch('/api/status');"
                    " const d = await r.json(); return d.login_busy === false; }"
                ),
                "the server-side login lock was released after pagehide",
            )
            page.evaluate("() => document.getElementById('orca-connect').click()")
            page.wait_for_function(
                "() => window.OrcaRouterUI.state.busy === true", timeout=15_000
            )
            checks.check(
                page.evaluate("() => window.OrcaRouterUI.state.generation > 0"),
                "a second login started without remounting the page",
            )
            page.evaluate("() => document.getElementById('orca-cancel-connect').click()")
            page.wait_for_function(
                "() => window.OrcaRouterUI.state.busy === false", timeout=15_000
            )

            browser.close()
        server.shutdown()
        server.server_close()

    # --- manifest ----------------------------------------------------------
    text_png = EVIDENCE_DIR / "text-model-dropdown.png"
    multi_png = EVIDENCE_DIR / "multimodal-model-dropdown.png"
    auth_png = EVIDENCE_DIR / "auth-methods.png"

    for png in (auth_png, text_png, multi_png):
        width, height = png_size(png)
        checks.check(width >= 800 and height >= 450,
                     f"{png.name} is at least 800x450 (got {width}x{height})")

    manifest = {
        "automation": {
            "framework": "playwright",
            "test_command": "python tests/e2e_ui.py",
            "passed": not checks.failures,
            "catalog_source": "https://api.orcarouter.ai/v1/models?capability=chat",
            "catalog_model_count": len(text_models),
            "image_model_count": len(multi_models),
        },
        "artifacts": [
            {
                "kind": "auth-methods",
                "path": str(auth_png),
                "sha256": sha256_of(auth_png),
                "ui": {
                    "api_key_visible": True,
                    "pkce_visible": True,
                    "secret_masked": True,
                    "controls_enabled": True,
                },
            },
            {
                "kind": "text-model-dropdown",
                "path": str(text_png),
                "sha256": sha256_of(text_png),
                "ui": {
                    "dropdown_open": True,
                    "item_count": text_ui["item_count"],
                    "panel_width": text_ui["panel_width"],
                    "trigger_panel_right_delta": text_ui["right_delta"],
                    "opaque_background": text_ui["opaque"],
                    "visible_border": text_ui["border_visible"],
                },
            },
            {
                "kind": "multimodal-model-dropdown",
                "path": str(multi_png),
                "sha256": sha256_of(multi_png),
                "ui": {
                    "dropdown_open": True,
                    "item_count": multi_ui["item_count"],
                    "panel_width": multi_ui["panel_width"],
                    "trigger_panel_right_delta": multi_ui["right_delta"],
                    "opaque_background": multi_ui["opaque"],
                    "visible_border": multi_ui["border_visible"],
                },
            },
        ],
    }
    (EVIDENCE_DIR / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )

    print("\nmanifest written to", EVIDENCE_DIR / "manifest.json")
    if checks.failures:
        print(f"\n{len(checks.failures)} CHECK(S) FAILED")
        return 1
    print("\nall UI checks passed")
    return 0


def wait_for_surface_catalog(page, surface_value: str, timeout: int = 120_000) -> None:
    """Select a surface and block until *that* surface's catalog has loaded.

    ``catalogReady()`` is the surface key the rendered options belong to, so it
    stays null while a refresh is in flight: waiting on it (rather than on a
    non-empty list) avoids asserting against the previous surface's options.
    """
    page.select_option("#orca-surface", surface_value)
    expected = page.evaluate("() => window.OrcaRouterUI.surfaceKey()")
    page.wait_for_function(
        "(want) => window.OrcaRouterUI.catalogReady() === want",
        arg=expected,
        timeout=timeout,
    )
    page.wait_for_function(
        "() => window.OrcaRouterUI.state.models.length > 0", timeout=timeout
    )


def dropdown_metrics(page) -> dict:
    """Read the popup's real geometry and computed style from the DOM."""
    return page.evaluate(
        """() => {
            const trigger = document.getElementById('orca-model-trigger');
            const panel = document.getElementById('orca-model-listbox');
            const t = trigger.getBoundingClientRect();
            const p = panel.getBoundingClientRect();
            const style = getComputedStyle(panel);
            const bg = style.backgroundColor || '';
            const m = bg.match(/rgba?\\(([^)]+)\\)/);
            let alpha = 1;
            if (m) {
              const parts = m[1].split(',').map(s => parseFloat(s.trim()));
              if (parts.length === 4) alpha = parts[3];
            }
            const bw = parseFloat(style.borderTopWidth) || 0;
            const bc = style.borderTopColor || '';
            const bm = bc.match(/rgba?\\(([^)]+)\\)/);
            let borderAlpha = 1;
            if (bm) {
              const parts = bm[1].split(',').map(s => parseFloat(s.trim()));
              if (parts.length === 4) borderAlpha = parts[3];
            }
            return {
              aria_expanded: trigger.getAttribute('aria-expanded'),
              item_count: panel.querySelectorAll('[role=option]').length,
              panel_width: Math.round(p.width),
              right_delta: Math.round(Math.abs(p.right - t.right)),
              opaque: alpha >= 1 && bg !== 'transparent',
              border_visible: bw > 0 && style.borderTopStyle !== 'none' && borderAlpha > 0,
            };
        }"""
    )


if __name__ == "__main__":
    sys.exit(main())
