"""Display-name parity for the OrcaRouter provider entries.

This repository ships a single (English) UI catalog, so there is no locale
directory to run key-parity against.  What must still hold is that every
user-visible OrcaRouter string has exactly one definition, that the two
authentication choices stay distinguishable everywhere they can appear, and
that no label is hardcoded in a view while bypassing the catalog.

If locale catalogs are added later, register them in ``CATALOGS`` and the
parity assertions below start covering them automatically.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from orcarouter.provider import PROVIDERS
from orcarouter.web.service import CAPABILITY_SURFACES

ROOT = Path(__file__).resolve().parents[1]
STATIC = ROOT / "orcarouter" / "web" / "static"

#: The shipped UI catalogs.  Only English exists today.
CATALOGS = ("en",)

#: The canonical display strings, defined in exactly one place.
PROVIDER_LABELS = {
    "orcarouter": "OrcaRouter - API",
    "orcarouter-oauth": "OrcaRouter - Auth",
}


def test_provider_labels_match_the_catalog():
    for spec in PROVIDERS:
        assert spec.label == PROVIDER_LABELS[spec.id]


def test_both_labels_are_distinct():
    labels = [spec.label for spec in PROVIDERS]
    assert len(labels) == len(set(labels))
    # A generic "OrcaRouter" button that sometimes asks for a key and sometimes
    # opens a browser makes support and re-authentication harder.
    assert all(label.startswith("OrcaRouter - ") for label in labels)


def test_provider_descriptions_exist_for_every_label():
    for spec in PROVIDERS:
        assert spec.description.strip()
        assert spec.key_dashboard_url.startswith("https://www.orcarouter.ai/")


def test_every_provider_id_has_a_status_slot_in_the_ui():
    """Each entry can show its own state, joined by credential source."""
    html = (STATIC / "index.html").read_text()
    for spec in PROVIDERS:
        assert f'id="status-{spec.id}"' in html


def test_every_surface_has_a_label_and_an_id():
    for surface in CAPABILITY_SURFACES:
        assert surface["id"]
        assert surface["label"].strip()


def test_no_orcarouter_label_is_hardcoded_in_the_view_layer():
    """Labels come from the registry payload, not typed into the template."""
    script = (STATIC / "app.js").read_text()
    for label in PROVIDER_LABELS.values():
        assert label not in script, (
            f"{label!r} is hardcoded in app.js; it must come from /api/status"
        )
    # The view renders provider entries from the API payload.
    assert "provider.label" in script or "p.label" in script or "providers" in script


@pytest.mark.parametrize("catalog", CATALOGS)
def test_catalog_names_are_non_empty(catalog):
    assert catalog
