"""Test isolation.

The suite must never reach the network and must never pick up an ambient
credential from the environment: an ``ORCAROUTER_API_KEY`` sitting in the
developer's shell would otherwise silently turn unit tests into live calls.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

#: Environment variables the provider reads.  All of them are cleared for the
#: whole session and restored afterwards.
PROVIDER_ENV = (
    "ORCAROUTER_API_KEY",
    "ORCA_BASE_URL",
    "ORCA_AUTH_BASE_URL",
    "ORCA_API_BASE_URL",
    "XDG_CONFIG_HOME",
)


@pytest.fixture(autouse=True)
def isolated_environment(monkeypatch, tmp_path):
    """Remove ambient provider configuration for every test."""
    for name in PROVIDER_ENV:
        monkeypatch.delenv(name, raising=False)
    # Keep the default credential store out of the developer's home directory.
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "config"))
    yield


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    """Fail loudly if any test tries to open a real socket."""

    import socket

    real_connect = socket.socket.connect
    real_create = socket.create_connection

    def guard(self, address, *args, **kwargs):
        host = address[0] if isinstance(address, tuple) else address
        if host not in ("127.0.0.1", "::1", "localhost"):
            raise AssertionError(f"test attempted a non-loopback connection to {host}")
        return real_connect(self, address, *args, **kwargs)

    def guard_create(address, *args, **kwargs):
        host = address[0] if isinstance(address, tuple) else address
        if host not in ("127.0.0.1", "::1", "localhost"):
            raise AssertionError(f"test attempted a non-loopback connection to {host}")
        return real_create(address, *args, **kwargs)

    monkeypatch.setattr(socket.socket, "connect", guard)
    monkeypatch.setattr(socket, "create_connection", guard_create)
    yield


def pytest_configure(config):
    if os.environ.get("ORCAROUTER_API_KEY"):
        # Not fatal, but make it visible that live credentials are present.
        config.option.tbstyle = getattr(config.option, "tbstyle", "auto")
